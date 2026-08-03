#![no_std]
#![no_main]
#![allow(dead_code)]

#[path = "../src/can/command.rs"]
mod command;
#[path = "../src/constants.rs"]
mod constants;
#[allow(clippy::wrong_self_convention)]
#[path = "../src/payload.rs"]
mod payload;

use command::GroundCommand;
use constants::LORA_TRANSMIT_INTERVAL_MS;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart, UartRx, UartTx},
};
use esp_println::println;
use payload::{PAYLOAD_LEN, Payload};

esp_bootloader_esp_idf::esp_app_desc!();

const UART_BAUD: u32 = 115_200;
const RESPONSE_INTER_BYTE_TIMEOUT_MS: u64 = 20;
const SUMMARY_INTERVAL: u32 = 100;
const EVENT_QUEUE_DEPTH: usize = 16;
const LOG_QUEUE_DEPTH: usize = 8;

#[derive(Clone, Copy)]
enum TestEvent {
    AuxFall {
        at_us: u64,
    },
    AuxRise {
        at_us: u64,
    },
    UplinkValid {
        command: u8,
        sequence: u8,
        first_us: u64,
        last_us: u64,
    },
    ChecksumError,
    LengthError,
    UnknownCommand,
}

#[derive(Clone, Copy)]
struct TestRecord {
    dl_seq: u8,
    dl_schedule_us: u64,
    dl_uart_start_us: u64,
    dl_uart_end_us: u64,
    dl_aux_fall_us: Option<u64>,
    dl_aux_rise_us: Option<u64>,
    ul_command: Option<u8>,
    ul_seq: Option<u8>,
    ul_first_us: Option<u64>,
    ul_last_us: Option<u64>,
    next_dl_schedule_us: u64,
    tx_ok: bool,
}

impl TestRecord {
    fn new(
        dl_seq: u8,
        dl_schedule_us: u64,
        dl_uart_start_us: u64,
        dl_uart_end_us: u64,
        next_dl_schedule_us: u64,
        tx_ok: bool,
    ) -> Self {
        Self {
            dl_seq,
            dl_schedule_us,
            dl_uart_start_us,
            dl_uart_end_us,
            dl_aux_fall_us: None,
            dl_aux_rise_us: None,
            ul_command: None,
            ul_seq: None,
            ul_first_us: None,
            ul_last_us: None,
            next_dl_schedule_us,
            tx_ok,
        }
    }
}

#[derive(Clone, Copy)]
enum LogEvent {
    Record(TestRecord),
    ChecksumError,
    LengthError,
    UnknownCommand,
    SequenceMissing(u32),
    SequenceDuplicate,
}

static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, TestEvent, EVENT_QUEUE_DEPTH> =
    Channel::new();
static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogEvent, LOG_QUEUE_DEPTH> = Channel::new();

fn now_us() -> u64 {
    Instant::now().as_micros()
}

fn build_downlink(sequence: u8) -> [u8; PAYLOAD_LEN] {
    let mut payload = Payload::new();
    payload.status = sequence;
    payload.gnss_lat = 0x1234_5678;
    payload.gnss_long = -0x1234_5678;
    payload.gnss_height = 123;
    payload.angle_speed = [1, -2, 3];
    payload.acceleration = [100, -200, 300];
    payload.integrated_angle = [10, 20, 30];
    payload.air_pressure = [0x11, 0x22, 0x33];
    payload.air_speed = 0x44;
    payload.fin_angle = -5;
    payload.check_sum = payload.calculate_checksum();
    payload.to_bytes()
}

async fn write_all(tx: &mut UartTx<'static, Async>, mut bytes: &[u8]) -> bool {
    while !bytes.is_empty() {
        match tx.write_async(bytes).await {
            Ok(0) => return false,
            Ok(written) => bytes = &bytes[written..],
            Err(_) => return false,
        }
    }
    tx.flush_async().await.is_ok()
}

#[embassy_executor::task]
async fn uplink_rx_task(mut rx: UartRx<'static, Async>) {
    let mut frame = [0u8; 3];
    let mut len = 0usize;
    let mut first_us = 0u64;

    loop {
        let mut byte = [0u8; 1];
        let read = if len == 0 {
            rx.read_async(&mut byte).await.map_err(|_| ())
        } else {
            match with_timeout(
                Duration::from_millis(RESPONSE_INTER_BYTE_TIMEOUT_MS),
                rx.read_async(&mut byte),
            )
            .await
            {
                Ok(result) => result.map_err(|_| ()),
                Err(_) => {
                    len = 0;
                    EVENT_CHANNEL.send(TestEvent::LengthError).await;
                    continue;
                }
            }
        };

        match read {
            Ok(1) => {
                let received_at = now_us();
                if len == 0 {
                    first_us = received_at;
                }
                frame[len] = byte[0];
                len += 1;
                if len < frame.len() {
                    continue;
                }

                len = 0;
                let [command, sequence, checksum] = frame;
                if checksum != command ^ sequence {
                    EVENT_CHANNEL.send(TestEvent::ChecksumError).await;
                } else if GroundCommand::decode_legacy(command).is_none() {
                    EVENT_CHANNEL.send(TestEvent::UnknownCommand).await;
                } else {
                    EVENT_CHANNEL
                        .send(TestEvent::UplinkValid {
                            command,
                            sequence,
                            first_us,
                            last_us: received_at,
                        })
                        .await;
                }
            }
            Ok(_) | Err(()) => {
                if len != 0 {
                    len = 0;
                    EVENT_CHANNEL.send(TestEvent::LengthError).await;
                }
                Timer::after_millis(1).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn aux_monitor_task(mut aux: Input<'static>) {
    if aux.is_low() {
        EVENT_CHANNEL
            .send(TestEvent::AuxFall { at_us: now_us() })
            .await;
    }

    loop {
        if aux.is_high() {
            aux.wait_for_falling_edge().await;
            EVENT_CHANNEL
                .send(TestEvent::AuxFall { at_us: now_us() })
                .await;
        } else {
            aux.wait_for_rising_edge().await;
            EVENT_CHANNEL
                .send(TestEvent::AuxRise { at_us: now_us() })
                .await;
        }
    }
}

struct Aggregate {
    count: u64,
    min: u64,
    max: u64,
    sum: u128,
}

impl Aggregate {
    const fn new() -> Self {
        Self {
            count: 0,
            min: u64::MAX,
            max: 0,
            sum: 0,
        }
    }

    fn add(&mut self, value: u64) {
        self.count = self.count.saturating_add(1);
        self.min = self.min.min(value);
        self.max = self.max.max(value);
        self.sum = self.sum.saturating_add(value as u128);
    }

    fn min_or_zero(&self) -> u64 {
        if self.count == 0 { 0 } else { self.min }
    }

    fn average(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            (self.sum / self.count as u128) as u64
        }
    }
}

#[embassy_executor::task]
async fn logger_task() {
    let mut downlinks = 0u64;
    let mut received = 0u64;
    let mut checksum_errors = 0u64;
    let mut length_errors = 0u64;
    let mut unknown_commands = 0u64;
    let mut sequence_missing = 0u64;
    let mut sequence_duplicates = 0u64;
    let mut rx_offset = Aggregate::new();
    let mut guard = Aggregate::new();
    let mut aux_low = Aggregate::new();
    let mut max_lateness = 0u64;

    println!(
        "TEST,dl_seq,dl_schedule_us,dl_uart_start_us,dl_uart_end_us,aux_fall_us,aux_rise_us,ul_received,ul_command,ul_seq,ul_first_us,ul_last_us,next_dl_schedule_us,uart_tx_duration_us,command_rx_offset_us,guard_us,schedule_lateness_us,checksum_ok,tx_ok"
    );

    loop {
        match LOG_CHANNEL.receive().await {
            LogEvent::ChecksumError => checksum_errors = checksum_errors.saturating_add(1),
            LogEvent::LengthError => length_errors = length_errors.saturating_add(1),
            LogEvent::UnknownCommand => unknown_commands = unknown_commands.saturating_add(1),
            LogEvent::SequenceMissing(count) => {
                sequence_missing = sequence_missing.saturating_add(count as u64)
            }
            LogEvent::SequenceDuplicate => {
                sequence_duplicates = sequence_duplicates.saturating_add(1)
            }
            LogEvent::Record(record) => {
                downlinks = downlinks.saturating_add(1);
                let uart_duration = record
                    .dl_uart_end_us
                    .saturating_sub(record.dl_uart_start_us);
                let lateness = record
                    .dl_uart_start_us
                    .saturating_sub(record.dl_schedule_us);
                max_lateness = max_lateness.max(lateness);

                let ul_received = record.ul_last_us.is_some();
                let command_rx_offset = record
                    .ul_last_us
                    .map(|at| at.saturating_sub(record.dl_uart_start_us));
                let guard_us = record
                    .ul_last_us
                    .map(|at| record.next_dl_schedule_us.saturating_sub(at));
                let aux_duration = match (record.dl_aux_fall_us, record.dl_aux_rise_us) {
                    (Some(fall), Some(rise)) if rise >= fall => Some(rise - fall),
                    _ => None,
                };

                if let Some(value) = command_rx_offset {
                    received = received.saturating_add(1);
                    rx_offset.add(value);
                }
                if let Some(value) = guard_us {
                    guard.add(value);
                }
                if let Some(value) = aux_duration {
                    aux_low.add(value);
                }

                println!(
                    "TEST,{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                    record.dl_seq,
                    record.dl_schedule_us,
                    record.dl_uart_start_us,
                    record.dl_uart_end_us,
                    record.dl_aux_fall_us.unwrap_or(0),
                    record.dl_aux_rise_us.unwrap_or(0),
                    ul_received as u8,
                    record.ul_command.unwrap_or(0),
                    record.ul_seq.unwrap_or(0),
                    record.ul_first_us.unwrap_or(0),
                    record.ul_last_us.unwrap_or(0),
                    record.next_dl_schedule_us,
                    uart_duration,
                    command_rx_offset.unwrap_or(0),
                    guard_us.unwrap_or(0),
                    lateness,
                    ul_received as u8,
                    record.tx_ok as u8,
                );

                if downlinks.is_multiple_of(SUMMARY_INTERVAL as u64) {
                    let not_received = downlinks.saturating_sub(received);
                    let success_basis_points = received.saturating_mul(10_000) / downlinks;
                    println!(
                        "SUMMARY,tx={},rx={},timeout={},checksum_error={},length_error={},unknown_command={},success_bp={},rx_offset_min={},rx_offset_max={},rx_offset_avg={},guard_min={},guard_max={},guard_avg={},lateness_max={},aux_low_min={},aux_low_max={},aux_low_avg={},seq_missing={},seq_duplicate={}",
                        downlinks,
                        received,
                        not_received,
                        checksum_errors,
                        length_errors,
                        unknown_commands,
                        success_basis_points,
                        rx_offset.min_or_zero(),
                        rx_offset.max,
                        rx_offset.average(),
                        guard.min_or_zero(),
                        guard.max,
                        guard.average(),
                        max_lateness,
                        aux_low.min_or_zero(),
                        aux_low.max,
                        aux_low.average(),
                        sequence_missing,
                        sequence_duplicates,
                    );
                }
            }
        }
    }
}

async fn run_test(mut tx: UartTx<'static, Async>) -> ! {
    let period = Duration::from_millis(LORA_TRANSMIT_INTERVAL_MS);
    let mut next_tx = Instant::now() + Duration::from_secs(1);
    let mut sequence = 0u8;
    let mut active_record: Option<TestRecord> = None;
    let mut aux_is_low = false;
    let mut last_aux_fall_us = None;
    let mut last_rx_sequence = None;

    loop {
        match select(Timer::at(next_tx), EVENT_CHANNEL.receive()).await {
            Either::First(_) => {
                if let Some(record) = active_record.take() {
                    LOG_CHANNEL.send(LogEvent::Record(record)).await;
                }

                let schedule_us = next_tx.as_micros();
                next_tx += period;
                let next_schedule_us = next_tx.as_micros();
                let uart_start_us = now_us();
                let bytes = build_downlink(sequence);
                let tx_ok = write_all(&mut tx, &bytes).await;
                let uart_end_us = now_us();
                let mut record = TestRecord::new(
                    sequence,
                    schedule_us,
                    uart_start_us,
                    uart_end_us,
                    next_schedule_us,
                    tx_ok,
                );
                if aux_is_low {
                    record.dl_aux_fall_us = last_aux_fall_us;
                }
                active_record = Some(record);
                sequence = sequence.wrapping_add(1);
            }
            Either::Second(event) => match event {
                TestEvent::AuxFall { at_us } => {
                    aux_is_low = true;
                    last_aux_fall_us = Some(at_us);
                    if let Some(record) = active_record.as_mut()
                        && at_us >= record.dl_uart_start_us
                        && at_us < record.next_dl_schedule_us
                    {
                        record.dl_aux_fall_us = Some(at_us);
                    }
                }
                TestEvent::AuxRise { at_us } => {
                    aux_is_low = false;
                    if let Some(record) = active_record.as_mut()
                        && at_us >= record.dl_uart_start_us
                        && at_us < record.next_dl_schedule_us
                    {
                        record.dl_aux_rise_us = Some(at_us);
                    }
                }
                TestEvent::ChecksumError => {
                    LOG_CHANNEL.send(LogEvent::ChecksumError).await;
                }
                TestEvent::LengthError => {
                    LOG_CHANNEL.send(LogEvent::LengthError).await;
                }
                TestEvent::UnknownCommand => {
                    LOG_CHANNEL.send(LogEvent::UnknownCommand).await;
                }
                TestEvent::UplinkValid {
                    command,
                    sequence: received_sequence,
                    first_us,
                    last_us,
                } => {
                    if let Some(previous_sequence) = last_rx_sequence {
                        let delta = received_sequence.wrapping_sub(previous_sequence);
                        if delta == 0 {
                            LOG_CHANNEL.send(LogEvent::SequenceDuplicate).await;
                        } else if delta > 1 {
                            LOG_CHANNEL
                                .send(LogEvent::SequenceMissing((delta - 1) as u32))
                                .await;
                        }
                    }
                    last_rx_sequence = Some(received_sequence);

                    if let Some(record) = active_record.as_mut()
                        && received_sequence == record.dl_seq
                        && record.ul_last_us.is_none()
                    {
                        record.ul_command = Some(command);
                        record.ul_seq = Some(received_sequence);
                        record.ul_first_us = Some(first_us);
                        record.ul_last_us = Some(last_us);
                    }
                }
            },
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let lora_tx = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let lora_rx = Input::new(peripherals.GPIO12, InputConfig::default());
    let aux = Input::new(peripherals.GPIO8, InputConfig::default());
    let mut m0 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let mut m1 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    m0.set_low();
    m1.set_low();

    let uart_config = UartConfig::default()
        .with_baudrate(UART_BAUD)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let uart = match Uart::new(peripherals.UART2, uart_config) {
        Ok(uart) => uart.with_rx(lora_rx).with_tx(lora_tx).into_async(),
        Err(error) => {
            println!("LoRa test UART init error: {:?}", error);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    let (rx, tx) = uart.split();

    match uplink_rx_task(rx) {
        Ok(token) => spawner.spawn(token),
        Err(_) => println!("LoRa test RX task spawn failed"),
    }
    match aux_monitor_task(aux) {
        Ok(token) => spawner.spawn(token),
        Err(_) => println!("LoRa test AUX task spawn failed"),
    }
    match logger_task() {
        Ok(token) => spawner.spawn(token),
        Err(_) => println!("LoRa test logger task spawn failed"),
    }

    println!(
        "LoRa roundtrip TX test: ESP32-S3 UART2 TX=GPIO11 RX=GPIO12 AUX=GPIO8 M0=GPIO9 LOW M1=GPIO10 LOW baud={} period_ms={} payload_len={} destination=0000 channel=04 uplink=command,sequence,command_xor_sequence",
        UART_BAUD, LORA_TRANSMIT_INTERVAL_MS, PAYLOAD_LEN,
    );
    run_test(tx).await
}
