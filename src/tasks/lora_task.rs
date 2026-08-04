use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_hal::{
    Async,
    gpio::Input,
    uart::{RxError, TxError, UartRx, UartTx},
};
use esp_println::println;

use crate::{
    can::{
        command::{
            CommandCategory, CommandPriority, CommandRequestState, GroundCommand,
            command_already_satisfied, may_supersede,
        },
        protocol::CanTxMessage,
    },
    constants::{LORA_AUX_TIMEOUT_MS, LORA_RX_TX_GUARD_MS, LORA_TRANSMIT_INTERVAL_MS},
    lora_uplink::UplinkFrameBuffer,
    state::{
        CAN_SAFETY_TX_SIGNAL, CAN_TX_CHANNEL, COMMAND_REQUEST_FAILURE_COUNT, COMMAND_REQUEST_STATE,
        CONTROLLER_STATUS_RAW, CanTxRequest, GNSS_CMD_CHANNEL, GnssCommand,
        HAS_VALID_CONTROLLER_STATUS, LAST_COMMAND_FAILURE, LATEST_LOGGING_GENERATION,
        LATEST_PARA_POSITION_GENERATION, LATEST_SEQUENCE_GENERATION, LOGGING_REQUESTED,
        LORA_AUX_TIMEOUT_COUNT, LORA_COMMAND_DROP_COUNT, LORA_RX_ERROR_COUNT, LORA_TX_ERROR_COUNT,
        PAYLOAD_MUTEX, RECEIVED_DATA_CHANNEL, SD_FLUSH_SIGNAL,
    },
};

#[cfg(feature = "lora-timing-debug")]
use crate::lora_timing::{
    AUX_DONE_INTERVAL, AUX_LOW_DELAY, AUX_LOW_DURATION, IDLE_GAP, INITIAL_AUX_WAIT,
    PAYLOAD_PREPARE, POST_GUARD_AUX_WAIT, POST_UART_TO_DONE, PRE_AUX_WAIT, REQUEST_INTERVAL,
    RX_GUARD_WAIT, RequestSource, TX_TOTAL, TimingCollector, TimingReport, TxTimingTrace,
    UART_FLUSH, UART_WRITE, WRITE_START_INTERVAL,
};

static LORA_RX_ACTIVITY_SIGNAL: Signal<CriticalSectionRawMutex, Instant> = Signal::new();

#[cfg(feature = "lora-timing-debug")]
const AUX_LOW_OBSERVE_TIMEOUT_MS: u64 = 15;

#[cfg(feature = "lora-timing-debug")]
fn now_us() -> u64 {
    Instant::now().as_micros()
}

#[cfg(feature = "lora-timing-debug")]
fn print_timing_report(report: TimingReport) {
    macro_rules! print_metric {
        ($name:literal, $index:expr) => {{
            let metric = report.metrics[$index];
            println!(
                concat!("  ", $name, ": count={} min={} max={} avg={}"),
                metric.count, metric.min, metric.max, metric.average
            );
        }};
    }

    println!("LoRa TX timing us, samples=10");
    print_metric!("request_interval", REQUEST_INTERVAL);
    print_metric!("pre_aux_wait", PRE_AUX_WAIT);
    print_metric!("initial_aux_wait", INITIAL_AUX_WAIT);
    print_metric!("rx_guard_wait", RX_GUARD_WAIT);
    print_metric!("post_guard_aux_wait", POST_GUARD_AUX_WAIT);
    print_metric!("payload_prepare", PAYLOAD_PREPARE);
    print_metric!("uart_write", UART_WRITE);
    print_metric!("uart_flush", UART_FLUSH);
    print_metric!("aux_low_delay", AUX_LOW_DELAY);
    print_metric!("aux_low_duration", AUX_LOW_DURATION);
    print_metric!("post_uart_to_done", POST_UART_TO_DONE);
    print_metric!("tx_total", TX_TOTAL);
    print_metric!("write_start_interval", WRITE_START_INTERVAL);
    print_metric!("aux_done_interval", AUX_DONE_INTERVAL);
    print_metric!("idle_gap", IDLE_GAP);
    println!(
        "  aux_low_not_observed={} periodic_requests={} top_trigger_requests={} idle_gap_invalid={}",
        report.aux_low_not_observed,
        report.periodic_requests,
        report.top_trigger_requests,
        report.idle_gap_invalid
    );
}

const fn get_target_can_message(command: GroundCommand) -> Option<CanTxMessage> {
    match command {
        GroundCommand::StopSequence => Some(CanTxMessage::StopSequence { command: b'q' }),
        GroundCommand::StartSequence => Some(CanTxMessage::StartSequence { command: b's' }),
        GroundCommand::StartLogging => Some(CanTxMessage::StartLogging { command: b'l' }),
        GroundCommand::StopLogging => Some(CanTxMessage::StopLogging { command: b'm' }),
        GroundCommand::EmergencyStopPara => Some(CanTxMessage::EmergencyStopPara { command: b'z' }),
        GroundCommand::StopFinControl => Some(CanTxMessage::StopFinControl { command: b'E' }),
        GroundCommand::ClosePara => Some(CanTxMessage::ClosePara { command: b'c' }),
        GroundCommand::OpenPara => Some(CanTxMessage::OpenPara { command: b'o' }),
        GroundCommand::GnssOn | GroundCommand::GnssOff => None,
    }
}

async fn write_all(tx: &mut UartTx<'static, Async>, mut bytes: &[u8]) -> Result<bool, TxError> {
    while !bytes.is_empty() {
        let written = tx.write_async(bytes).await?;
        if written == 0 {
            println!("LoRa UART write made no progress");
            return Ok(false);
        }
        bytes = &bytes[written..];
    }
    Ok(true)
}

async fn wait_for_aux_high(aux_pin: &mut Input<'static>) -> bool {
    if aux_pin.is_high() {
        return true;
    }

    if with_timeout(
        Duration::from_millis(LORA_AUX_TIMEOUT_MS),
        aux_pin.wait_for_high(),
    )
    .await
    .is_ok()
        && aux_pin.is_high()
    {
        return true;
    }

    LORA_AUX_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
    println!("LoRa AUX timeout");
    false
}

async fn wait_for_rx_guard() {
    let Some(mut last_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
        return;
    };
    let guard_duration = Duration::from_millis(LORA_RX_TX_GUARD_MS);

    loop {
        let deadline = last_rx + guard_duration;
        if deadline <= Instant::now() {
            let Some(updated_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
                return;
            };
            last_rx = updated_rx;
            continue;
        }

        match select(LORA_RX_ACTIVITY_SIGNAL.wait(), Timer::at(deadline)).await {
            Either::First(updated_rx) => last_rx = updated_rx,
            Either::Second(()) => {
                let Some(updated_rx) = LORA_RX_ACTIVITY_SIGNAL.try_take() else {
                    return;
                };
                last_rx = updated_rx;
            }
        }
    }
}

async fn transmit_latest_payload(
    tx: &mut UartTx<'static, Async>,
    aux_pin: &mut Input<'static>,
    #[cfg(feature = "lora-timing-debug")] timing: &mut TxTimingTrace,
) {
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t3 = Some(now_us());
    }
    let payload_bytes = {
        let mut payload = PAYLOAD_MUTEX.lock().await;
        payload.check_sum = payload.calculate_checksum();
        payload.to_bytes()
    };
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t4 = Some(now_us());
    }

    if !wait_for_aux_high(aux_pin).await {
        return;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t1 = Some(now_us());
    }
    wait_for_rx_guard().await;
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t2 = Some(now_us());
    }
    if !wait_for_aux_high(aux_pin).await {
        return;
    }

    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t5 = Some(now_us());
    }
    match write_all(tx, &payload_bytes).await {
        Ok(true) => {}
        Ok(false) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            return;
        }
        Err(error) => {
            LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("LoRa UART write error: {:?}", error);
            return;
        }
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t6 = Some(now_us());
    }
    if let Err(error) = tx.flush_async().await {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("LoRa UART flush error: {:?}", error);
        return;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t7 = Some(now_us());
        if aux_pin.is_low() {
            timing.t8 = Some(now_us());
        } else if with_timeout(
            Duration::from_millis(AUX_LOW_OBSERVE_TIMEOUT_MS),
            aux_pin.wait_for_low(),
        )
        .await
        .is_ok()
        {
            timing.t8 = Some(now_us());
        } else {
            timing.aux_low_not_observed = true;
        }
    }

    if !wait_for_aux_high(aux_pin).await {
        return;
    }
    #[cfg(feature = "lora-timing-debug")]
    {
        timing.t9 = Some(now_us());
    }
}

#[embassy_executor::task]
pub async fn command_process_task() {
    let mut next_generation = 0u32;

    loop {
        let Some(command) = GroundCommand::decode_legacy(RECEIVED_DATA_CHANNEL.receive().await)
        else {
            continue;
        };

        let effects = command.acceptance_effects();
        if let Some(logging_requested) = effects.logging_requested {
            LOGGING_REQUESTED.store(logging_requested, Ordering::Relaxed);
            if !logging_requested {
                SD_FLUSH_SIGNAL.signal(());
            }
        }
        if effects.gnss_turn_on {
            GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await;
        }

        match command {
            GroundCommand::GnssOn => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await,
            GroundCommand::GnssOff => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await,
            _ => {}
        }

        if let Some(message) = get_target_can_message(command) {
            next_generation = next_generation.wrapping_add(1);
            let generation = next_generation;
            match command.category() {
                CommandCategory::Sequence => {
                    LATEST_SEQUENCE_GENERATION.store(generation, Ordering::Relaxed)
                }
                CommandCategory::Logging => {
                    LATEST_LOGGING_GENERATION.store(generation, Ordering::Relaxed)
                }
                CommandCategory::ParachutePosition => {
                    LATEST_PARA_POSITION_GENERATION.store(generation, Ordering::Relaxed)
                }
                CommandCategory::Other => {}
            }

            let request = if command.is_confirmed_by_controller_status() {
                if HAS_VALID_CONTROLLER_STATUS.load(Ordering::Relaxed)
                    && command_already_satisfied(
                        command,
                        crate::can::protocol::ControllerStatus::from_raw(
                            CONTROLLER_STATUS_RAW.load(Ordering::Relaxed),
                        )
                        .sequence_active(),
                    )
                {
                    *COMMAND_REQUEST_STATE.lock().await =
                        CommandRequestState::already_satisfied(generation, command);
                    println!("CAN command already satisfied: {:?}", command);
                    continue;
                }
                let superseded = {
                    let mut state = COMMAND_REQUEST_STATE.lock().await;
                    if let Some(existing) = state.in_flight_command()
                        && !may_supersede(existing, command)
                    {
                        println!("normal command rejected while safety request is pending");
                        continue;
                    }
                    let superseded = if state.is_in_flight() {
                        *state = state.supersede();
                        state.failure()
                    } else {
                        None
                    };
                    *state = crate::can::command::CommandRequestState::queue(generation, command);
                    superseded
                };
                if let Some(superseded) = superseded {
                    *LAST_COMMAND_FAILURE.lock().await = Some(superseded);
                    COMMAND_REQUEST_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                    println!("superseding pending CAN command");
                }
                CanTxRequest::tracked(message, command, generation)
            } else {
                CanTxRequest::untracked(message, command, generation)
            };
            if command.priority() == CommandPriority::SafetyCritical {
                CAN_SAFETY_TX_SIGNAL.signal(request);
            } else {
                CAN_TX_CHANNEL.send(request).await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lora_rx_task(mut rx: UartRx<'static, Async>) {
    let mut rx_buf = [0u8; 32];
    let mut uplink_frame = UplinkFrameBuffer::new();

    loop {
        match rx.read_async(&mut rx_buf).await {
            Ok(len) => {
                for &byte in &rx_buf[..len] {
                    if let Some(command) = uplink_frame.push(byte) {
                        LORA_RX_ACTIVITY_SIGNAL.signal(Instant::now());
                        if GroundCommand::decode_legacy(command).is_some()
                            && RECEIVED_DATA_CHANNEL.try_send(command).is_err()
                        {
                            LORA_COMMAND_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
            Err(error) => {
                LORA_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                uplink_frame.reset();
                match error {
                    RxError::FifoOverflowed => println!("LoRa UART RX FIFO overflowed"),
                    _ => println!("LoRa UART receive error: {:?}", error),
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn lora_tx_task(mut tx: UartTx<'static, Async>, mut aux_pin: Input<'static>) {
    let interval = Duration::from_millis(LORA_TRANSMIT_INTERVAL_MS);
    let mut next_tx_at = Instant::now() + interval;
    #[cfg(feature = "lora-timing-debug")]
    let mut timing_collector = TimingCollector::new();

    loop {
        Timer::at(next_tx_at).await;
        #[cfg(feature = "lora-timing-debug")]
        let mut timing = TxTimingTrace::new(now_us(), RequestSource::Periodic);

        transmit_latest_payload(
            &mut tx,
            &mut aux_pin,
            #[cfg(feature = "lora-timing-debug")]
            &mut timing,
        )
        .await;

        #[cfg(feature = "lora-timing-debug")]
        if let Some(report) = timing_collector.record(timing) {
            print_timing_report(report);
        }

        let scheduled_next = next_tx_at + interval;
        let now = Instant::now();
        next_tx_at = if scheduled_next <= now {
            now + interval
        } else {
            scheduled_next
        };
    }
}
