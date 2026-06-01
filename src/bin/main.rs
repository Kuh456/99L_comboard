#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::sync::atomic::Ordering;

use c99l_comboard::{
    state::IS_CAN_ERROR,
    tasks::{command_process_task, gnss_manager_task, lora_task, parse_gnss_task},
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
use esp_rtos::embassy::Executor;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "the second-core stack is intentionally initialized during startup"
)]
#[esp_rtos::main]
async fn main(spawner0: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init(Stack::new());
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let can_tx = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO16, InputConfig::default());
    let mut led1 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let _led2 = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let lora_tx = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let mut m0 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let mut m1 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    let gnss_tx = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let gnss_en = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let aux_pin = Input::new(peripherals.GPIO8, InputConfig::default());
    let lora_rx = Input::new(peripherals.GPIO12, InputConfig::default());
    let gnss_rx = Input::new(peripherals.GPIO21, InputConfig::default());

    m0.set_low();
    m1.set_low();

    let uart_config1 = UartConfig::default()
        .with_baudrate(9600)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let mut uart1 = Uart::new(peripherals.UART1, uart_config1)
        .unwrap()
        .with_rx(gnss_rx)
        .with_tx(gnss_tx)
        .into_async();

    let uart_config1_fast = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    if let Err(e) = uart1.apply_config(&uart_config1_fast) {
        esp_println::println!("UART config error (115200baud rate): {:?}", e);
    }

    spawner0.spawn(gnss_manager_task(uart1, gnss_en).unwrap());
    spawner0.spawn(command_process_task().unwrap());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());

            executor.run(|spawner| {
                let mut can_config = twai::TwaiConfiguration::new(
                    peripherals.TWAI0,
                    can_rx,
                    can_tx,
                    BaudRate::B125K,
                    TwaiMode::Normal,
                )
                .into_async();
                can_config.set_filter(const {
                    SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"])
                });
                let can = can_config.start();
                let (_rx, _tx) = can.split();

                let uart_config2 = UartConfig::default()
                    .with_baudrate(9600)
                    .with_data_bits(DataBits::_8)
                    .with_parity(Parity::None)
                    .with_stop_bits(StopBits::_1);
                let uart2 = Uart::new(peripherals.UART2, uart_config2)
                    .unwrap()
                    .with_rx(lora_rx)
                    .with_tx(lora_tx)
                    .into_async();

                // CANタスクを有効化する場合は _rx/_tx を rx/tx に戻して spawn する。
                // spawner
                //     .spawn(can_receive_task(rx))
                //     .expect("can_receive_task should spawn during setup");
                // spawner
                //     .spawn(can_transmit_task(tx))
                //     .expect("can_transmit_task should spawn during setup");
                spawner.spawn(parse_gnss_task().unwrap());
                spawner.spawn(lora_task(uart2, aux_pin).unwrap());
            });
        },
    );

    loop {
        if IS_CAN_ERROR.load(Ordering::Relaxed) {
            led1.toggle();
        } else {
            led1.set_low();
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
