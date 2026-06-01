use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{Async, gpio::Input, uart::Uart};
use esp_println::println;

use crate::{
    constants::{
        CAN_ID_EMERGENCY_STOP_PARA, CAN_ID_ERASE_FLASH, CAN_ID_POWER_OFF_CAMERA,
        CAN_ID_POWER_ON_CAMERA, CAN_ID_STAET_LOGGING, CAN_ID_STAET_RECORDING,
        CAN_ID_START_SEQUENCE, CAN_ID_STOP_LOGGING, CAN_ID_STOP_RECORDING, CAN_ID_STOP_SEQUENCE,
        LORA_TRANSMIT_INTERVAL_MS,
    },
    state::{
        CAN_TX_CHANNEL, GNSS_CMD_CHANNEL, GnssCommand, IS_LOGGING, LAST_SEEN_CAMERA, LAST_SEEN_LOG,
        PAYLOAD_MUTEX, RECEIVED_DATA_CHANNEL, TRIGGER_SIGNAL,
    },
};

const fn get_target_can_id(cmd: u8) -> Option<u16> {
    match cmd {
        b'e' => Some(CAN_ID_STOP_SEQUENCE),
        b's' => Some(CAN_ID_START_SEQUENCE),
        // b'p' => Some(CAN_ID_ACTUATE_PARA),
        b'x' => Some(CAN_ID_ERASE_FLASH),
        b'l' => Some(CAN_ID_STAET_LOGGING),
        b'm' => Some(CAN_ID_STOP_LOGGING),
        b'c' => Some(CAN_ID_STAET_RECORDING),
        b'v' => Some(CAN_ID_STOP_RECORDING),
        b'i' => Some(CAN_ID_POWER_ON_CAMERA),
        b'o' => Some(CAN_ID_POWER_OFF_CAMERA),
        b'a' => Some(CAN_ID_EMERGENCY_STOP_PARA),
        _ => None,
    }
}

#[embassy_executor::task]
pub async fn command_process_task() {
    loop {
        let command = RECEIVED_DATA_CHANNEL.receive().await;

        match command {
            b's' => {
                IS_LOGGING.store(true, Ordering::Relaxed);
                GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await;
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.status = (payload.status & 0b1101_1111) | 0b0010_0000;
            }
            b'e' => {
                IS_LOGGING.store(false, Ordering::Relaxed);
                GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await;
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.status &= 0b1101_1111;
            }
            b'l' => IS_LOGGING.store(true, Ordering::Relaxed),
            b'm' => IS_LOGGING.store(false, Ordering::Relaxed),
            b'g' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await,
            b'h' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await,
            _ => {}
        }

        if let Some(can_id) = get_target_can_id(command) {
            CAN_TX_CHANNEL.send((can_id, command)).await;
        }
    }
}

#[embassy_executor::task]
pub async fn lora_task(mut uart: Uart<'static, Async>, mut aux_pin: Input<'static>) {
    let mut rx_buf = [0u8; 64];
    let mut last_tx = Instant::now();

    loop {
        let receive_fut = select(TRIGGER_SIGNAL.wait(), uart.read_async(&mut rx_buf));

        match embassy_time::with_timeout(Duration::from_secs(3), receive_fut).await {
            Ok(Either::First(_trigger)) => {}
            Ok(Either::Second(Ok(len))) => {
                if len > 0 {
                    println!("cmd: {:?}", &rx_buf[..len]);
                    RECEIVED_DATA_CHANNEL.send(rx_buf[0]).await;
                }
            }
            Ok(Either::Second(Err(_))) => {}
            Err(_) => {}
        }

        if last_tx.elapsed().as_millis() >= LORA_TRANSMIT_INTERVAL_MS {
            let payload = {
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.check_sum = payload.calculate_checksum();
                *payload
            };
            let payload_bytes = payload.to_bytes();

            let _ = uart.write_async(&payload_bytes).await;
            let _ = uart.flush();

            aux_pin.wait_for_high().await;
            last_tx = Instant::now();
        }
    }
}

#[embassy_executor::task]
pub async fn create_lora_payload() {
    loop {
        {
            let mut payload = PAYLOAD_MUTEX.lock().await;
            let now = Instant::now();
            let is_timeout = |last_seen: Option<Instant>| -> bool {
                match last_seen {
                    Some(time) => now.duration_since(time).as_secs() >= 80,
                    None => true,
                }
            };

            if is_timeout(*LAST_SEEN_LOG.lock().await) {
                payload.status &= 0b1111_0111;
            }
            if is_timeout(*LAST_SEEN_CAMERA.lock().await) {
                payload.status &= 0b1110_1111;
            }
            // if is_timeout(*LAST_SEEN_POWER.lock().await) {
            //     payload.status &= 0b1101_1111;
            // }

            payload.check_sum = payload.calculate_checksum();
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
