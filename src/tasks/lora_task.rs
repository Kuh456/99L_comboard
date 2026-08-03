use core::sync::atomic::Ordering;

use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use esp_hal::{
    Async,
    gpio::Input,
    uart::{TxError, Uart},
};
use esp_println::println;

use crate::{
    can::protocol::ComboardCanMessage,
    constants::{LORA_AUX_TIMEOUT_MS, LORA_TRANSMIT_INTERVAL_MS},
    state::{
        CAN_TX_CHANNEL, GNSS_CMD_CHANNEL, GnssCommand, IS_LOGGING, LAST_SEEN_LOG, PAYLOAD_MUTEX,
        RECEIVED_DATA_CHANNEL, SD_FLUSH_SIGNAL, TRIGGER_SIGNAL,
    },
};

const fn get_target_can_message(cmd: u8) -> Option<ComboardCanMessage> {
    match cmd {
        b'q' => Some(ComboardCanMessage::StopSequence { command: cmd }),
        b's' => Some(ComboardCanMessage::StartSequence { command: cmd }),
        b'l' => Some(ComboardCanMessage::StartLogging { command: cmd }),
        b'm' => Some(ComboardCanMessage::StopLogging { command: cmd }),
        b'z' => Some(ComboardCanMessage::EmergencyStopPara { command: cmd }),
        b'E' => Some(ComboardCanMessage::StopFinControl { command: cmd }),
        b'c' => Some(ComboardCanMessage::ClosePara { command: cmd }),
        b'o' => Some(ComboardCanMessage::OpenPara { command: cmd }),
        _ => None,
    }
}

const fn is_supported_command(command: u8) -> bool {
    matches!(
        command,
        b's' | b'q' | b'l' | b'm' | b'g' | b'h' | b'z' | b'E' | b'c' | b'o'
    )
}

async fn write_all(uart: &mut Uart<'static, Async>, mut bytes: &[u8]) -> Result<bool, TxError> {
    while !bytes.is_empty() {
        let written = uart.write_async(bytes).await?;
        if written == 0 {
            println!("LoRa UART write made no progress");
            return Ok(false);
        }
        bytes = &bytes[written..];
    }
    Ok(true)
}

async fn transmit_latest_payload(uart: &mut Uart<'static, Async>, aux_pin: &mut Input<'static>) {
    let log_is_fresh = match *LAST_SEEN_LOG.lock().await {
        Some(last_seen) => Instant::now().duration_since(last_seen).as_secs() < 80,
        None => false,
    };

    let payload_bytes = {
        let mut payload = PAYLOAD_MUTEX.lock().await;
        if log_is_fresh {
            payload.status |= 0b0000_1000;
        } else {
            payload.status &= 0b1111_0111;
        }
        payload.check_sum = payload.calculate_checksum();
        payload.to_bytes()
    };

    match write_all(uart, &payload_bytes).await {
        Ok(true) => {}
        Ok(false) => return,
        Err(error) => {
            println!("LoRa UART write error: {:?}", error);
            return;
        }
    }
    if let Err(error) = uart.flush_async().await {
        println!("LoRa UART flush error: {:?}", error);
        return;
    }

    if aux_pin.is_low()
        && with_timeout(
            Duration::from_millis(LORA_AUX_TIMEOUT_MS),
            aux_pin.wait_for_high(),
        )
        .await
        .is_err()
    {
        println!("LoRa AUX timeout");
    }
}

#[embassy_executor::task]
pub async fn command_process_task() {
    loop {
        let command = RECEIVED_DATA_CHANNEL.receive().await;

        match command {
            b's' => {
                GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await;
                IS_LOGGING.store(true, Ordering::Relaxed);
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.status = (payload.status & 0b1101_1111) | 0b0010_0000;
            }
            b'q' => {
                IS_LOGGING.store(false, Ordering::Relaxed);
                SD_FLUSH_SIGNAL.signal(());
                GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await;
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.status &= 0b1101_1111;
            }
            b'l' => IS_LOGGING.store(true, Ordering::Relaxed),
            b'm' => {
                IS_LOGGING.store(false, Ordering::Relaxed);
                SD_FLUSH_SIGNAL.signal(());
            }
            b'g' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await,
            b'h' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await,
            _ => {}
        }

        if let Some(message) = get_target_can_message(command) {
            CAN_TX_CHANNEL.send(message).await;
        }
    }
}

#[embassy_executor::task]
pub async fn lora_task(mut uart: Uart<'static, Async>, mut aux_pin: Input<'static>) {
    let mut rx_buf = [0u8; 64];
    let mut tx_ticker = Ticker::every(Duration::from_millis(LORA_TRANSMIT_INTERVAL_MS));

    loop {
        match select3(
            TRIGGER_SIGNAL.wait(),
            uart.read_async(&mut rx_buf),
            tx_ticker.next(),
        )
        .await
        {
            Either3::First(_trigger) => {
                transmit_latest_payload(&mut uart, &mut aux_pin).await;
            }
            Either3::Second(Ok(len)) => {
                if len > 0 {
                    println!("cmd: {:?}", &rx_buf[..len]);
                    for &command in &rx_buf[..len] {
                        if is_supported_command(command) {
                            RECEIVED_DATA_CHANNEL.send(command).await;
                        }
                    }
                }
            }
            Either3::Second(Err(error)) => {
                println!("LoRa UART receive error: {:?}", error);
            }
            Either3::Third(_) => {
                transmit_latest_payload(&mut uart, &mut aux_pin).await;
            }
        }
    }
}
