use core::sync::atomic::Ordering;

use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Ticker, with_timeout};
use esp_hal::{
    Async,
    gpio::Input,
    uart::{TxError, Uart},
};
use esp_println::println;

use crate::{
    can::{command::GroundCommand, protocol::ComboardCanMessage},
    constants::{LORA_AUX_TIMEOUT_MS, LORA_TRANSMIT_INTERVAL_MS},
    state::{
        CAN_TX_CHANNEL, COMMAND_REQUEST_FAILURE_COUNT, COMMAND_REQUEST_STATE, CanTxRequest,
        GNSS_CMD_CHANNEL, GnssCommand, LAST_COMMAND_FAILURE, PAYLOAD_MUTEX, RECEIVED_DATA_CHANNEL,
        TRIGGER_SIGNAL,
    },
};

const fn get_target_can_message(cmd: u8) -> Option<(ComboardCanMessage, Option<GroundCommand>)> {
    match cmd {
        b'q' => Some((
            ComboardCanMessage::StopSequence { command: cmd },
            Some(GroundCommand::StopSequence),
        )),
        b's' => Some((
            ComboardCanMessage::StartSequence { command: cmd },
            Some(GroundCommand::StartSequence),
        )),
        b'l' => Some((ComboardCanMessage::StartLogging { command: cmd }, None)),
        b'm' => Some((ComboardCanMessage::StopLogging { command: cmd }, None)),
        b'z' => Some((
            ComboardCanMessage::EmergencyStopPara { command: cmd },
            Some(GroundCommand::EmergencyStop),
        )),
        b'E' => Some((ComboardCanMessage::StopFinControl { command: cmd }, None)),
        b'c' => Some((ComboardCanMessage::ClosePara { command: cmd }, None)),
        b'o' => Some((ComboardCanMessage::OpenPara { command: cmd }, None)),
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
    let payload_bytes = {
        let mut payload = PAYLOAD_MUTEX.lock().await;
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
    let mut next_tracking_token = 0u32;

    loop {
        let command = RECEIVED_DATA_CHANNEL.receive().await;

        match command {
            b'g' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOn).await,
            b'h' => GNSS_CMD_CHANNEL.send(GnssCommand::TurnOff).await,
            _ => {}
        }

        if let Some((message, tracked_command)) = get_target_can_message(command) {
            let request = match tracked_command {
                Some(tracked_command) => {
                    next_tracking_token = next_tracking_token.wrapping_add(1);
                    let superseded = {
                        let mut state = COMMAND_REQUEST_STATE.lock().await;
                        let superseded = if state.is_in_flight() {
                            *state = state.supersede();
                            state.failure()
                        } else {
                            None
                        };
                        *state = crate::can::command::CommandRequestState::queue(
                            next_tracking_token,
                            tracked_command,
                        );
                        superseded
                    };
                    if let Some(superseded) = superseded {
                        *LAST_COMMAND_FAILURE.lock().await = Some(superseded);
                        COMMAND_REQUEST_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
                        println!("superseding pending CAN command");
                    }
                    CanTxRequest::tracked(message, next_tracking_token)
                }
                None => CanTxRequest::untracked(message),
            };
            CAN_TX_CHANNEL.send(request).await;
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
