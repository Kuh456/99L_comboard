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
    can::{
        command::{
            CommandCategory, CommandPriority, CommandRequestState, GroundCommand,
            command_already_satisfied, may_supersede,
        },
        protocol::CanTxMessage,
    },
    constants::{LORA_AUX_TIMEOUT_MS, LORA_TRANSMIT_INTERVAL_MS},
    state::{
        CAN_SAFETY_TX_SIGNAL, CAN_TX_CHANNEL, COMMAND_REQUEST_FAILURE_COUNT, COMMAND_REQUEST_STATE,
        CONTROLLER_STATUS_RAW, CanTxRequest, GNSS_CMD_CHANNEL, GnssCommand,
        HAS_VALID_CONTROLLER_STATUS, LAST_COMMAND_FAILURE, LATEST_LOGGING_GENERATION,
        LATEST_PARA_POSITION_GENERATION, LATEST_SEQUENCE_GENERATION, LOGGING_REQUESTED,
        LORA_AUX_TIMEOUT_COUNT, LORA_RX_ERROR_COUNT, LORA_TX_ERROR_COUNT, PAYLOAD_MUTEX,
        RECEIVED_DATA_CHANNEL, SD_FLUSH_SIGNAL, TRIGGER_SIGNAL,
    },
};

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
    if let Err(error) = uart.flush_async().await {
        LORA_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
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
        LORA_AUX_TIMEOUT_COUNT.fetch_add(1, Ordering::Relaxed);
        println!("LoRa AUX timeout");
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
                        if GroundCommand::decode_legacy(command).is_some() {
                            RECEIVED_DATA_CHANNEL.send(command).await;
                        }
                    }
                }
            }
            Either3::Second(Err(error)) => {
                LORA_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                println!("LoRa UART receive error: {:?}", error);
            }
            Either3::Third(_) => {
                transmit_latest_payload(&mut uart, &mut aux_pin).await;
            }
        }
    }
}
