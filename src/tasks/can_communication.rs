use core::sync::atomic::Ordering;

use embassy_futures::select::{Either4, select4};
use embassy_time::{Duration, Ticker};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, EspTwaiError, EspTwaiFrame},
};
use esp_println::println;

use crate::{
    can::{
        health::{CanHealth, classify_can_health},
        protocol::ComboardCanMessage,
        tx::{CanTxError, transmit_message_with_timeout},
    },
    constants::{CAN_HEALTH_MONITOR_INTERVAL_MS, CAN_PROBE_INTERVAL_MS, CAN_TX_TIMEOUT_MS},
    state::{
        CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_TEC, CAN_TX_CHANNEL, CAN_TX_ERROR_COUNT,
        IS_CAN_ERROR, PAYLOAD_MUTEX, TRIGGER_SIGNAL,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeState {
    Normal,
    Recovering,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeEvent {
    TransmitSucceeded,
    BusOff,
    TimedOutUnknownState,
    ProbeSucceeded,
    ProbeFailed,
}

const fn transition_runtime(
    state: CanRuntimeState,
    event: CanRuntimeEvent,
) -> (CanRuntimeState, bool) {
    match (state, event) {
        (
            CanRuntimeState::Normal,
            CanRuntimeEvent::BusOff | CanRuntimeEvent::TimedOutUnknownState,
        ) => (CanRuntimeState::Recovering, true),
        (CanRuntimeState::Recovering, CanRuntimeEvent::ProbeSucceeded) => {
            (CanRuntimeState::Normal, false)
        }
        (state, _) => (state, false),
    }
}

fn enter_recovering(
    can: twai::Twai<'static, Async>,
    state: CanRuntimeState,
    event: CanRuntimeEvent,
) -> (twai::Twai<'static, Async>, CanRuntimeState, bool) {
    let (next_state, restart_required) = transition_runtime(state, event);
    let can = if restart_required {
        can.stop().start()
    } else {
        can
    };

    (can, next_state, restart_required)
}

fn drain_pending_commands() -> usize {
    let mut dropped = 0;
    while CAN_TX_CHANNEL.try_receive().is_ok() {
        dropped += 1;
    }
    dropped
}

fn publish_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    let health = classify_can_health(tec, rec, can.is_bus_off());

    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);
    CAN_HEALTH.store(health as u8, Ordering::Relaxed);

    health
}

fn publish_error_state(state: CanRuntimeState, health: CanHealth) {
    IS_CAN_ERROR.store(
        state != CanRuntimeState::Normal || health != CanHealth::Active,
        Ordering::Relaxed,
    );
}

async fn apply_received_message(message: ComboardCanMessage) {
    match message {
        ComboardCanMessage::LiftOff { .. } => {
            let mut payload = PAYLOAD_MUTEX.lock().await;
            payload.status = (payload.status & 0b1011_1111) | 0b0100_0000;
        }
        ComboardCanMessage::Top { .. } => {
            TRIGGER_SIGNAL.signal(true);
            let mut payload = PAYLOAD_MUTEX.lock().await;
            payload.status = (payload.status & 0b0111_1111) | 0b1000_0000;
        }
        ComboardCanMessage::AngleSpeed { xyz } => {
            PAYLOAD_MUTEX.lock().await.angle_speed = xyz;
        }
        ComboardCanMessage::Acceleration { xyz } => {
            PAYLOAD_MUTEX.lock().await.acceleration = xyz;
        }
        ComboardCanMessage::AirPressure { bytes } => {
            PAYLOAD_MUTEX.lock().await.air_pressure = bytes;
        }
        ComboardCanMessage::AccumulatedAngle { xyz } => {
            PAYLOAD_MUTEX.lock().await.integrated_angle = xyz;
        }
        // The protocol contains three i16 fin values, while Payload has one i8
        // field. Do not guess which value or narrowing rule should be used.
        ComboardCanMessage::FinAngle { .. } => {}
        // Payload has no unambiguous phase/flags destination.
        ComboardCanMessage::IntegratedBoardStatus { .. } => {}
        // Command frames are transmitted by this board and have no RX side effect.
        ComboardCanMessage::StopFinControl { .. }
        | ComboardCanMessage::EmergencyStopPara { .. }
        | ComboardCanMessage::StartSequence { .. }
        | ComboardCanMessage::StopSequence { .. }
        | ComboardCanMessage::OpenPara { .. }
        | ComboardCanMessage::ClosePara { .. }
        | ComboardCanMessage::StartLogging { .. }
        | ComboardCanMessage::StopLogging { .. } => {}
    }
}

async fn handle_received_frame(frame: EspTwaiFrame) {
    let id = match frame.id() {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(id) => {
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("unsupported extended CAN frame: id=0x{:08x}", id.as_raw());
            return;
        }
    };

    match ComboardCanMessage::decode_standard(id, frame.data()) {
        Ok(message) => apply_received_message(message).await,
        Err(error) => {
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            println!("invalid CAN frame: {:?}", error);
        }
    }
}

fn probe_message() -> ComboardCanMessage {
    // Existing status/heartbeat ID; zero values have no actuator side effect.
    ComboardCanMessage::IntegratedBoardStatus { phase: 0, flags: 0 }
}

#[embassy_executor::task]
pub async fn can_communication_task(mut can: twai::Twai<'static, Async>) {
    let mut runtime_state = CanRuntimeState::Normal;
    let mut probe_ticker = Ticker::every(Duration::from_millis(CAN_PROBE_INTERVAL_MS));
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));
    let tx_timeout = Duration::from_millis(CAN_TX_TIMEOUT_MS);

    loop {
        match select4(
            probe_ticker.next(),
            health_ticker.next(),
            CAN_TX_CHANNEL.receive(),
            can.receive_async(),
        )
        .await
        {
            Either4::Third(message) => {
                if runtime_state == CanRuntimeState::Recovering {
                    let dropped = 1 + drain_pending_commands();
                    println!(
                        "dropping {} CAN command(s) while recovering; first: {:?}",
                        dropped, message
                    );
                    continue;
                }

                match transmit_message_with_timeout(&mut can, message, tx_timeout).await {
                    Ok(()) => {
                        runtime_state =
                            transition_runtime(runtime_state, CanRuntimeEvent::TransmitSucceeded).0;
                    }
                    Err(error) => {
                        CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        IS_CAN_ERROR.store(true, Ordering::Relaxed);
                        println!("CAN transmit error: {:?}", error);

                        if matches!(error, CanTxError::BusOff | CanTxError::TimedOutUnknownState) {
                            let event = if error == CanTxError::BusOff {
                                CanRuntimeEvent::BusOff
                            } else {
                                CanRuntimeEvent::TimedOutUnknownState
                            };
                            let recovery = enter_recovering(can, runtime_state, event);
                            can = recovery.0;
                            runtime_state = recovery.1;
                            if recovery.2 {
                                let dropped = drain_pending_commands();
                                println!("TWAI restarted; dropped {} queued commands", dropped);
                            }
                        }
                    }
                }
            }
            Either4::Fourth(result) => match result {
                Ok(frame) => handle_received_frame(frame).await,
                Err(error) => {
                    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                    IS_CAN_ERROR.store(true, Ordering::Relaxed);
                    println!("CAN receive error: {:?}", error);

                    if error == EspTwaiError::BusOff {
                        let recovery =
                            enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                        can = recovery.0;
                        runtime_state = recovery.1;
                        if recovery.2 {
                            let dropped = drain_pending_commands();
                            println!("TWAI restarted; dropped {} queued commands", dropped);
                        }
                    }
                }
            },
            Either4::First(_) => {
                if runtime_state == CanRuntimeState::Recovering {
                    match transmit_message_with_timeout(&mut can, probe_message(), tx_timeout).await
                    {
                        Ok(()) => {
                            runtime_state =
                                transition_runtime(runtime_state, CanRuntimeEvent::ProbeSucceeded)
                                    .0;
                            let health = publish_health(&can);
                            publish_error_state(runtime_state, health);
                            println!("CAN probe succeeded; normal operation resumed");
                        }
                        Err(error) => {
                            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                            IS_CAN_ERROR.store(true, Ordering::Relaxed);
                            runtime_state =
                                transition_runtime(runtime_state, CanRuntimeEvent::ProbeFailed).0;
                            println!("CAN probe failed: {:?}", error);
                        }
                    }
                }
            }
            Either4::Second(_) => {
                let health = publish_health(&can);
                if health == CanHealth::BusOff {
                    let recovery = enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                    can = recovery.0;
                    runtime_state = recovery.1;
                    if recovery.2 {
                        let dropped = drain_pending_commands();
                        println!("TWAI restarted; dropped {} queued commands", dropped);
                    }
                }
                publish_error_state(runtime_state, health);
            }
        }
    }
}
