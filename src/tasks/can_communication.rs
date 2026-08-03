use core::sync::atomic::Ordering;

use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Ticker};
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
    constants::{
        CAN_CONSECUTIVE_ERROR_THRESHOLD, CAN_HEALTH_MONITOR_INTERVAL_MS, CAN_TX_TIMEOUT_MS,
    },
    state::{
        CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_TEC, CAN_TX_CHANNEL, CAN_TX_ERROR_COUNT,
        IS_CAN_ERROR, LAST_SEEN_LOG, PAYLOAD_MUTEX, TRIGGER_SIGNAL,
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeState {
    AwaitingTraffic,
    Normal,
    BusRecovering,
    TxStateUnknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanRuntimeEvent {
    TransmitSucceeded,
    ReceiveSucceeded,
    BusOff,
    TimedOutUnknownState,
}

const fn transition_runtime(
    state: CanRuntimeState,
    event: CanRuntimeEvent,
) -> (CanRuntimeState, bool) {
    match (state, event) {
        (
            CanRuntimeState::AwaitingTraffic,
            CanRuntimeEvent::TransmitSucceeded | CanRuntimeEvent::ReceiveSucceeded,
        ) => (CanRuntimeState::Normal, false),
        (CanRuntimeState::AwaitingTraffic, CanRuntimeEvent::BusOff) => {
            (CanRuntimeState::BusRecovering, true)
        }
        (CanRuntimeState::AwaitingTraffic, CanRuntimeEvent::TimedOutUnknownState) => {
            (CanRuntimeState::TxStateUnknown, true)
        }
        (CanRuntimeState::Normal, CanRuntimeEvent::BusOff) => {
            (CanRuntimeState::BusRecovering, true)
        }
        (CanRuntimeState::Normal, CanRuntimeEvent::TimedOutUnknownState) => {
            (CanRuntimeState::TxStateUnknown, true)
        }
        (CanRuntimeState::TxStateUnknown, CanRuntimeEvent::BusOff) => {
            (CanRuntimeState::BusRecovering, false)
        }
        (CanRuntimeState::BusRecovering, CanRuntimeEvent::TimedOutUnknownState) => {
            (CanRuntimeState::TxStateUnknown, false)
        }
        (
            CanRuntimeState::BusRecovering | CanRuntimeState::TxStateUnknown,
            CanRuntimeEvent::TransmitSucceeded,
        ) => (CanRuntimeState::Normal, false),
        (CanRuntimeState::BusRecovering, CanRuntimeEvent::ReceiveSucceeded) => {
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

fn publish_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    let health = classify_can_health(tec, rec, can.is_bus_off());

    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);
    CAN_HEALTH.store(health as u8, Ordering::Relaxed);

    health
}

fn publish_error_state(
    state: CanRuntimeState,
    health: CanHealth,
    consecutive_tx_errors: u8,
    consecutive_rx_errors: u8,
) {
    IS_CAN_ERROR.store(
        state != CanRuntimeState::Normal
            || health != CanHealth::Active
            || consecutive_tx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD
            || consecutive_rx_errors >= CAN_CONSECUTIVE_ERROR_THRESHOLD,
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
            {
                let mut payload = PAYLOAD_MUTEX.lock().await;
                payload.status = (payload.status & 0b0111_1111) | 0b1000_0000;
            }
            TRIGGER_SIGNAL.signal(true);
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
        // 0x200 is the log-board heartbeat; phase/flags have no Payload mapping.
        ComboardCanMessage::IntegratedBoardStatus { .. } => {
            *LAST_SEEN_LOG.lock().await = Some(Instant::now());
            let mut payload = PAYLOAD_MUTEX.lock().await;
            payload.status |= 0b0000_1000;
        }
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

#[embassy_executor::task]
pub async fn can_communication_task(mut can: twai::Twai<'static, Async>) {
    let mut runtime_state = CanRuntimeState::AwaitingTraffic;
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));
    let tx_timeout = Duration::from_millis(CAN_TX_TIMEOUT_MS);
    let mut consecutive_tx_errors = 0u8;
    let mut consecutive_rx_errors = 0u8;

    loop {
        match select3(
            health_ticker.next(),
            CAN_TX_CHANNEL.receive(),
            can.receive_async(),
        )
        .await
        {
            Either3::Second(message) => {
                match transmit_message_with_timeout(&mut can, message, tx_timeout).await {
                    Ok(()) => {
                        consecutive_tx_errors = 0;
                        runtime_state =
                            transition_runtime(runtime_state, CanRuntimeEvent::TransmitSucceeded).0;
                    }
                    Err(error) => {
                        CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        consecutive_tx_errors = consecutive_tx_errors.saturating_add(1);
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
                                println!("TWAI restarted after transmit failure");
                            }
                        }
                    }
                }
                let health = publish_health(&can);
                publish_error_state(
                    runtime_state,
                    health,
                    consecutive_tx_errors,
                    consecutive_rx_errors,
                );
            }
            Either3::Third(result) => match result {
                Ok(frame) => {
                    consecutive_rx_errors = 0;
                    runtime_state =
                        transition_runtime(runtime_state, CanRuntimeEvent::ReceiveSucceeded).0;
                    handle_received_frame(frame).await;
                    let health = publish_health(&can);
                    publish_error_state(
                        runtime_state,
                        health,
                        consecutive_tx_errors,
                        consecutive_rx_errors,
                    );
                }
                Err(error) => {
                    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                    consecutive_rx_errors = consecutive_rx_errors.saturating_add(1);
                    println!("CAN receive error: {:?}", error);

                    if error == EspTwaiError::BusOff {
                        let recovery =
                            enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                        can = recovery.0;
                        runtime_state = recovery.1;
                        if recovery.2 {
                            println!("TWAI restarted after receive failure");
                        }
                    }
                    let health = publish_health(&can);
                    publish_error_state(
                        runtime_state,
                        health,
                        consecutive_tx_errors,
                        consecutive_rx_errors,
                    );
                }
            },
            Either3::First(_) => {
                let health = publish_health(&can);
                if health == CanHealth::BusOff {
                    let recovery = enter_recovering(can, runtime_state, CanRuntimeEvent::BusOff);
                    can = recovery.0;
                    runtime_state = recovery.1;
                    if recovery.2 {
                        println!("TWAI restarted after health monitor detected Bus Off");
                    }
                }
                publish_error_state(
                    runtime_state,
                    health,
                    consecutive_tx_errors,
                    consecutive_rx_errors,
                );
            }
        }
    }
}
