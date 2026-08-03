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
        command::CommandRequestState,
        health::{CanHealth, classify_can_health},
        protocol::{
            CanRxMessage, ControllerLinkState, ControllerStatus, ControllerStatusFlags,
            controller_status_effects,
        },
        tx::{CanTxError, transmit_message_with_timeout},
    },
    constants::{
        CAN_CONSECUTIVE_ERROR_THRESHOLD, CAN_HEALTH_MONITOR_INTERVAL_MS, CAN_TX_TIMEOUT_MS,
        COMMAND_CONFIRM_TIMEOUT_MS,
    },
    state::{
        CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_TEC, CAN_TX_CHANNEL, CAN_TX_ERROR_COUNT,
        COMMAND_REQUEST_FAILURE_COUNT, COMMAND_REQUEST_STATE, CONTROLLER_STATUS_RAW,
        CONTROLLER_STATUS_RX_COUNT, CONTROLLER_STATUS_STATE, FIN_ANGLE_DROPPED_COUNT,
        GNSS_CMD_CHANNEL, GnssCommand, HAS_VALID_CONTROLLER_STATUS, IS_CAN_ERROR,
        LAST_COMMAND_FAILURE, LEGACY_LIFTOFF_TOP_RX_COUNT, LOGGING_REQUESTED, PAYLOAD_MUTEX,
        SD_FLUSH_SIGNAL, TRIGGER_SIGNAL,
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

async fn apply_received_message(message: CanRxMessage) {
    match message {
        CanRxMessage::LiftOff { .. } | CanRxMessage::Top { .. } => {
            LEGACY_LIFTOFF_TOP_RX_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        CanRxMessage::AngleSpeed { xyz } => {
            PAYLOAD_MUTEX.lock().await.angle_speed = xyz;
        }
        CanRxMessage::Acceleration { xyz } => {
            PAYLOAD_MUTEX.lock().await.acceleration = xyz;
        }
        CanRxMessage::AirPressure { bytes } => {
            PAYLOAD_MUTEX.lock().await.air_pressure = bytes;
        }
        CanRxMessage::AccumulatedAngle { xyz } => {
            PAYLOAD_MUTEX.lock().await.integrated_angle = xyz;
        }
        // The protocol contains three i16 fin values, while Payload has one i8
        // field. Do not guess which value or narrowing rule should be used.
        CanRxMessage::FinAngle { .. } => {
            FIN_ANGLE_DROPPED_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        CanRxMessage::ControllerStatus { status } => {
            apply_controller_status(status).await;
        } // Command frames are transmitted by this board and have no RX side effect.
    }
}

fn previous_controller_status() -> Option<ControllerStatusFlags> {
    if !HAS_VALID_CONTROLLER_STATUS.load(Ordering::Relaxed) {
        return None;
    }

    match ControllerStatus::from_raw(CONTROLLER_STATUS_RAW.load(Ordering::Relaxed)) {
        ControllerStatus::Valid(status) => Some(status),
        ControllerStatus::Unknown(_) => None,
    }
}

async fn apply_controller_status(status: ControllerStatus) {
    {
        let mut controller = CONTROLLER_STATUS_STATE.lock().await;
        controller.status = Some(status);
        controller.last_seen = Some(Instant::now());
        controller.link = ControllerLinkState::Online;
    }
    CONTROLLER_STATUS_RX_COUNT.fetch_add(1, Ordering::Relaxed);

    let ControllerStatus::Valid(status) = status else {
        HAS_VALID_CONTROLLER_STATUS.store(false, Ordering::Relaxed);
        println!("unknown controller status: 0x{:02x}", status.raw());
        return;
    };

    let effects = controller_status_effects(previous_controller_status(), status);
    CONTROLLER_STATUS_RAW.store(status.raw(), Ordering::Relaxed);
    HAS_VALID_CONTROLLER_STATUS.store(true, Ordering::Relaxed);
    PAYLOAD_MUTEX.lock().await.status = status.raw();

    if let Some(sequence_active) = effects.sequence_changed {
        LOGGING_REQUESTED.store(sequence_active, Ordering::Relaxed);
        let gnss_command = if sequence_active {
            GnssCommand::TurnOn
        } else {
            SD_FLUSH_SIGNAL.signal(());
            GnssCommand::TurnOff
        };
        publish_latest_gnss_command(gnss_command);
    }

    let mut request_state = COMMAND_REQUEST_STATE.lock().await;
    let previous_request = *request_state;
    *request_state = request_state.confirm(status.sequence_active(), status.liftoff_detected());
    if previous_request != *request_state
        && let CommandRequestState::Completed { command, .. } = *request_state
    {
        println!("CAN command confirmed by controller: {:?}", command);
    }
    drop(request_state);

    if effects.top_rising {
        TRIGGER_SIGNAL.signal(true);
    }
}

fn publish_latest_gnss_command(command: GnssCommand) {
    if GNSS_CMD_CHANNEL.try_send(command).is_ok() {
        return;
    }

    if GNSS_CMD_CHANNEL.try_receive().is_err() || GNSS_CMD_CHANNEL.try_send(command).is_err() {
        println!("failed to publish GNSS state from controller status");
    }
}

async fn mark_request_transmitted(token: u32) {
    let mut state = COMMAND_REQUEST_STATE.lock().await;
    *state = state.mark_transmitted(token, Instant::now().as_millis());
}

async fn mark_request_transmit_failed(token: u32) {
    let failure = {
        let mut state = COMMAND_REQUEST_STATE.lock().await;
        let previous = *state;
        *state = state.mark_transmit_failed(token);
        (previous != *state).then(|| state.failure()).flatten()
    };
    if let Some(failure) = failure {
        COMMAND_REQUEST_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
        *LAST_COMMAND_FAILURE.lock().await = Some(failure);
    }
}

async fn expire_pending_request() {
    let failure = {
        let mut state = COMMAND_REQUEST_STATE.lock().await;
        let previous = *state;
        *state = state.expire(Instant::now().as_millis(), COMMAND_CONFIRM_TIMEOUT_MS);
        (previous != *state).then(|| state.failure()).flatten()
    };
    if let Some(failure) = failure {
        COMMAND_REQUEST_FAILURE_COUNT.fetch_add(1, Ordering::Relaxed);
        *LAST_COMMAND_FAILURE.lock().await = Some(failure);
        println!(
            "CAN command confirmation failed: {:?} {:?}",
            failure.command, failure.reason
        );
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

    match CanRxMessage::decode_standard(id, frame.data()) {
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
            Either3::Second(request) => {
                match transmit_message_with_timeout(&mut can, request.message, tx_timeout).await {
                    Ok(()) => {
                        consecutive_tx_errors = 0;
                        if let Some(token) = request.tracking_token {
                            mark_request_transmitted(token).await;
                        }
                        runtime_state =
                            transition_runtime(runtime_state, CanRuntimeEvent::TransmitSucceeded).0;
                    }
                    Err(error) => {
                        CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                        consecutive_tx_errors = consecutive_tx_errors.saturating_add(1);
                        if let Some(token) = request.tracking_token {
                            mark_request_transmit_failed(token).await;
                        }
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
                expire_pending_request().await;
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
