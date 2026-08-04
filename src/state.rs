use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::Instant;

use crate::{
    can::{
        command::{CommandFailureRecord, CommandRequestState, GroundCommand},
        protocol::{CanTxMessage, ControllerLinkState, ControllerStatus},
    },
    payload::Payload,
};

#[derive(Clone, Copy, Debug)]
pub struct ControllerStatusState {
    pub status: Option<ControllerStatus>,
    pub last_seen: Option<Instant>,
    pub link: ControllerLinkState,
}

impl ControllerStatusState {
    pub const fn new() -> Self {
        Self {
            status: None,
            last_seen: None,
            link: ControllerLinkState::Unknown,
        }
    }
}

impl Default for ControllerStatusState {
    fn default() -> Self {
        Self::new()
    }
}

pub type GnssPacket = [u8; 90];

#[derive(Debug, Clone, Copy)]
pub enum GnssCommand {
    TurnOn,
    TurnOff,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanTxRequest {
    pub message: CanTxMessage,
    pub tracking_token: Option<u32>,
    pub command: GroundCommand,
    pub generation: u32,
}

impl CanTxRequest {
    pub const fn untracked(message: CanTxMessage, command: GroundCommand, generation: u32) -> Self {
        Self {
            message,
            tracking_token: None,
            command,
            generation,
        }
    }

    pub const fn tracked(message: CanTxMessage, command: GroundCommand, token: u32) -> Self {
        Self {
            message,
            tracking_token: Some(token),
            command,
            generation: token,
        }
    }
}

pub static CONTROLLER_STATUS_STATE: Mutex<CriticalSectionRawMutex, ControllerStatusState> =
    Mutex::new(ControllerStatusState::new());
pub static CONTROLLER_STATUS_RAW: AtomicU8 = AtomicU8::new(0);
pub static HAS_VALID_CONTROLLER_STATUS: AtomicBool = AtomicBool::new(false);
pub static CONTROLLER_STATUS_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LEGACY_LIFTOFF_TOP_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static FIN_ANGLE_DROPPED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static COMMAND_REQUEST_STATE: Mutex<CriticalSectionRawMutex, CommandRequestState> =
    Mutex::new(CommandRequestState::Idle);
pub static LAST_COMMAND_FAILURE: Mutex<CriticalSectionRawMutex, Option<CommandFailureRecord>> =
    Mutex::new(None);
pub static COMMAND_REQUEST_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_CHANNEL: Channel<CriticalSectionRawMutex, GnssPacket, 5> = Channel::new();
pub static RECEIVED_DATA_CHANNEL: Channel<CriticalSectionRawMutex, u8, 10> = Channel::new();
pub static PAYLOAD_MUTEX: Mutex<CriticalSectionRawMutex, Payload> = Mutex::new(Payload::new());
pub static LOGGING_REQUESTED: AtomicBool = AtomicBool::new(false);
pub static LOGGING_ACTIVE: AtomicBool = AtomicBool::new(false);
pub static SD_HAS_ERROR: AtomicBool = AtomicBool::new(false);
pub static SD_WRITE_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static SD_DROPPED_ROW_COUNT: AtomicU32 = AtomicU32::new(0);
pub static HAS_UNFLUSHED_DATA: AtomicBool = AtomicBool::new(false);
pub static SD_FLUSH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, CanTxRequest, 8> = Channel::new();
pub static CAN_SAFETY_TX_SIGNAL: Signal<CriticalSectionRawMutex, CanTxRequest> = Signal::new();
pub static LATEST_SEQUENCE_GENERATION: AtomicU32 = AtomicU32::new(0);
pub static LATEST_LOGGING_GENERATION: AtomicU32 = AtomicU32::new(0);
pub static LATEST_PARA_POSITION_GENERATION: AtomicU32 = AtomicU32::new(0);
pub static IS_CAN_ERROR: AtomicBool = AtomicBool::new(true);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_COMMAND_DROP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LORA_AUX_TIMEOUT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_SETTING_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_CHANNEL_DROP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_CMD_CHANNEL: Channel<CriticalSectionRawMutex, GnssCommand, 2> = Channel::new();
