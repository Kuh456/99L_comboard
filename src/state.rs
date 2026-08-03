use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::Instant;

use crate::{
    can::{
        command::{CommandFailureRecord, CommandRequestState},
        protocol::ComboardCanMessage,
    },
    payload::Payload,
};

pub type GnssPacket = [u8; 90];

#[derive(Debug, Clone, Copy)]
pub enum GnssCommand {
    TurnOn,
    TurnOff,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanTxRequest {
    pub message: ComboardCanMessage,
    pub tracking_token: Option<u32>,
}

impl CanTxRequest {
    pub const fn untracked(message: ComboardCanMessage) -> Self {
        Self {
            message,
            tracking_token: None,
        }
    }

    pub const fn tracked(message: ComboardCanMessage, token: u32) -> Self {
        Self {
            message,
            tracking_token: Some(token),
        }
    }
}

pub static LAST_SEEN_CONTROLLER: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);
pub static CONTROLLER_STATUS_RAW: AtomicU8 = AtomicU8::new(0);
pub static HAS_VALID_CONTROLLER_STATUS: AtomicBool = AtomicBool::new(false);
pub static CONTROLLER_STATUS_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static COMMAND_REQUEST_STATE: Mutex<CriticalSectionRawMutex, CommandRequestState> =
    Mutex::new(CommandRequestState::Idle);
pub static LAST_COMMAND_FAILURE: Mutex<CriticalSectionRawMutex, Option<CommandFailureRecord>> =
    Mutex::new(None);
pub static COMMAND_REQUEST_FAILURE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TRIGGER_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static GNSS_CHANNEL: Channel<CriticalSectionRawMutex, GnssPacket, 5> = Channel::new();
pub static RECEIVED_DATA_CHANNEL: Channel<CriticalSectionRawMutex, u8, 10> = Channel::new();
pub static PAYLOAD_MUTEX: Mutex<CriticalSectionRawMutex, Payload> = Mutex::new(Payload::new());
pub static IS_LOGGING: AtomicBool = AtomicBool::new(false);
pub static HAS_UNFLUSHED_DATA: AtomicBool = AtomicBool::new(false);
pub static SD_FLUSH_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, CanTxRequest, 8> = Channel::new();
pub static IS_CAN_ERROR: AtomicBool = AtomicBool::new(true);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GNSS_CMD_CHANNEL: Channel<CriticalSectionRawMutex, GnssCommand, 2> = Channel::new();
