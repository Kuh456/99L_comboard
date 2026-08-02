use core::sync::atomic::AtomicBool;

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::Instant;

use crate::payload::Payload;

pub type GnssPacket = [u8; 90];

#[derive(Debug, Clone, Copy)]
pub enum GnssCommand {
    TurnOn,
    TurnOff,
}

pub static LAST_SEEN_LOG: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);
pub static LAST_SEEN_CAMERA: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);
// pub static LAST_SEEN_POWER: Mutex<CriticalSectionRawMutex, Option<Instant>> = Mutex::new(None);

pub static TRIGGER_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static GNSS_CHANNEL: Channel<CriticalSectionRawMutex, GnssPacket, 5> = Channel::new();
pub static RECEIVED_DATA_CHANNEL: Channel<CriticalSectionRawMutex, u8, 10> = Channel::new();
pub static PAYLOAD_MUTEX: Mutex<CriticalSectionRawMutex, Payload> = Mutex::new(Payload::new());
pub static IS_LOGGING: AtomicBool = AtomicBool::new(false);
pub static HAS_UNFLUSHED_DATA: AtomicBool = AtomicBool::new(false);
pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, (u16, u8), 5> = Channel::new();
pub static IS_CAN_ERROR: AtomicBool = AtomicBool::new(true);
pub static GNSS_CMD_CHANNEL: Channel<CriticalSectionRawMutex, GnssCommand, 2> = Channel::new();
