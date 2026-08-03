pub const BUF_SIZE: usize = 2048;
pub const LORA_TRANSMIT_INTERVAL_MS: u64 = 500;
pub const SD_LOG_INTERVAL_MS: u64 = 100;
pub const SD_FLUSH_INTERVAL_SECS: u64 = 1;
pub const CAN_TX_TIMEOUT_MS: u64 = 100;
pub const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 100;
pub const CAN_CONSECUTIVE_ERROR_THRESHOLD: u8 = 3;
// Provisional confirmation-observation window. Reconfigure this after the
// actual 0x200 transmit period has been measured on the integrated hardware.
pub const COMMAND_CONFIRM_TIMEOUT_MS: u64 = 500;
pub const LORA_AUX_TIMEOUT_MS: u64 = 1_000;
