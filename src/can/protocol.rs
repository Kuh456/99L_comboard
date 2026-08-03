pub const CAN_ID_STOP_FIN_CONTROL: u16 = 0x001;
pub const CAN_ID_EMERGENCY_STOP_PARA: u16 = 0x003;
pub const CAN_ID_START_SEQUENCE: u16 = 0x005;
pub const CAN_ID_STOP_SEQUENCE: u16 = 0x00a;

pub const CAN_ID_START_LOGGING: u16 = 0x011;
pub const CAN_ID_STOP_LOGGING: u16 = 0x01e;

// 現在の指定値。優先度については後述。
pub const CAN_ID_OPEN_PARA: u16 = 0x30d;
pub const CAN_ID_CLOSE_PARA: u16 = 0x30e;

pub const CAN_ID_AIR_PRESSURE: u16 = 0x10a;
pub const CAN_ID_ACCELERATION: u16 = 0x11a;
pub const CAN_ID_ANGLE_SPEED: u16 = 0x120;
pub const CAN_ID_FIN_ANGLE: u16 = 0x13a;
pub const CAN_ID_ACCUMULATED_ANGLE: u16 = 0x14a;

pub const CAN_ID_CONTROLLER_STATUS: u16 = 0x200;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControllerLinkState {
    Unknown,
    Online,
    TimedOut,
}

pub const fn controller_link_state(
    has_received_status: bool,
    elapsed_ms: u64,
    timeout_ms: Option<u64>,
) -> ControllerLinkState {
    if !has_received_status {
        ControllerLinkState::Unknown
    } else if let Some(timeout_ms) = timeout_ms
        && elapsed_ms >= timeout_ms
    {
        ControllerLinkState::TimedOut
    } else {
        ControllerLinkState::Online
    }
}

const CONTROLLER_STATUS_KNOWN_MASK: u8 = 0b11101111;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerStatus {
    raw: u8,
}

impl ControllerStatus {
    pub const fn from_raw(raw: u8) -> Self {
        Self { raw }
    }

    pub const fn raw(self) -> u8 {
        self.raw
    }

    pub const fn top_detected(self) -> bool {
        self.raw & (1 << 0) != 0
    }

    pub const fn main_power_on(self) -> bool {
        self.raw & (1 << 1) != 0
    }

    pub const fn emergency_power_on(self) -> bool {
        self.raw & (1 << 2) != 0
    }

    pub const fn control_active(self) -> bool {
        self.raw & (1 << 3) != 0
    }

    pub const fn sequence_active(self) -> bool {
        self.raw & (1 << 5) != 0
    }

    pub const fn liftoff_detected(self) -> bool {
        self.raw & (1 << 6) != 0
    }

    pub const fn parachute_motor_open(self) -> bool {
        self.raw & (1 << 7) != 0
    }

    pub const fn unknown_bits(self) -> u8 {
        self.raw & !CONTROLLER_STATUS_KNOWN_MASK
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerStatusEffects {
    pub sequence_changed: Option<bool>,
    pub top_rising: bool,
}

pub const fn controller_status_effects(
    previous: Option<ControllerStatus>,
    current: ControllerStatus,
) -> ControllerStatusEffects {
    ControllerStatusEffects {
        sequence_changed: match previous {
            Some(previous) if previous.sequence_active() == current.sequence_active() => None,
            _ => Some(current.sequence_active()),
        },
        top_rising: current.top_detected()
            && match previous {
                Some(previous) => !previous.top_detected(),
                None => true,
            },
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanTxMessage {
    StopFinControl { command: u8 },
    EmergencyStopPara { command: u8 },
    StartSequence { command: u8 },
    StopSequence { command: u8 },
    OpenPara { command: u8 },
    ClosePara { command: u8 },
    StartLogging { command: u8 },
    StopLogging { command: u8 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanRxMessage {
    LiftOff { value: u8 },
    Top { value: u8 },
    AngleSpeed { xyz: [i16; 3] },
    Acceleration { xyz: [i16; 3] },
    AirPressure { bytes: [u8; 3] },
    FinAngle { xyz: [i16; 3] },
    AccumulatedAngle { xyz: [i16; 3] },
    ControllerStatus { status: ControllerStatus },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanDecodeError {
    UnknownId(u16),
    InvalidDlc {
        id: u16,
        expected: usize,
        actual: usize,
    },
}

impl CanTxMessage {
    pub const fn id(&self) -> u16 {
        match self {
            Self::StopFinControl { .. } => CAN_ID_STOP_FIN_CONTROL,
            Self::EmergencyStopPara { .. } => CAN_ID_EMERGENCY_STOP_PARA,
            Self::StartSequence { .. } => CAN_ID_START_SEQUENCE,
            Self::StopSequence { .. } => CAN_ID_STOP_SEQUENCE,
            Self::OpenPara { .. } => CAN_ID_OPEN_PARA,
            Self::ClosePara { .. } => CAN_ID_CLOSE_PARA,
            Self::StartLogging { .. } => CAN_ID_START_LOGGING,
            Self::StopLogging { .. } => CAN_ID_STOP_LOGGING,
        }
    }

    pub const fn dlc(&self) -> usize {
        match self {
            Self::StopFinControl { .. }
            | Self::EmergencyStopPara { .. }
            | Self::StartSequence { .. }
            | Self::StopSequence { .. }
            | Self::OpenPara { .. }
            | Self::ClosePara { .. }
            | Self::StartLogging { .. }
            | Self::StopLogging { .. } => 1,
        }
    }

    pub fn encode_payload(&self, out: &mut [u8; 8]) -> usize {
        out.fill(0);

        match *self {
            Self::StopFinControl { command }
            | Self::EmergencyStopPara { command }
            | Self::StartSequence { command }
            | Self::StopSequence { command }
            | Self::OpenPara { command }
            | Self::ClosePara { command }
            | Self::StartLogging { command }
            | Self::StopLogging { command } => {
                out[0] = command;
            }
        }

        self.dlc()
    }
}

impl CanRxMessage {
    pub fn decode_standard(id: u16, data: &[u8]) -> Result<Self, CanDecodeError> {
        match id {
            CAN_ID_LIFT_OFF => {
                require_dlc(id, data, 1)?;
                Ok(Self::LiftOff { value: data[0] })
            }

            CAN_ID_TOP => {
                require_dlc(id, data, 1)?;
                Ok(Self::Top { value: data[0] })
            }

            CAN_ID_ANGLE_SPEED => {
                require_dlc(id, data, 6)?;
                Ok(Self::AngleSpeed {
                    xyz: decode_i16x3_be(data),
                })
            }

            CAN_ID_ACCELERATION => {
                require_dlc(id, data, 6)?;
                Ok(Self::Acceleration {
                    xyz: decode_i16x3_be(data),
                })
            }

            CAN_ID_AIR_PRESSURE => {
                require_dlc(id, data, 3)?;
                Ok(Self::AirPressure {
                    bytes: [data[0], data[1], data[2]],
                })
            }

            CAN_ID_FIN_ANGLE => {
                require_dlc(id, data, 6)?;
                Ok(Self::FinAngle {
                    xyz: decode_i16x3_be(data),
                })
            }

            CAN_ID_ACCUMULATED_ANGLE => {
                require_dlc(id, data, 6)?;
                Ok(Self::AccumulatedAngle {
                    xyz: decode_i16x3_be(data),
                })
            }

            CAN_ID_CONTROLLER_STATUS => {
                require_dlc(id, data, 1)?;
                Ok(Self::ControllerStatus {
                    status: ControllerStatus::from_raw(data[0]),
                })
            }

            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }
}

fn require_dlc(id: u16, data: &[u8], expected: usize) -> Result<(), CanDecodeError> {
    if data.len() == expected {
        Ok(())
    } else {
        Err(CanDecodeError::InvalidDlc {
            id,
            expected,
            actual: data.len(),
        })
    }
}

fn decode_i16x3_be(data: &[u8]) -> [i16; 3] {
    [
        i16::from_be_bytes([data[0], data[1]]),
        i16::from_be_bytes([data[2], data[3]]),
        i16::from_be_bytes([data[4], data[5]]),
    ]
}
