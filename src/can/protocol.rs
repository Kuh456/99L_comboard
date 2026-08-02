pub const CAN_ID_EMERGENCY_STOP_PARA: u16 = 0x003;
pub const CAN_ID_START_SEQUENCE: u16 = 0x005;
pub const CAN_ID_STOP_SEQUENCE: u16 = 0x00a;

pub const CAN_ID_START_LOGGING: u16 = 0x011;
pub const CAN_ID_STOP_LOGGING: u16 = 0x01e;

// 現在の指定値。優先度については後述。
pub const CAN_ID_OPEN_PARA: u16 = 0x30d;
pub const CAN_ID_CLOSE_PARA: u16 = 0x30e;

pub const CAN_ID_AIR_PRESSURE: u16 = 0x10a;
pub const CAN_ID_LIFT_OFF: u16 = 0x110;
pub const CAN_ID_ACCELERATION: u16 = 0x11a;
pub const CAN_ID_ANGLE_SPEED: u16 = 0x120;
pub const CAN_ID_TOP: u16 = 0x12a;
pub const CAN_ID_FIN_ANGLE: u16 = 0x13a;
pub const CAN_ID_ACCUMULATED_ANGLE: u16 = 0x14a;

pub const CAN_ID_INTEGRATED_BOARD_STATUS: u16 = 0x200;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComboardCanMessage {
    // 通信基板 → 他基板
    EmergencyStopPara { command: u8 },
    StartSequence { command: u8 },
    StopSequence { command: u8 },
    OpenPara { command: u8 },
    ClosePara { command: u8 },
    StartLogging { command: u8 },
    StopLogging { command: u8 },

    // 他基板 → 通信基板
    LiftOff { value: u8 },
    Top { value: u8 },
    AngleSpeed { xyz: [i16; 3] },
    Acceleration { xyz: [i16; 3] },
    AirPressure { bytes: [u8; 3] },
    FinAngle { xyz: [i16; 3] },
    AccumulatedAngle { xyz: [i16; 3] },
    IntegratedBoardStatus { phase: u8, flags: u8 },
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

impl ComboardCanMessage {
    pub const fn id(&self) -> u16 {
        match self {
            Self::EmergencyStopPara { .. } => CAN_ID_EMERGENCY_STOP_PARA,
            Self::StartSequence { .. } => CAN_ID_START_SEQUENCE,
            Self::StopSequence { .. } => CAN_ID_STOP_SEQUENCE,
            Self::OpenPara { .. } => CAN_ID_OPEN_PARA,
            Self::ClosePara { .. } => CAN_ID_CLOSE_PARA,
            Self::StartLogging { .. } => CAN_ID_START_LOGGING,
            Self::StopLogging { .. } => CAN_ID_STOP_LOGGING,

            Self::LiftOff { .. } => CAN_ID_LIFT_OFF,
            Self::Top { .. } => CAN_ID_TOP,
            Self::AngleSpeed { .. } => CAN_ID_ANGLE_SPEED,
            Self::Acceleration { .. } => CAN_ID_ACCELERATION,
            Self::AirPressure { .. } => CAN_ID_AIR_PRESSURE,
            Self::FinAngle { .. } => CAN_ID_FIN_ANGLE,
            Self::AccumulatedAngle { .. } => CAN_ID_ACCUMULATED_ANGLE,
            Self::IntegratedBoardStatus { .. } => CAN_ID_INTEGRATED_BOARD_STATUS,
        }
    }

    pub const fn dlc(&self) -> usize {
        match self {
            Self::EmergencyStopPara { .. }
            | Self::StartSequence { .. }
            | Self::StopSequence { .. }
            | Self::OpenPara { .. }
            | Self::ClosePara { .. }
            | Self::StartLogging { .. }
            | Self::StopLogging { .. }
            | Self::LiftOff { .. }
            | Self::Top { .. } => 1,

            Self::IntegratedBoardStatus { .. } => 2,

            Self::AirPressure { .. } => 3,

            Self::AngleSpeed { .. }
            | Self::Acceleration { .. }
            | Self::FinAngle { .. }
            | Self::AccumulatedAngle { .. } => 6,
        }
    }

    pub fn encode_payload(&self, out: &mut [u8; 8]) -> usize {
        out.fill(0);

        match *self {
            Self::EmergencyStopPara { command }
            | Self::StartSequence { command }
            | Self::StopSequence { command }
            | Self::OpenPara { command }
            | Self::ClosePara { command }
            | Self::StartLogging { command }
            | Self::StopLogging { command } => {
                out[0] = command;
            }

            Self::LiftOff { value } | Self::Top { value } => {
                out[0] = value;
            }

            Self::AngleSpeed { xyz }
            | Self::Acceleration { xyz }
            | Self::FinAngle { xyz }
            | Self::AccumulatedAngle { xyz } => {
                encode_i16x3_be(xyz, out);
            }

            Self::AirPressure { bytes } => {
                out[0..3].copy_from_slice(&bytes);
            }

            Self::IntegratedBoardStatus { phase, flags } => {
                out[0] = phase;
                out[1] = flags;
            }
        }

        self.dlc()
    }

    pub fn decode_standard(id: u16, data: &[u8]) -> Result<Self, CanDecodeError> {
        match id {
            CAN_ID_EMERGENCY_STOP_PARA => {
                require_dlc(id, data, 1)?;
                Ok(Self::EmergencyStopPara { command: data[0] })
            }

            CAN_ID_START_SEQUENCE => {
                require_dlc(id, data, 1)?;
                Ok(Self::StartSequence { command: data[0] })
            }

            CAN_ID_STOP_SEQUENCE => {
                require_dlc(id, data, 1)?;
                Ok(Self::StopSequence { command: data[0] })
            }

            CAN_ID_OPEN_PARA => {
                require_dlc(id, data, 1)?;
                Ok(Self::OpenPara { command: data[0] })
            }

            CAN_ID_CLOSE_PARA => {
                require_dlc(id, data, 1)?;
                Ok(Self::ClosePara { command: data[0] })
            }

            CAN_ID_START_LOGGING => {
                require_dlc(id, data, 1)?;
                Ok(Self::StartLogging { command: data[0] })
            }

            CAN_ID_STOP_LOGGING => {
                require_dlc(id, data, 1)?;
                Ok(Self::StopLogging { command: data[0] })
            }

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

            CAN_ID_INTEGRATED_BOARD_STATUS => {
                require_dlc(id, data, 2)?;
                Ok(Self::IntegratedBoardStatus {
                    phase: data[0],
                    flags: data[1],
                })
            }

            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }

    pub const fn is_command(&self) -> bool {
        matches!(
            self,
            Self::EmergencyStopPara { .. }
                | Self::StartSequence { .. }
                | Self::StopSequence { .. }
                | Self::OpenPara { .. }
                | Self::ClosePara { .. }
                | Self::StartLogging { .. }
                | Self::StopLogging { .. }
        )
    }

    pub const fn is_telemetry(&self) -> bool {
        !self.is_command()
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

fn encode_i16x3_be(values: [i16; 3], out: &mut [u8; 8]) {
    for (index, value) in values.into_iter().enumerate() {
        let offset = index * 2;
        let bytes = value.to_be_bytes();

        out[offset] = bytes[0];
        out[offset + 1] = bytes[1];
    }
}

fn decode_i16x3_be(data: &[u8]) -> [i16; 3] {
    [
        i16::from_be_bytes([data[0], data[1]]),
        i16::from_be_bytes([data[2], data[3]]),
        i16::from_be_bytes([data[4], data[5]]),
    ]
}
