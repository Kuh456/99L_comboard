#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundCommand {
    StartSequence,
    StopSequence,
    EmergencyStopPara,
    StartLogging,
    StopLogging,
    StopFinControl,
    OpenPara,
    ClosePara,
    GnssOn,
    GnssOff,
}

impl GroundCommand {
    pub const fn decode_legacy(byte: u8) -> Option<Self> {
        match byte {
            b's' => Some(Self::StartSequence),
            b'q' => Some(Self::StopSequence),
            b'z' => Some(Self::EmergencyStopPara),
            b'l' => Some(Self::StartLogging),
            b'm' => Some(Self::StopLogging),
            b'E' => Some(Self::StopFinControl),
            b'o' => Some(Self::OpenPara),
            b'c' => Some(Self::ClosePara),
            b'g' => Some(Self::GnssOn),
            b'h' => Some(Self::GnssOff),
            _ => None,
        }
    }

    pub const fn is_confirmed_by_controller_status(self) -> bool {
        matches!(
            self,
            Self::StartSequence | Self::StopSequence | Self::EmergencyStopPara
        )
    }

    pub const fn is_safety_critical(self) -> bool {
        matches!(self, Self::EmergencyStopPara)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandFailure {
    TransmitFailed,
    ConfirmationTimedOut,
    Superseded,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandFailureRecord {
    pub token: u32,
    pub command: GroundCommand,
    pub reason: CommandFailure,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandRequestState {
    Idle,
    Queued {
        token: u32,
        command: GroundCommand,
    },
    AwaitingConfirmation {
        token: u32,
        command: GroundCommand,
        transmitted_at_ms: u64,
    },
    Completed {
        token: u32,
        command: GroundCommand,
    },
    Failed {
        token: u32,
        command: GroundCommand,
        reason: CommandFailure,
    },
}

impl CommandRequestState {
    pub const fn queue(token: u32, command: GroundCommand) -> Self {
        Self::Queued { token, command }
    }

    pub const fn is_in_flight(self) -> bool {
        matches!(
            self,
            Self::Queued { .. } | Self::AwaitingConfirmation { .. }
        )
    }

    pub const fn mark_transmitted(self, token: u32, now_ms: u64) -> Self {
        match self {
            Self::Queued {
                token: current_token,
                command,
            } if current_token == token => Self::AwaitingConfirmation {
                token,
                command,
                transmitted_at_ms: now_ms,
            },
            state => state,
        }
    }

    pub const fn mark_transmit_failed(self, token: u32) -> Self {
        match self {
            Self::Queued {
                token: current_token,
                command,
            } if current_token == token => Self::Failed {
                token,
                command,
                reason: CommandFailure::TransmitFailed,
            },
            state => state,
        }
    }

    pub const fn confirm(self, sequence_active: bool, liftoff_detected: bool) -> Self {
        match self {
            Self::AwaitingConfirmation { token, command, .. }
                if command_completed(command, sequence_active, liftoff_detected) =>
            {
                Self::Completed { token, command }
            }
            state => state,
        }
    }

    pub const fn expire(self, now_ms: u64, timeout_ms: u64) -> Self {
        match self {
            Self::AwaitingConfirmation {
                token,
                command,
                transmitted_at_ms,
            } if now_ms.saturating_sub(transmitted_at_ms) >= timeout_ms => Self::Failed {
                token,
                command,
                reason: CommandFailure::ConfirmationTimedOut,
            },
            state => state,
        }
    }

    pub const fn supersede(self) -> Self {
        match self {
            Self::Queued { token, command } | Self::AwaitingConfirmation { token, command, .. } => {
                Self::Failed {
                    token,
                    command,
                    reason: CommandFailure::Superseded,
                }
            }
            state => state,
        }
    }

    pub const fn failure(self) -> Option<CommandFailureRecord> {
        match self {
            Self::Failed {
                token,
                command,
                reason,
            } => Some(CommandFailureRecord {
                token,
                command,
                reason,
            }),
            _ => None,
        }
    }
}

pub const fn command_completed(
    command: GroundCommand,
    sequence_active: bool,
    liftoff_detected: bool,
) -> bool {
    match command {
        GroundCommand::StartSequence => sequence_active,
        GroundCommand::StopSequence => !sequence_active,
        GroundCommand::EmergencyStopPara => !liftoff_detected,
        GroundCommand::StartLogging
        | GroundCommand::StopLogging
        | GroundCommand::StopFinControl
        | GroundCommand::OpenPara
        | GroundCommand::ClosePara
        | GroundCommand::GnssOn
        | GroundCommand::GnssOff => false,
    }
}
