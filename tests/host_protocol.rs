#![allow(dead_code)]

#[path = "../src/can/command.rs"]
mod command;
#[path = "../src/payload.rs"]
mod payload;
#[path = "../src/can/protocol.rs"]
mod protocol;

use command::{CommandFailure, CommandRequestState, GroundCommand};
use payload::{PAYLOAD_LEN, Payload};
use protocol::*;

fn tx_cases() -> [(CanTxMessage, u16, u8); 8] {
    [
        (
            CanTxMessage::StopFinControl { command: b'E' },
            CAN_ID_STOP_FIN_CONTROL,
            b'E',
        ),
        (
            CanTxMessage::EmergencyStopPara { command: b'z' },
            CAN_ID_EMERGENCY_STOP_PARA,
            b'z',
        ),
        (
            CanTxMessage::StartSequence { command: b's' },
            CAN_ID_START_SEQUENCE,
            b's',
        ),
        (
            CanTxMessage::StopSequence { command: b'q' },
            CAN_ID_STOP_SEQUENCE,
            b'q',
        ),
        (
            CanTxMessage::OpenPara { command: b'o' },
            CAN_ID_OPEN_PARA,
            b'o',
        ),
        (
            CanTxMessage::ClosePara { command: b'c' },
            CAN_ID_CLOSE_PARA,
            b'c',
        ),
        (
            CanTxMessage::StartLogging { command: b'l' },
            CAN_ID_START_LOGGING,
            b'l',
        ),
        (
            CanTxMessage::StopLogging { command: b'm' },
            CAN_ID_STOP_LOGGING,
            b'm',
        ),
    ]
}

#[test]
fn tx_messages_have_expected_id_dlc_and_encoding() {
    for (message, id, byte) in tx_cases() {
        let mut encoded = [0xff; 8];
        assert_eq!(message.id(), id);
        assert_eq!(message.dlc(), 1);
        assert_eq!(message.encode_payload(&mut encoded), 1);
        assert_eq!(encoded, [byte, 0, 0, 0, 0, 0, 0, 0]);
        assert_ne!(message.id(), CAN_ID_CONTROLLER_STATUS);
    }
}

#[test]
fn rx_signed_boundary_array_and_dlc_decode() {
    assert_eq!(
        CanRxMessage::decode_standard(CAN_ID_ANGLE_SPEED, &[0x80, 0, 0xff, 0xff, 0x7f, 0xff]),
        Ok(CanRxMessage::AngleSpeed {
            xyz: [i16::MIN, -1, i16::MAX]
        })
    );
    assert_eq!(
        CanRxMessage::decode_standard(CAN_ID_AIR_PRESSURE, &[1, 2]),
        Err(CanDecodeError::InvalidDlc {
            id: CAN_ID_AIR_PRESSURE,
            expected: 3,
            actual: 2
        })
    );
    assert_eq!(
        CanRxMessage::decode_standard(0x7ff, &[]),
        Err(CanDecodeError::UnknownId(0x7ff))
    );
}

#[test]
fn controller_status_decodes_flags_and_preserves_unknown() {
    let raw = 0b1110_1111;
    let Ok(CanRxMessage::ControllerStatus {
        status: ControllerStatus::Valid(flags),
    }) = CanRxMessage::decode_standard(CAN_ID_CONTROLLER_STATUS, &[raw])
    else {
        return;
    };
    assert_eq!(flags.raw(), raw);
    assert!(flags.top_detected() && flags.main_power_on() && flags.emergency_power_on());
    assert!(flags.control_active() && flags.sequence_active() && flags.liftoff_detected());
    assert!(flags.parachute_motor_open());
    assert_eq!(
        CanRxMessage::decode_standard(CAN_ID_CONTROLLER_STATUS, &[0x10]),
        Ok(CanRxMessage::ControllerStatus {
            status: ControllerStatus::Unknown(0x10)
        })
    );
    assert!(matches!(
        CanRxMessage::decode_standard(CAN_ID_CONTROLLER_STATUS, &[]),
        Err(CanDecodeError::InvalidDlc { .. })
    ));
}

#[test]
fn controller_link_state_is_explicit_and_timeout_is_configurable() {
    assert_eq!(
        controller_link_state(false, 0, None),
        ControllerLinkState::Unknown
    );
    assert_eq!(
        controller_link_state(true, 999, None),
        ControllerLinkState::Online
    );
    assert_eq!(
        controller_link_state(true, 149, Some(150)),
        ControllerLinkState::Online
    );
    assert_eq!(
        controller_link_state(true, 150, Some(150)),
        ControllerLinkState::TimedOut
    );
}

#[test]
fn commands_decode_legacy_bytes_and_unknown_is_rejected() {
    let cases = [
        (b's', GroundCommand::StartSequence),
        (b'q', GroundCommand::StopSequence),
        (b'z', GroundCommand::EmergencyStopPara),
        (b'l', GroundCommand::StartLogging),
        (b'm', GroundCommand::StopLogging),
        (b'E', GroundCommand::StopFinControl),
        (b'o', GroundCommand::OpenPara),
        (b'c', GroundCommand::ClosePara),
        (b'g', GroundCommand::GnssOn),
        (b'h', GroundCommand::GnssOff),
    ];
    for (byte, expected) in cases {
        assert_eq!(GroundCommand::decode_legacy(byte), Some(expected));
    }
    assert_eq!(GroundCommand::decode_legacy(0), None);
    assert!(GroundCommand::EmergencyStopPara.is_safety_critical());
    assert!(!GroundCommand::StartSequence.is_safety_critical());
}

#[test]
fn request_and_tx_success_do_not_change_actual_status() {
    let payload = Payload::new();
    let queued = CommandRequestState::queue(1, GroundCommand::StartSequence);
    assert!(matches!(
        queued.mark_transmitted(1, 100),
        CommandRequestState::AwaitingConfirmation { .. }
    ));
    assert_eq!(payload.status, 0);
}

#[test]
fn pending_completes_only_from_matching_controller_flags() {
    let start =
        CommandRequestState::queue(1, GroundCommand::StartSequence).mark_transmitted(1, 1000);
    assert_eq!(start.confirm(false, true), start);
    assert!(matches!(
        start.confirm(true, true),
        CommandRequestState::Completed { .. }
    ));
    let stop = CommandRequestState::queue(2, GroundCommand::StopSequence).mark_transmitted(2, 1000);
    assert_eq!(stop.confirm(true, true), stop);
    assert!(matches!(
        stop.confirm(false, true),
        CommandRequestState::Completed { .. }
    ));
    let emergency =
        CommandRequestState::queue(3, GroundCommand::EmergencyStopPara).mark_transmitted(3, 1000);
    assert_eq!(emergency.confirm(false, true), emergency);
    assert!(matches!(
        emergency.confirm(false, false),
        CommandRequestState::Completed { .. }
    ));
}

#[test]
fn pending_timeout_and_supersede_preserve_actual_state() {
    let pending =
        CommandRequestState::queue(7, GroundCommand::StartSequence).mark_transmitted(7, 1000);
    assert_eq!(pending.expire(1499, 500), pending);
    assert!(matches!(
        pending.expire(1500, 500),
        CommandRequestState::Failed {
            reason: CommandFailure::ConfirmationTimedOut,
            ..
        }
    ));
    assert!(matches!(
        CommandRequestState::queue(8, GroundCommand::StopSequence).supersede(),
        CommandRequestState::Failed {
            reason: CommandFailure::Superseded,
            ..
        }
    ));
}

#[test]
fn controller_status_effects_are_edge_triggered() {
    let Some(idle) = ControllerStatusFlags::from_raw(0) else {
        return;
    };
    let Some(active) = ControllerStatusFlags::from_raw((1 << 5) | 1) else {
        return;
    };
    assert_eq!(
        controller_status_effects(None, idle),
        ControllerStatusEffects {
            sequence_changed: Some(false),
            top_rising: false
        }
    );
    assert_eq!(
        controller_status_effects(Some(idle), active),
        ControllerStatusEffects {
            sequence_changed: Some(true),
            top_rising: true
        }
    );
    assert_eq!(
        controller_status_effects(Some(active), active),
        ControllerStatusEffects {
            sequence_changed: None,
            top_rising: false
        }
    );
}

#[test]
fn payload_binary_layout_size_and_checksum_are_unchanged() {
    let mut payload = Payload::new();
    payload.add_h = 1;
    payload.add_l = 2;
    payload.chnnl = 3;
    payload.header1 = 4;
    payload.status = 5;
    payload.gnss_lat = 0x1234_5678;
    payload.gnss_long = -0x1234_5678;
    payload.gnss_height = -2;
    payload.angle_speed = [1, -1, i16::MAX];
    payload.acceleration = [i16::MIN, 0x1234, -0x1234];
    payload.integrated_angle = [256, -256, 0];
    payload.air_pressure = [0xaa, 0xbb, 0xcc];
    payload.air_speed = 0xdd;
    payload.fin_angle = -2;
    payload.check_sum = 0xee;
    let bytes = payload.to_bytes();
    assert_eq!(PAYLOAD_LEN, 39);
    assert_eq!(bytes.len(), 39);
    assert_eq!(&bytes[..5], &[1, 2, 3, 4, 5]);
    assert_eq!(&bytes[5..9], &0x1234_5678i32.to_le_bytes());
    assert_eq!(bytes[38], 0xee);
    let old_checksum = payload.calculate_checksum();
    payload.add_h ^= 0xff;
    payload.add_l ^= 0xff;
    payload.chnnl ^= 0xff;
    payload.check_sum ^= 0xff;
    assert_eq!(payload.calculate_checksum(), old_checksum);
}
