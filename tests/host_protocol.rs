#![allow(dead_code)]

#[path = "../src/payload.rs"]
mod payload;
#[path = "../src/can/protocol.rs"]
mod protocol;

use payload::{PAYLOAD_LEN, Payload};
use protocol::*;

fn message_cases() -> [(ComboardCanMessage, u16, usize, [u8; 8]); 16] {
    [
        (
            ComboardCanMessage::StopFinControl { command: b'E' },
            CAN_ID_STOP_FIN_CONTROL,
            1,
            [b'E', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::EmergencyStopPara { command: b'z' },
            CAN_ID_EMERGENCY_STOP_PARA,
            1,
            [b'z', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::StartSequence { command: b's' },
            CAN_ID_START_SEQUENCE,
            1,
            [b's', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::StopSequence { command: b'q' },
            CAN_ID_STOP_SEQUENCE,
            1,
            [b'q', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::OpenPara { command: b'o' },
            CAN_ID_OPEN_PARA,
            1,
            [b'o', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::ClosePara { command: b'c' },
            CAN_ID_CLOSE_PARA,
            1,
            [b'c', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::StartLogging { command: b'l' },
            CAN_ID_START_LOGGING,
            1,
            [b'l', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::StopLogging { command: b'm' },
            CAN_ID_STOP_LOGGING,
            1,
            [b'm', 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::LiftOff { value: 0xa5 },
            CAN_ID_LIFT_OFF,
            1,
            [0xa5, 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::Top { value: 0x5a },
            CAN_ID_TOP,
            1,
            [0x5a, 0, 0, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::AngleSpeed {
                xyz: [i16::MIN, -1, i16::MAX],
            },
            CAN_ID_ANGLE_SPEED,
            6,
            [0x80, 0x00, 0xff, 0xff, 0x7f, 0xff, 0, 0],
        ),
        (
            ComboardCanMessage::Acceleration {
                xyz: [0x1234, -0x1234, 0],
            },
            CAN_ID_ACCELERATION,
            6,
            [0x12, 0x34, 0xed, 0xcc, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::AirPressure {
                bytes: [0x12, 0x80, 0xff],
            },
            CAN_ID_AIR_PRESSURE,
            3,
            [0x12, 0x80, 0xff, 0, 0, 0, 0, 0],
        ),
        (
            ComboardCanMessage::FinAngle { xyz: [-2, 3, -4] },
            CAN_ID_FIN_ANGLE,
            6,
            [0xff, 0xfe, 0, 3, 0xff, 0xfc, 0, 0],
        ),
        (
            ComboardCanMessage::AccumulatedAngle {
                xyz: [256, -256, 1],
            },
            CAN_ID_ACCUMULATED_ANGLE,
            6,
            [1, 0, 0xff, 0, 0, 1, 0, 0],
        ),
        (
            ComboardCanMessage::IntegratedBoardStatus {
                phase: 0x12,
                flags: 0xa5,
            },
            CAN_ID_INTEGRATED_BOARD_STATUS,
            2,
            [0x12, 0xa5, 0, 0, 0, 0, 0, 0],
        ),
    ]
}

#[test]
fn every_message_has_expected_id_dlc_and_encoding() {
    for (message, expected_id, expected_dlc, expected_bytes) in message_cases() {
        let mut encoded = [0xff; 8];
        let encoded_len = message.encode_payload(&mut encoded);

        assert_eq!(message.id(), expected_id, "{message:?}");
        assert_eq!(message.dlc(), expected_dlc, "{message:?}");
        assert_eq!(encoded_len, expected_dlc, "{message:?}");
        assert_eq!(encoded, expected_bytes, "{message:?}");
    }
}

#[test]
fn every_message_round_trips() {
    for (message, _, _, _) in message_cases() {
        let mut encoded = [0; 8];
        let len = message.encode_payload(&mut encoded);
        let decoded = ComboardCanMessage::decode_standard(message.id(), &encoded[..len]);

        assert_eq!(decoded, Ok(message), "{message:?}");
    }
}

#[test]
fn every_message_rejects_wrong_dlc() {
    for (message, _, expected_dlc, _) in message_cases() {
        for actual_dlc in 0..=8 {
            if actual_dlc == expected_dlc {
                continue;
            }

            assert_eq!(
                ComboardCanMessage::decode_standard(message.id(), &[0; 8][..actual_dlc]),
                Err(CanDecodeError::InvalidDlc {
                    id: message.id(),
                    expected: expected_dlc,
                    actual: actual_dlc,
                }),
                "{message:?} accepted DLC {actual_dlc}",
            );
        }
    }
}

#[test]
fn unknown_standard_id_is_rejected() {
    assert_eq!(
        ComboardCanMessage::decode_standard(0x7ff, &[]),
        Err(CanDecodeError::UnknownId(0x7ff)),
    );
}

#[test]
fn lora_payload_binary_layout_and_size_are_unchanged() {
    let payload = Payload {
        add_h: 0x01,
        add_l: 0x02,
        chnnl: 0x03,
        header1: 0x04,
        status: 0x05,
        gnss_lat: 0x1234_5678,
        gnss_long: -0x1234_5678,
        gnss_height: -2,
        angle_speed: [1, -1, i16::MAX],
        acceleration: [i16::MIN, 0x1234, -0x1234],
        integrated_angle: [256, -256, 0],
        air_pressure: [0xaa, 0xbb, 0xcc],
        air_speed: 0xdd,
        fin_angle: -2,
        check_sum: 0xee,
    };

    let bytes = payload.to_bytes();
    assert_eq!(PAYLOAD_LEN, 39);
    assert_eq!(bytes.len(), 39);
    assert_eq!(
        bytes,
        [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x78, 0x56, 0x34, 0x12, 0x88, 0xa9, 0xcb, 0xed, 0xfe,
            0xff, 0x01, 0x00, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x80, 0x34, 0x12, 0xcc, 0xed, 0x00,
            0x01, 0x00, 0xff, 0x00, 0x00, 0xaa, 0xbb, 0xcc, 0xdd, 0xfe, 0xee,
        ],
    );
}

#[test]
fn payload_checksum_excludes_radio_address_and_checksum_byte() {
    let mut payload = Payload::new();
    payload.header1 = 0xaa;
    payload.status = 0x55;
    payload.gnss_lat = 0x0102_0304;
    payload.check_sum = 0xff;

    let bytes = payload.to_bytes();
    let expected = bytes[3..PAYLOAD_LEN - 1]
        .iter()
        .fold(0, |checksum, byte| checksum ^ byte);
    assert_eq!(payload.calculate_checksum(), expected);
}
