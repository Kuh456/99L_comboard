pub const PAYLOAD_LEN: usize = 39;

#[derive(Debug, Copy, Clone)]
pub struct Payload {
    pub add_h: u8,
    pub add_l: u8,
    pub chnnl: u8,
    pub header1: u8,
    pub status: u8,
    pub gnss_lat: i32,
    pub gnss_long: i32,
    pub gnss_height: i16,
    pub angle_speed: [i16; 3],
    pub acceleration: [i16; 3],
    pub integrated_angle: [i16; 3],
    pub air_pressure: [u8; 3],
    pub air_speed: u8,
    pub fin_angle: i8,
    pub check_sum: u8,
}

impl Payload {
    pub const fn new() -> Self {
        Self {
            add_h: 0x00,
            add_l: 0x00,
            chnnl: 0x03,
            header1: 0xaa,
            status: 0,
            gnss_lat: 0,
            gnss_long: 0,
            gnss_height: 0,
            angle_speed: [0; 3],
            acceleration: [0; 3],
            integrated_angle: [0; 3],
            air_pressure: [0; 3],
            air_speed: 0,
            fin_angle: 0,
            check_sum: 0,
        }
    }

    pub fn calculate_checksum(&self) -> u8 {
        let bytes = self.to_bytes();
        let mut sum = 0;

        for &byte in &bytes[3..bytes.len() - 1] {
            sum ^= byte;
        }

        sum
    }

    pub fn to_bytes(&self) -> [u8; PAYLOAD_LEN] {
        let mut buf = [0u8; PAYLOAD_LEN];
        let mut offset = 0;

        buf[offset] = self.add_h;
        offset += 1;
        buf[offset] = self.add_l;
        offset += 1;
        buf[offset] = self.chnnl;
        offset += 1;
        buf[offset] = self.header1;
        offset += 1;
        buf[offset] = self.status;
        offset += 1;

        buf[offset..offset + 4].copy_from_slice(&self.gnss_lat.to_le_bytes());
        offset += 4;
        buf[offset..offset + 4].copy_from_slice(&self.gnss_long.to_le_bytes());
        offset += 4;
        buf[offset..offset + 2].copy_from_slice(&self.gnss_height.to_le_bytes());
        offset += 2;

        for &val in &self.angle_speed {
            buf[offset..offset + 2].copy_from_slice(&val.to_le_bytes());
            offset += 2;
        }

        for &val in &self.acceleration {
            buf[offset..offset + 2].copy_from_slice(&val.to_le_bytes());
            offset += 2;
        }

        for &val in &self.integrated_angle {
            buf[offset..offset + 2].copy_from_slice(&val.to_le_bytes());
            offset += 2;
        }

        for &val in &self.air_pressure {
            buf[offset] = val;
            offset += 1;
        }

        buf[offset] = self.air_speed;
        offset += 1;

        buf[offset] = self.fin_angle as u8;
        offset += 1;

        buf[offset] = self.check_sum;

        buf
    }
}

impl Default for Payload {
    fn default() -> Self {
        Self::new()
    }
}

pub fn encode_height_10m_i16(height_m: f32) -> i16 {
    if !height_m.is_finite() {
        return 0;
    }

    let scaled = if height_m >= 0.0 {
        height_m / 10.0 + 0.5
    } else {
        height_m / 10.0 - 0.5
    };

    if scaled > i16::MAX as f32 {
        i16::MAX
    } else if scaled < i16::MIN as f32 {
        i16::MIN
    } else {
        scaled as i16
    }
}

pub fn decode_height_10m_i16(raw: i16) -> f32 {
    raw as f32 * 10.0
}

pub fn encode_fin_angle_i8(angle: i16) -> i8 {
    if angle > i8::MAX as i16 {
        i8::MAX
    } else if angle < i8::MIN as i16 {
        i8::MIN
    } else {
        angle as i8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_is_serialized_in_the_wire_format() {
        let payload = Payload {
            add_h: 0x00,
            add_l: 0x00,
            chnnl: 0x03,
            header1: 0xaa,
            status: 0x55,
            gnss_lat: 0x0102_0304,
            gnss_long: -2,
            gnss_height: 0x1122,
            angle_speed: [1, 2, 3],
            acceleration: [-1, -2, -3],
            integrated_angle: [0x1122, 0x3344, 0x5566],
            air_pressure: [0x10, 0x20, 0x30],
            air_speed: 0x77,
            fin_angle: -1,
            check_sum: 0xa5,
        };

        assert_eq!(
            payload.to_bytes(),
            [
                0x00, 0x00, 0x03, 0xaa, 0x55, 0x04, 0x03, 0x02, 0x01, 0xfe, 0xff, 0xff, 0xff, 0x22,
                0x11, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0xff, 0xff, 0xfe, 0xff, 0xfd, 0xff, 0x22,
                0x11, 0x44, 0x33, 0x66, 0x55, 0x10, 0x20, 0x30, 0x77, 0xff, 0xa5,
            ]
        );
    }

    #[test]
    fn checksum_covers_header_through_fin_angle() {
        let mut payload = Payload::new();
        payload.status = 0x55;
        payload.integrated_angle = [1, 2, 3];
        payload.air_speed = 0x77;
        payload.fin_angle = -1;

        let bytes = payload.to_bytes();
        let expected = bytes[3..PAYLOAD_LEN - 1]
            .iter()
            .copied()
            .fold(0, core::ops::BitXor::bitxor);

        assert_eq!(payload.calculate_checksum(), expected);
    }
}
