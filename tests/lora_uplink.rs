#[path = "../src/lora_uplink.rs"]
mod lora_uplink;

use lora_uplink::UplinkFrameBuffer;

fn collect_commands(buffer: &mut UplinkFrameBuffer, bytes: &[u8]) -> Vec<u8> {
    bytes.iter().filter_map(|&byte| buffer.push(byte)).collect()
}

#[test]
fn accepts_valid_frame() {
    let mut buffer = UplinkFrameBuffer::new();
    assert_eq!(collect_commands(&mut buffer, &[0x55, b's', 0x26]), [b's']);
}

#[test]
fn rejects_wrong_header() {
    let mut buffer = UplinkFrameBuffer::new();
    assert!(collect_commands(&mut buffer, &[0x54, b's', 0x27]).is_empty());
}

#[test]
fn rejects_wrong_checksum() {
    let mut buffer = UplinkFrameBuffer::new();
    assert!(collect_commands(&mut buffer, &[0x55, b's', 0x00]).is_empty());
}

#[test]
fn resynchronizes_at_the_next_header_after_garbage() {
    let mut buffer = UplinkFrameBuffer::new();
    assert_eq!(
        collect_commands(&mut buffer, &[0x00, 0xff, 0x55, b's', 0x26]),
        [b's']
    );
}

#[test]
fn checksum_failure_can_end_with_the_next_header() {
    let mut buffer = UplinkFrameBuffer::new();
    assert_eq!(
        collect_commands(&mut buffer, &[0x55, b's', 0x55, b'q', 0x24]),
        [b'q']
    );
}

#[test]
fn accepts_split_frame() {
    let mut buffer = UplinkFrameBuffer::new();
    assert!(collect_commands(&mut buffer, &[0x55]).is_empty());
    assert!(collect_commands(&mut buffer, &[b's']).is_empty());
    assert_eq!(collect_commands(&mut buffer, &[0x26]), [b's']);
}

#[test]
fn reset_discards_a_partial_frame() {
    let mut buffer = UplinkFrameBuffer::new();
    assert!(buffer.push(0x55).is_none());

    buffer.reset();

    assert_eq!(collect_commands(&mut buffer, &[0x55, b'q', 0x24]), [b'q']);
}

#[test]
fn accepts_consecutive_frames() {
    let mut buffer = UplinkFrameBuffer::new();
    assert_eq!(
        collect_commands(&mut buffer, &[0x55, b's', 0x26, 0x55, b'q', 0x24]),
        [b's', b'q']
    );
}
