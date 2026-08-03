const HEADER: u8 = 0x55;

pub(crate) struct UplinkFrameBuffer {
    bytes: [u8; 3],
    len: usize,
}

impl UplinkFrameBuffer {
    pub(crate) const fn new() -> Self {
        Self {
            bytes: [0; 3],
            len: 0,
        }
    }

    pub(crate) fn push(&mut self, byte: u8) -> Option<u8> {
        self.bytes[self.len] = byte;
        self.len += 1;

        if self.len < self.bytes.len() {
            return None;
        }

        self.len = 0;
        let [header, command, checksum] = self.bytes;
        if header == HEADER && checksum == header ^ command {
            Some(command)
        } else {
            None
        }
    }
}
