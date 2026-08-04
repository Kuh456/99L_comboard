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
        match self.len {
            0 => {
                if byte == HEADER {
                    self.bytes[0] = byte;
                    self.len = 1;
                }
                None
            }
            1 => {
                if byte == HEADER {
                    self.bytes[0] = byte;
                } else {
                    self.bytes[1] = byte;
                    self.len = 2;
                }
                None
            }
            _ => {
                let command = self.bytes[1];
                self.len = if byte == HEADER { 1 } else { 0 };
                (byte == HEADER ^ command).then_some(command)
            }
        }
    }

    pub(crate) fn reset(&mut self) {
        self.len = 0;
    }
}
