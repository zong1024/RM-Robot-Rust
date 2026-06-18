//! Camera-to-robot vision packet definitions.
//!
//! The protocol intentionally carries compact frame summaries instead of full
//! RGB/depth images. Full 640x360 depth plus RGB frames exceed what the STM32
//! control board should receive in a real-time control loop.

pub const VISION_MAGIC: [u8; 2] = *b"OB";
pub const VISION_VERSION: u8 = 1;
pub const VISION_PACKET_FRAME_SUMMARY: u8 = 1;
pub const VISION_HEADER_LEN: usize = 16;
pub const VISION_FRAME_SUMMARY_PAYLOAD_LEN: usize = 50;
pub const VISION_MAX_PAYLOAD_LEN: usize = VISION_FRAME_SUMMARY_PAYLOAD_LEN;
pub const VISION_MAX_PACKET_LEN: usize = VISION_HEADER_LEN + VISION_MAX_PAYLOAD_LEN;
pub const VISION_FLAG_DEPTH_VALID: u16 = 1 << 0;
pub const VISION_FLAG_RGB_VALID: u16 = 1 << 1;
pub const VISION_GRID_CELLS: usize = 16;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VisionFrameSummary {
    pub sequence: u32,
    pub captured_at_ms: u32,
    pub flags: u16,
    pub depth_width: u16,
    pub depth_height: u16,
    pub rgb_width: u16,
    pub rgb_height: u16,
    pub depth_min_mm: u16,
    pub depth_max_mm: u16,
    pub depth_center_mm: u16,
    pub depth_mean_mm: u16,
    /// 4x4 depth sample grid in millimeters, row-major. A value of 0 means no
    /// valid depth was present in that cell.
    pub depth_grid_mm: [u16; VISION_GRID_CELLS],
}

impl VisionFrameSummary {
    pub const fn has_depth(&self) -> bool {
        self.flags & VISION_FLAG_DEPTH_VALID != 0
    }

    pub const fn has_rgb(&self) -> bool {
        self.flags & VISION_FLAG_RGB_VALID != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VisionPacket {
    FrameSummary(VisionFrameSummary),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VisionParseError {
    BadVersion,
    BadLength,
    BadCrc,
    UnknownPacket,
}

#[derive(Clone, Copy, Debug)]
pub struct VisionPacketParser {
    buffer: [u8; VISION_MAX_PACKET_LEN],
    len: usize,
    expected_len: usize,
}

impl VisionPacketParser {
    pub const fn new() -> Self {
        Self {
            buffer: [0; VISION_MAX_PACKET_LEN],
            len: 0,
            expected_len: VISION_HEADER_LEN,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.expected_len = VISION_HEADER_LEN;
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<VisionPacket>, VisionParseError> {
        if self.len == 0 && byte != VISION_MAGIC[0] {
            return Ok(None);
        }
        if self.len == 1 && byte != VISION_MAGIC[1] {
            self.reset();
            if byte == VISION_MAGIC[0] {
                self.buffer[0] = byte;
                self.len = 1;
            }
            return Ok(None);
        }

        if self.len >= self.buffer.len() {
            self.reset();
            return Err(VisionParseError::BadLength);
        }

        self.buffer[self.len] = byte;
        self.len += 1;

        if self.len == VISION_HEADER_LEN {
            if self.buffer[2] != VISION_VERSION {
                self.reset();
                return Err(VisionParseError::BadVersion);
            }
            let payload_len = u16::from_le_bytes([self.buffer[12], self.buffer[13]]) as usize;
            if payload_len > VISION_MAX_PAYLOAD_LEN {
                self.reset();
                return Err(VisionParseError::BadLength);
            }
            self.expected_len = VISION_HEADER_LEN + payload_len;
        }

        if self.len < self.expected_len {
            return Ok(None);
        }

        let packet = self.decode_packet();
        self.reset();
        packet.map(Some)
    }

    fn decode_packet(&self) -> Result<VisionPacket, VisionParseError> {
        let received_crc = u16::from_le_bytes([self.buffer[14], self.buffer[15]]);
        let mut crc_input = [0u8; VISION_HEADER_LEN - 2 + VISION_MAX_PAYLOAD_LEN];
        crc_input[..14].copy_from_slice(&self.buffer[..14]);
        let payload_len = u16::from_le_bytes([self.buffer[12], self.buffer[13]]) as usize;
        crc_input[14..14 + payload_len]
            .copy_from_slice(&self.buffer[VISION_HEADER_LEN..VISION_HEADER_LEN + payload_len]);
        let computed_crc = crc16_ccitt_false(&crc_input[..14 + payload_len]);
        if received_crc != computed_crc {
            return Err(VisionParseError::BadCrc);
        }

        match self.buffer[3] {
            VISION_PACKET_FRAME_SUMMARY => self.decode_frame_summary(payload_len),
            _ => Err(VisionParseError::UnknownPacket),
        }
    }

    fn decode_frame_summary(&self, payload_len: usize) -> Result<VisionPacket, VisionParseError> {
        if payload_len != VISION_FRAME_SUMMARY_PAYLOAD_LEN {
            return Err(VisionParseError::BadLength);
        }
        let payload = &self.buffer[VISION_HEADER_LEN..VISION_HEADER_LEN + payload_len];
        let mut grid = [0u16; VISION_GRID_CELLS];
        for (idx, cell) in grid.iter_mut().enumerate() {
            let offset = 18 + idx * 2;
            *cell = u16::from_le_bytes([payload[offset], payload[offset + 1]]);
        }

        Ok(VisionPacket::FrameSummary(VisionFrameSummary {
            sequence: u32::from_le_bytes([
                self.buffer[4],
                self.buffer[5],
                self.buffer[6],
                self.buffer[7],
            ]),
            captured_at_ms: u32::from_le_bytes([
                self.buffer[8],
                self.buffer[9],
                self.buffer[10],
                self.buffer[11],
            ]),
            flags: u16::from_le_bytes([payload[0], payload[1]]),
            depth_width: u16::from_le_bytes([payload[2], payload[3]]),
            depth_height: u16::from_le_bytes([payload[4], payload[5]]),
            rgb_width: u16::from_le_bytes([payload[6], payload[7]]),
            rgb_height: u16::from_le_bytes([payload[8], payload[9]]),
            depth_min_mm: u16::from_le_bytes([payload[10], payload[11]]),
            depth_max_mm: u16::from_le_bytes([payload[12], payload[13]]),
            depth_center_mm: u16::from_le_bytes([payload[14], payload[15]]),
            depth_mean_mm: u16::from_le_bytes([payload[16], payload[17]]),
            depth_grid_mm: grid,
        }))
    }
}

impl Default for VisionPacketParser {
    fn default() -> Self {
        Self::new()
    }
}

pub fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in bytes {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn append_u16(out: &mut [u8], offset: usize, value: u16) {
        out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn build_packet(summary: VisionFrameSummary) -> [u8; VISION_MAX_PACKET_LEN] {
        let mut packet = [0u8; VISION_MAX_PACKET_LEN];
        packet[0..2].copy_from_slice(&VISION_MAGIC);
        packet[2] = VISION_VERSION;
        packet[3] = VISION_PACKET_FRAME_SUMMARY;
        packet[4..8].copy_from_slice(&summary.sequence.to_le_bytes());
        packet[8..12].copy_from_slice(&summary.captured_at_ms.to_le_bytes());
        packet[12..14].copy_from_slice(&(VISION_FRAME_SUMMARY_PAYLOAD_LEN as u16).to_le_bytes());

        {
            let payload = &mut packet[VISION_HEADER_LEN..VISION_MAX_PACKET_LEN];
            append_u16(payload, 0, summary.flags);
            append_u16(payload, 2, summary.depth_width);
            append_u16(payload, 4, summary.depth_height);
            append_u16(payload, 6, summary.rgb_width);
            append_u16(payload, 8, summary.rgb_height);
            append_u16(payload, 10, summary.depth_min_mm);
            append_u16(payload, 12, summary.depth_max_mm);
            append_u16(payload, 14, summary.depth_center_mm);
            append_u16(payload, 16, summary.depth_mean_mm);
            for (idx, value) in summary.depth_grid_mm.iter().enumerate() {
                append_u16(payload, 18 + idx * 2, *value);
            }
        }

        let mut crc_input = [0u8; VISION_HEADER_LEN - 2 + VISION_MAX_PAYLOAD_LEN];
        crc_input[..14].copy_from_slice(&packet[..14]);
        crc_input[14..].copy_from_slice(&packet[VISION_HEADER_LEN..VISION_MAX_PACKET_LEN]);
        let crc = crc16_ccitt_false(&crc_input);
        packet[14..16].copy_from_slice(&crc.to_le_bytes());
        packet
    }

    #[test]
    fn parses_frame_summary_after_stream_noise() {
        let mut parser = VisionPacketParser::new();
        let mut grid = [0u16; VISION_GRID_CELLS];
        grid[5] = 1200;
        let summary = VisionFrameSummary {
            sequence: 42,
            captured_at_ms: 1000,
            flags: VISION_FLAG_DEPTH_VALID | VISION_FLAG_RGB_VALID,
            depth_width: 640,
            depth_height: 360,
            rgb_width: 640,
            rgb_height: 480,
            depth_min_mm: 300,
            depth_max_mm: 5000,
            depth_center_mm: 1100,
            depth_mean_mm: 1600,
            depth_grid_mm: grid,
        };
        for byte in [0, 1, b'O', b'X', 2] {
            assert_eq!(parser.push(byte), Ok(None));
        }

        let mut parsed = None;
        for byte in build_packet(summary) {
            if let Some(packet) = parser.push(byte).unwrap() {
                parsed = Some(packet);
            }
        }

        assert_eq!(parsed, Some(VisionPacket::FrameSummary(summary)));
    }

    #[test]
    fn rejects_crc_mismatch_and_recovers() {
        let mut parser = VisionPacketParser::new();
        let summary = VisionFrameSummary {
            sequence: 7,
            flags: VISION_FLAG_DEPTH_VALID,
            depth_width: 640,
            depth_height: 360,
            ..VisionFrameSummary::default()
        };
        let mut bad = build_packet(summary);
        bad[VISION_HEADER_LEN + 10] ^= 0x55;

        let mut err = None;
        for byte in bad {
            if let Err(parse_err) = parser.push(byte) {
                err = Some(parse_err);
            }
        }
        assert_eq!(err, Some(VisionParseError::BadCrc));

        let mut parsed = None;
        for byte in build_packet(summary) {
            if let Some(packet) = parser.push(byte).unwrap() {
                parsed = Some(packet);
            }
        }
        assert_eq!(parsed, Some(VisionPacket::FrameSummary(summary)));
    }
}
