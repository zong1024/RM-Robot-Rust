pub const VISION_MAGIC: [u8; 2] = *b"OB";
pub const VISION_VERSION: u8 = 1;
pub const VISION_PACKET_FRAME_SUMMARY: u8 = 1;
pub const VISION_HEADER_LEN: usize = 16;
pub const VISION_GRID_CELLS: usize = 16;
pub const VISION_FRAME_SUMMARY_PAYLOAD_LEN: usize = 50;
pub const VISION_FRAME_SUMMARY_PACKET_LEN: usize =
    VISION_HEADER_LEN + VISION_FRAME_SUMMARY_PAYLOAD_LEN;
pub const VISION_FLAG_DEPTH_VALID: u16 = 1 << 0;
pub const VISION_FLAG_RGB_VALID: u16 = 1 << 1;

#[derive(Clone, Copy, Debug, Default)]
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
    pub depth_grid_mm: [u16; VISION_GRID_CELLS],
}

impl VisionFrameSummary {
    pub fn encode(self) -> [u8; VISION_FRAME_SUMMARY_PACKET_LEN] {
        let mut packet = [0u8; VISION_FRAME_SUMMARY_PACKET_LEN];
        packet[0..2].copy_from_slice(&VISION_MAGIC);
        packet[2] = VISION_VERSION;
        packet[3] = VISION_PACKET_FRAME_SUMMARY;
        packet[4..8].copy_from_slice(&self.sequence.to_le_bytes());
        packet[8..12].copy_from_slice(&self.captured_at_ms.to_le_bytes());
        packet[12..14].copy_from_slice(&(VISION_FRAME_SUMMARY_PAYLOAD_LEN as u16).to_le_bytes());

        {
            let payload = &mut packet[VISION_HEADER_LEN..VISION_FRAME_SUMMARY_PACKET_LEN];
            put_u16(payload, 0, self.flags);
            put_u16(payload, 2, self.depth_width);
            put_u16(payload, 4, self.depth_height);
            put_u16(payload, 6, self.rgb_width);
            put_u16(payload, 8, self.rgb_height);
            put_u16(payload, 10, self.depth_min_mm);
            put_u16(payload, 12, self.depth_max_mm);
            put_u16(payload, 14, self.depth_center_mm);
            put_u16(payload, 16, self.depth_mean_mm);
            for (idx, value) in self.depth_grid_mm.iter().enumerate() {
                put_u16(payload, 18 + idx * 2, *value);
            }
        }

        let mut crc_input = [0u8; VISION_HEADER_LEN - 2 + VISION_FRAME_SUMMARY_PAYLOAD_LEN];
        crc_input[..14].copy_from_slice(&packet[..14]);
        crc_input[14..]
            .copy_from_slice(&packet[VISION_HEADER_LEN..VISION_FRAME_SUMMARY_PACKET_LEN]);
        let crc = crc16_ccitt_false(&crc_input);
        packet[14..16].copy_from_slice(&crc.to_le_bytes());
        packet
    }
}

fn put_u16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
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

    #[test]
    fn frame_summary_packet_matches_wire_layout() {
        let mut grid = [0u16; VISION_GRID_CELLS];
        for (idx, value) in grid.iter_mut().enumerate() {
            *value = 1000 + idx as u16;
        }

        let summary = VisionFrameSummary {
            sequence: 0x1122_3344,
            captured_at_ms: 0xaabb_ccdd,
            flags: VISION_FLAG_DEPTH_VALID | VISION_FLAG_RGB_VALID,
            depth_width: 640,
            depth_height: 360,
            rgb_width: 640,
            rgb_height: 480,
            depth_min_mm: 300,
            depth_max_mm: 5000,
            depth_center_mm: 1200,
            depth_mean_mm: 1600,
            depth_grid_mm: grid,
        };

        let packet = summary.encode();
        assert_eq!(packet.len(), VISION_FRAME_SUMMARY_PACKET_LEN);
        assert_eq!(&packet[0..2], &VISION_MAGIC);
        assert_eq!(packet[2], VISION_VERSION);
        assert_eq!(packet[3], VISION_PACKET_FRAME_SUMMARY);
        assert_eq!(
            u32::from_le_bytes(packet[4..8].try_into().unwrap()),
            summary.sequence
        );
        assert_eq!(
            u32::from_le_bytes(packet[8..12].try_into().unwrap()),
            summary.captured_at_ms
        );
        assert_eq!(
            u16::from_le_bytes(packet[12..14].try_into().unwrap()),
            VISION_FRAME_SUMMARY_PAYLOAD_LEN as u16
        );
        assert_eq!(
            u16::from_le_bytes(packet[16..18].try_into().unwrap()),
            summary.flags
        );
        assert_eq!(
            u16::from_le_bytes(packet[18..20].try_into().unwrap()),
            summary.depth_width
        );
        assert_eq!(
            u16::from_le_bytes(packet[64..66].try_into().unwrap()),
            summary.depth_grid_mm[15]
        );

        let received_crc = u16::from_le_bytes(packet[14..16].try_into().unwrap());
        let mut crc_input = [0u8; VISION_HEADER_LEN - 2 + VISION_FRAME_SUMMARY_PAYLOAD_LEN];
        crc_input[..14].copy_from_slice(&packet[..14]);
        crc_input[14..].copy_from_slice(&packet[VISION_HEADER_LEN..]);
        assert_eq!(received_crc, crc16_ccitt_false(&crc_input));
    }
}
