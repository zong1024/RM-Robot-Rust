//! 电机反馈领域模型。

use crate::config::{CONTROL_PERIOD_S, ENCODER_COUNTS_PER_REV, TWO_PI};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorFeedback {
    pub encoder: u16,
    pub speed_rpm: i16,
    pub measured_current: i16,
    pub commanded_current: i16,
    pub temperature: u8,
    pub received_at_ms: u32,
    pub frame_count: u32,
}

impl MotorFeedback {
    pub fn is_fresh(&self, now_ms: u32, timeout_ms: u32) -> bool {
        self.frame_count != 0 && now_ms.wrapping_sub(self.received_at_ms) <= timeout_ms
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EncoderTracker {
    initialized: bool,
    last_raw: u16,
    total_counts: i32,
    speed_rad_s: f32,
}

impl EncoderTracker {
    pub const fn new() -> Self {
        Self {
            initialized: false,
            last_raw: 0,
            total_counts: 0,
            speed_rad_s: 0.0,
        }
    }

    pub fn update(&mut self, raw: u16) {
        if !self.initialized {
            self.initialized = true;
            self.last_raw = raw;
            return;
        }

        let mut delta = raw as i32 - self.last_raw as i32;
        if delta > 4096 {
            delta -= 8192;
        } else if delta < -4096 {
            delta += 8192;
        }
        self.total_counts = self.total_counts.wrapping_add(delta);
        self.speed_rad_s = delta as f32 * TWO_PI / ENCODER_COUNTS_PER_REV / CONTROL_PERIOD_S;
        self.last_raw = raw;
    }

    pub fn angle_rad(&self) -> f32 {
        self.total_counts as f32 * TWO_PI / ENCODER_COUNTS_PER_REV
    }

    pub fn speed_rad_s(&self) -> f32 {
        self.speed_rad_s
    }

    pub fn initialized(&self) -> bool {
        self.initialized
    }
}

impl Default for EncoderTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 编码器跨零点时保持连续() {
        let mut tracker = EncoderTracker::new();
        tracker.update(8180);
        tracker.update(12);
        assert_eq!(tracker.total_counts, 24);
        tracker.update(8180);
        assert_eq!(tracker.total_counts, 0);
    }
}
