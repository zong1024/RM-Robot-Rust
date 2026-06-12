//! FS-i6 + FS-A8S 的 S.BUS 解码、通道映射和整车安全门。

use crate::{
    config::{
        ARM_HOLD_MS, REMOTE_CHANNEL_LIMIT, REMOTE_DEADZONE, REMOTE_SWB_CHANNEL_INDEX,
        REMOTE_SWC_CHANNEL_INDEX, REMOTE_TIMEOUT_MS,
    },
    domain::command::{ChassisCommand, GimbalCommand, RobotCommand, WheelMode},
};

pub const FRAME_LENGTH: usize = 25;
const CHANNEL_OFFSET: i16 = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Switch {
    High = 1,
    Low = 2,
    Middle = 3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RemoteData {
    pub channels: [i16; 16],
    pub switch_b: Switch,
    pub switch_c: Switch,
    pub frame_lost: bool,
    pub failsafe: bool,
    pub frame_count: u32,
    pub last_frame_ms: u32,
}

impl RemoteData {
    pub const fn new() -> Self {
        Self {
            channels: [0; 16],
            switch_b: Switch::High,
            switch_c: Switch::High,
            frame_lost: false,
            failsafe: false,
            frame_count: 0,
            last_frame_ms: 0,
        }
    }
}

impl Default for RemoteData {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SbusDecoder {
    frame: [u8; FRAME_LENGTH],
    index: usize,
    frame_count: u32,
    error_count: u32,
    raw_byte_count: u32,
}

impl SbusDecoder {
    pub const fn new() -> Self {
        Self {
            frame: [0; FRAME_LENGTH],
            index: 0,
            frame_count: 0,
            error_count: 0,
            raw_byte_count: 0,
        }
    }

    pub fn push(&mut self, byte: u8, now_ms: u32) -> Option<RemoteData> {
        self.raw_byte_count = self.raw_byte_count.wrapping_add(1);
        if self.index == 0 {
            if byte == 0x0f {
                self.frame[0] = byte;
                self.index = 1;
            }
            return None;
        }

        self.frame[self.index] = byte;
        self.index += 1;
        if self.index != FRAME_LENGTH {
            return None;
        }
        self.index = 0;

        if self.frame[0] != 0x0f || self.frame[24] != 0 {
            self.error_count = self.error_count.wrapping_add(1);
            return None;
        }

        self.frame_count = self.frame_count.wrapping_add(1);
        let mut channels = [0i16; 16];
        for (index, value) in channels.iter_mut().enumerate() {
            *value = decode_channel(&self.frame, index);
        }

        Some(RemoteData {
            channels,
            switch_b: decode_switch(channels[REMOTE_SWB_CHANNEL_INDEX]),
            switch_c: decode_switch(channels[REMOTE_SWC_CHANNEL_INDEX]),
            frame_lost: self.frame[23] & (1 << 2) != 0,
            failsafe: self.frame[23] & (1 << 3) != 0,
            frame_count: self.frame_count,
            last_frame_ms: now_ms,
        })
    }

    pub fn note_uart_error(&mut self) {
        self.index = 0;
        self.error_count = self.error_count.wrapping_add(1);
    }

    pub fn raw_byte_count(&self) -> u32 {
        self.raw_byte_count
    }

    pub fn error_count(&self) -> u32 {
        self.error_count
    }
}

impl Default for SbusDecoder {
    fn default() -> Self {
        Self::new()
    }
}

pub struct RemoteController {
    armed: bool,
    hold_started_at: Option<u32>,
}

impl RemoteController {
    pub const fn new() -> Self {
        Self {
            armed: false,
            hold_started_at: None,
        }
    }

    pub fn update(&mut self, data: &RemoteData, now_ms: u32) -> RobotCommand {
        let online = data.frame_count != 0
            && now_ms.wrapping_sub(data.last_frame_ms) <= REMOTE_TIMEOUT_MS
            && !data.frame_lost
            && !data.failsafe;
        let primary = [
            clamp_stick(data.channels[0]),
            clamp_stick(data.channels[1]),
            clamp_stick(data.channels[2]),
            clamp_stick(data.channels[3]),
        ];
        let sticks_centered = primary.iter().all(|value| *value == 0);
        let wheel_mode = wheel_mode_from_switch(data.switch_b);

        if !online || data.switch_c == Switch::High {
            self.armed = false;
            self.hold_started_at = None;
        } else if !self.armed {
            if data.switch_c == Switch::Middle && sticks_centered {
                if let Some(start) = self.hold_started_at {
                    if now_ms.wrapping_sub(start) >= ARM_HOLD_MS {
                        self.armed = true;
                        self.hold_started_at = None;
                    }
                } else {
                    self.hold_started_at = Some(now_ms);
                }
            } else {
                self.hold_started_at = None;
            }
        }

        let enabled = online && self.armed;
        if !enabled {
            return RobotCommand {
                chassis: ChassisCommand {
                    wheel_mode,
                    ..ChassisCommand::default()
                },
                ..RobotCommand::default()
            };
        }

        let left_horizontal = normalize(primary[3]);
        RobotCommand {
            chassis: ChassisCommand {
                forward: normalize(primary[2]),
                strafe: if wheel_mode == WheelMode::Mecanum {
                    left_horizontal
                } else {
                    0.0
                },
                turn: if wheel_mode == WheelMode::Ordinary {
                    left_horizontal
                } else {
                    0.0
                },
                wheel_mode,
            },
            gimbal: GimbalCommand {
                yaw_rate: normalize(primary[0]),
                // 遥控器向上推时通常为负值，这里取反使抬头为正。
                pitch_rate: -normalize(primary[1]),
            },
            enabled: true,
        }
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    pub fn force_disarm(&mut self) {
        self.armed = false;
        self.hold_started_at = None;
    }
}

impl Default for RemoteController {
    fn default() -> Self {
        Self::new()
    }
}

fn decode_channel(frame: &[u8; FRAME_LENGTH], channel: usize) -> i16 {
    let bit_offset = channel * 11;
    let byte_offset = 1 + bit_offset / 8;
    let shift = bit_offset % 8;
    let mut packed = frame[byte_offset] as u32 | ((frame[byte_offset + 1] as u32) << 8);
    if byte_offset + 2 < 23 {
        packed |= (frame[byte_offset + 2] as u32) << 16;
    }
    ((packed >> shift) & 0x07ff) as i16 - CHANNEL_OFFSET
}

fn decode_switch(value: i16) -> Switch {
    if value < -300 {
        Switch::High
    } else if value > 300 {
        Switch::Low
    } else {
        Switch::Middle
    }
}

fn clamp_stick(value: i16) -> i16 {
    if value > REMOTE_CHANNEL_LIMIT {
        REMOTE_CHANNEL_LIMIT
    } else if value < -REMOTE_CHANNEL_LIMIT {
        -REMOTE_CHANNEL_LIMIT
    } else if value.abs() < REMOTE_DEADZONE {
        0
    } else {
        value
    }
}

fn normalize(value: i16) -> f32 {
    value as f32 / REMOTE_CHANNEL_LIMIT as f32
}

fn wheel_mode_from_switch(value: Switch) -> WheelMode {
    if value == Switch::Low {
        WheelMode::Mecanum
    } else {
        WheelMode::Ordinary
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_frame(channels: [i16; 16]) -> [u8; FRAME_LENGTH] {
        let mut frame = [0u8; FRAME_LENGTH];
        frame[0] = 0x0f;
        frame[24] = 0;
        for (channel, value) in channels.iter().enumerate() {
            let raw = (*value + CHANNEL_OFFSET) as u16 & 0x07ff;
            let bit_offset = channel * 11;
            for bit in 0..11 {
                if raw & (1 << bit) != 0 {
                    let frame_bit = bit_offset + bit;
                    frame[1 + frame_bit / 8] |= 1 << (frame_bit % 8);
                }
            }
        }
        frame
    }

    #[test]
    fn sbus从ch6解析swb并从ch5解析swc() {
        let mut channels = [0; 16];
        channels[REMOTE_SWB_CHANNEL_INDEX] = 783;
        channels[REMOTE_SWC_CHANNEL_INDEX] = -784;
        let frame = encode_frame(channels);
        let mut decoder = SbusDecoder::new();
        let mut decoded = None;
        for byte in frame {
            decoded = decoder.push(byte, 10).or(decoded);
        }
        let remote = decoded.unwrap();
        assert_eq!(remote.switch_b, Switch::Low);
        assert_eq!(remote.switch_c, Switch::High);
    }

    #[test]
    fn 左摇杆控制底盘右摇杆控制云台() {
        let mut controller = RemoteController::new();
        let mut remote = RemoteData::new();
        remote.frame_count = 1;
        remote.switch_c = Switch::Middle;
        controller.update(&remote, 0);
        remote.last_frame_ms = ARM_HOLD_MS;
        assert!(controller.update(&remote, ARM_HOLD_MS).enabled);

        remote.channels[0] = 392;
        remote.channels[1] = -392;
        remote.channels[2] = 784;
        remote.channels[3] = -784;
        remote.last_frame_ms += 1;
        let command = controller.update(&remote, ARM_HOLD_MS + 1);
        assert_eq!(command.chassis.forward, 1.0);
        assert_eq!(command.chassis.strafe, 0.0);
        assert_eq!(command.chassis.turn, -1.0);
        assert_eq!(command.chassis.wheel_mode, WheelMode::Ordinary);
        assert_eq!(command.gimbal.yaw_rate, 0.5);
        assert_eq!(command.gimbal.pitch_rate, 0.5);
    }

    #[test]
    fn swb下档切换麦克纳姆横移模式() {
        let mut controller = RemoteController::new();
        let mut remote = RemoteData::new();
        remote.frame_count = 1;
        remote.switch_b = Switch::Low;
        remote.switch_c = Switch::Middle;
        controller.update(&remote, 0);
        remote.last_frame_ms = ARM_HOLD_MS;
        assert!(controller.update(&remote, ARM_HOLD_MS).enabled);

        remote.channels[2] = 392;
        remote.channels[3] = -784;
        remote.last_frame_ms += 1;
        let command = controller.update(&remote, ARM_HOLD_MS + 1);
        assert_eq!(command.chassis.forward, 0.5);
        assert_eq!(command.chassis.strafe, -1.0);
        assert_eq!(command.chassis.turn, 0.0);
        assert_eq!(command.chassis.wheel_mode, WheelMode::Mecanum);
    }

    #[test]
    fn 锁定状态仍保留swb轮胎模式() {
        let mut controller = RemoteController::new();
        let mut remote = RemoteData::new();
        remote.frame_count = 1;
        remote.switch_b = Switch::Low;
        remote.switch_c = Switch::High;
        let command = controller.update(&remote, 0);
        assert!(!command.enabled);
        assert_eq!(command.chassis.wheel_mode, WheelMode::Mecanum);
    }

    #[test]
    fn 高档和失联都会锁车() {
        let mut controller = RemoteController::new();
        let mut remote = RemoteData::new();
        remote.frame_count = 1;
        remote.switch_c = Switch::Middle;
        controller.update(&remote, 0);
        remote.last_frame_ms = ARM_HOLD_MS;
        controller.update(&remote, ARM_HOLD_MS);
        assert!(controller.armed());

        remote.switch_c = Switch::High;
        assert!(!controller.update(&remote, ARM_HOLD_MS + 1).enabled);
        assert!(!controller.armed());
    }
}
