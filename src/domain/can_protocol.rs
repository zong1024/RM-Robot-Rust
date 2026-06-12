//! DJI 电机 CAN 数据帧的纯编解码。

use crate::{
    config::{CHASSIS_FEEDBACK_IDS, PITCH_6020_FEEDBACK_ID, YAW_6623_FEEDBACK_ID},
    domain::motor::MotorFeedback,
};

pub fn is_expected_motor_frame(bus: u8, id: u16, extended: bool, remote: bool, dlc: u8) -> bool {
    if extended || remote || dlc != 8 {
        return false;
    }
    match bus {
        1 => CHASSIS_FEEDBACK_IDS.contains(&id),
        2 => id == YAW_6623_FEEDBACK_ID || id == PITCH_6020_FEEDBACK_ID,
        _ => false,
    }
}

pub fn decode_standard_motor(previous: MotorFeedback, data: [u8; 8], now_ms: u32) -> MotorFeedback {
    MotorFeedback {
        encoder: u16::from_be_bytes([data[0], data[1]]),
        speed_rpm: i16::from_be_bytes([data[2], data[3]]),
        measured_current: i16::from_be_bytes([data[4], data[5]]),
        commanded_current: previous.commanded_current,
        temperature: data[6],
        received_at_ms: now_ms,
        frame_count: previous.frame_count.wrapping_add(1),
    }
}

/// 6623 与 GM6020 的反馈布局不同：没有直接转速和温度字段。
pub fn decode_6623(previous: MotorFeedback, data: [u8; 8], now_ms: u32) -> MotorFeedback {
    MotorFeedback {
        encoder: u16::from_be_bytes([data[0], data[1]]),
        speed_rpm: previous.speed_rpm,
        measured_current: i16::from_be_bytes([data[2], data[3]]),
        commanded_current: i16::from_be_bytes([data[4], data[5]]),
        temperature: previous.temperature,
        received_at_ms: now_ms,
        frame_count: previous.frame_count.wrapping_add(1),
    }
}

pub fn encode_current_group(currents: [i16; 4]) -> [u8; 8] {
    let mut data = [0u8; 8];
    for (index, current) in currents.iter().enumerate() {
        let bytes = current.to_be_bytes();
        data[index * 2] = bytes[0];
        data[index * 2 + 1] = bytes[1];
    }
    data
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 正确解析标准电机反馈() {
        let motor = decode_standard_motor(
            MotorFeedback::default(),
            [0x12, 0x34, 0xff, 0x9c, 0x01, 0x2c, 55, 0],
            10,
        );
        assert_eq!(motor.encoder, 0x1234);
        assert_eq!(motor.speed_rpm, -100);
        assert_eq!(motor.measured_current, 300);
        assert_eq!(motor.temperature, 55);
    }

    #[test]
    fn 正确解析六六二三专用反馈() {
        let motor = decode_6623(
            MotorFeedback::default(),
            [0x1f, 0xff, 0xfb, 0x2e, 0x04, 0xd2, 0, 0],
            20,
        );
        assert_eq!(motor.encoder, 8191);
        assert_eq!(motor.measured_current, -1234);
        assert_eq!(motor.commanded_current, 1234);
        assert_eq!(motor.speed_rpm, 0);
    }

    #[test]
    fn 电流组使用大端序和电机编号顺序() {
        assert_eq!(
            encode_current_group([1, -2, 0x1234, -0x1234]),
            [0, 1, 0xff, 0xfe, 0x12, 0x34, 0xed, 0xcc]
        );
    }

    #[test]
    fn 仅接受目标总线上的标准八字节数据帧() {
        assert!(is_expected_motor_frame(1, 0x201, false, false, 8));
        assert!(is_expected_motor_frame(2, 0x205, false, false, 8));
        assert!(!is_expected_motor_frame(1, 0x205, false, false, 8));
        assert!(!is_expected_motor_frame(2, 0x201, false, false, 8));
        assert!(!is_expected_motor_frame(1, 0x201, true, false, 8));
        assert!(!is_expected_motor_frame(1, 0x201, false, true, 8));
        assert!(!is_expected_motor_frame(1, 0x201, false, false, 7));
    }
}
