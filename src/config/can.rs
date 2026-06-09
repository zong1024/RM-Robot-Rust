//! CAN 标识符配置。

pub const CAN_CHASSIS_COMMAND_ID: u16 = 0x200;
pub const CAN_GIMBAL_COMMAND_ID: u16 = 0x1ff;
pub const CHASSIS_FEEDBACK_IDS: [u16; 4] = [0x201, 0x202, 0x203, 0x204];
pub const YAW_6623_FEEDBACK_ID: u16 = 0x205;
pub const PITCH_6020_FEEDBACK_ID: u16 = 0x206;
