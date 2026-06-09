//! 云台速度、机械限位和电流配置。

pub const YAW_MAX_RATE_RAD_S: f32 = 2.5;
pub const PITCH_MAX_RATE_RAD_S: f32 = 1.8;
pub const PITCH_MIN_ANGLE_RAD: f32 = -0.45;
pub const PITCH_MAX_ANGLE_RAD: f32 = 0.45;
pub const YAW_6623_MAX_CURRENT: f32 = 5_000.0;
pub const PITCH_6020_MAX_CURRENT: f32 = 20_000.0;
