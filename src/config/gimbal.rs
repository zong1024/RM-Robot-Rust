//! 云台速度、机械限位和电流配置。

#[derive(Clone, Copy, Debug)]
pub struct GimbalCalibration {
    pub calibrated: bool,
    pub pitch_encoder_zero: u16,
    pub pitch_encoder_direction: f32,
}

/// 完成机械零点、方向和限位标定后才能修改并将 `calibrated` 设为 `true`。
pub const GIMBAL_CALIBRATION: GimbalCalibration = GimbalCalibration {
    calibrated: false,
    pitch_encoder_zero: 0,
    pitch_encoder_direction: 1.0,
};

pub const YAW_MAX_RATE_RAD_S: f32 = 2.5;
pub const PITCH_MAX_RATE_RAD_S: f32 = 1.8;
pub const PITCH_MIN_ANGLE_RAD: f32 = -0.45;
pub const PITCH_MAX_ANGLE_RAD: f32 = 0.45;
pub const YAW_6623_MAX_CURRENT: f32 = 5_000.0;
pub const PITCH_6020_MAX_CURRENT: f32 = 20_000.0;
