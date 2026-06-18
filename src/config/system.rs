//! 控制周期与设备健康状态配置。

pub const CONTROL_PERIOD_S: f32 = 0.001;
pub const CONTROL_PERIOD_MS: u32 = 1;
pub const MOTOR_FEEDBACK_TIMEOUT_MS: u32 = 20;
pub const MAX_CONTROL_GAP_MS: u32 = 5;

pub const ENCODER_COUNTS_PER_REV: f32 = 8192.0;
pub const TWO_PI: f32 = core::f32::consts::PI * 2.0;
