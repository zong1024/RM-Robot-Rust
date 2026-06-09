//! 控制周期与设备健康状态配置。

pub const CONTROL_PERIOD_S: f32 = 0.001;
pub const CONTROL_PERIOD_MS: u32 = 1;
pub const DEVICE_TIMEOUT_MS: u32 = 100;

pub const ENCODER_COUNTS_PER_REV: f32 = 8192.0;
pub const TWO_PI: f32 = core::f32::consts::PI * 2.0;
