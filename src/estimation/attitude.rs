//! 姿态估计抽象层。
//!
//! 当前工程不绑定具体 IMU。后续可实现 BMI088、ICM42688 或外部导航模块，
//! 再通过 `AttitudeProvider` 接入云台世界坐标系控制和里程计融合。

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Attitude {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
    pub gyro_rad_s: [f32; 3],
    pub timestamp_ms: u32,
    pub valid: bool,
}

pub trait AttitudeProvider {
    fn attitude(&self) -> Attitude;
}

#[derive(Default)]
pub struct NoAttitude;

impl AttitudeProvider for NoAttitude {
    fn attitude(&self) -> Attitude {
        Attitude::default()
    }
}
