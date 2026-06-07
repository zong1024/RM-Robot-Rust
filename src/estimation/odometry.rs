//! 普通四轮差速底盘里程计。

use crate::{
    config::{
        CHASSIS_MOTOR_DIRECTION, CHASSIS_TRACK_WIDTH_M, M3508_GEAR_RATIO, TWO_PI, WHEEL_RADIUS_M,
    },
    domain::motor::MotorFeedback,
};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub x_m: f32,
    pub y_m: f32,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyVelocity {
    pub forward_m_s: f32,
    pub yaw_rad_s: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OdometryState {
    pub pose: Pose2,
    pub velocity: BodyVelocity,
}

pub struct DifferentialOdometry {
    state: OdometryState,
}

impl DifferentialOdometry {
    pub const fn new() -> Self {
        Self {
            state: OdometryState {
                pose: Pose2 {
                    x_m: 0.0,
                    y_m: 0.0,
                    yaw_rad: 0.0,
                },
                velocity: BodyVelocity {
                    forward_m_s: 0.0,
                    yaw_rad_s: 0.0,
                },
            },
        }
    }

    /// 当外部姿态有效时传入世界系 yaw，否则使用轮速积分的 yaw。
    pub fn update(
        &mut self,
        motors: &[MotorFeedback; 4],
        dt_s: f32,
        external_yaw_rad: Option<f32>,
    ) -> OdometryState {
        let wheel_speed = core::array::from_fn::<_, 4, _>(|index| {
            motors[index].speed_rpm as f32 * CHASSIS_MOTOR_DIRECTION[index] / M3508_GEAR_RATIO
                * TWO_PI
                / 60.0
                * WHEEL_RADIUS_M
        });
        let left = (wheel_speed[0] + wheel_speed[2]) * 0.5;
        let right = (wheel_speed[1] + wheel_speed[3]) * 0.5;
        let forward = (left + right) * 0.5;
        let yaw_rate = (right - left) / CHASSIS_TRACK_WIDTH_M;

        if let Some(yaw) = external_yaw_rad {
            self.state.pose.yaw_rad = yaw;
        } else {
            self.state.pose.yaw_rad += yaw_rate * dt_s;
        }
        let yaw = self.state.pose.yaw_rad;
        self.state.pose.x_m += forward * libm::cosf(yaw) * dt_s;
        self.state.pose.y_m += forward * libm::sinf(yaw) * dt_s;
        self.state.velocity = BodyVelocity {
            forward_m_s: forward,
            yaw_rad_s: yaw_rate,
        };
        self.state
    }

    pub fn state(&self) -> OdometryState {
        self.state
    }

    pub fn reset(&mut self, pose: Pose2) {
        self.state = OdometryState {
            pose,
            velocity: BodyVelocity::default(),
        };
    }
}

impl Default for DifferentialOdometry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn motor(rpm: i16) -> MotorFeedback {
        MotorFeedback {
            speed_rpm: rpm,
            ..MotorFeedback::default()
        }
    }

    #[test]
    fn 四轮同向物理速度产生直线里程() {
        let mut odometry = DifferentialOdometry::new();
        let motors = [motor(1000), motor(-1000), motor(1000), motor(-1000)];
        let state = odometry.update(&motors, 1.0, Some(0.0));
        assert!(state.pose.x_m > 0.0);
        assert!(state.pose.y_m.abs() < 1e-6);
        assert!(state.velocity.yaw_rad_s.abs() < 1e-6);
    }

    #[test]
    fn 左右反向产生原地旋转() {
        let mut odometry = DifferentialOdometry::new();
        let motors = [motor(-1000), motor(-1000), motor(-1000), motor(-1000)];
        let state = odometry.update(&motors, 1.0, None);
        assert!(state.pose.x_m.abs() < 1e-6);
        assert!(state.velocity.yaw_rad_s.abs() > 0.1);
    }
}
