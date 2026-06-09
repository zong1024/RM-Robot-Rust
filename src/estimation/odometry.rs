//! 普通轮与麦克纳姆轮底盘里程计。

use crate::{
    config::{
        CHASSIS_MOTOR_DIRECTION, CHASSIS_TRACK_WIDTH_M, CHASSIS_WHEELBASE_M, M3508_GEAR_RATIO,
        TWO_PI, WHEEL_RADIUS_M,
    },
    domain::{command::WheelMode, motor::MotorFeedback},
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
    /// 车体向右为正。
    pub lateral_m_s: f32,
    pub yaw_rad_s: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OdometryState {
    pub pose: Pose2,
    pub velocity: BodyVelocity,
}

pub struct ChassisOdometry {
    state: OdometryState,
}

impl ChassisOdometry {
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
                    lateral_m_s: 0.0,
                    yaw_rad_s: 0.0,
                },
            },
        }
    }

    /// 当外部姿态有效时传入世界系 yaw，否则使用轮速积分的 yaw。
    pub fn update(
        &mut self,
        motors: &[MotorFeedback; 4],
        wheel_mode: WheelMode,
        feedback_valid: bool,
        dt_s: f32,
        external_yaw_rad: Option<f32>,
    ) -> OdometryState {
        if !feedback_valid {
            if let Some(yaw) = external_yaw_rad {
                self.state.pose.yaw_rad = yaw;
            }
            self.state.velocity = BodyVelocity::default();
            return self.state;
        }

        let wheel_speed = core::array::from_fn::<_, 4, _>(|index| {
            motors[index].speed_rpm as f32 * CHASSIS_MOTOR_DIRECTION[index] / M3508_GEAR_RATIO
                * TWO_PI
                / 60.0
                * WHEEL_RADIUS_M
        });
        let (forward, lateral, yaw_rate) = match wheel_mode {
            WheelMode::Ordinary => {
                let left = (wheel_speed[0] + wheel_speed[2]) * 0.5;
                let right = (wheel_speed[1] + wheel_speed[3]) * 0.5;
                (
                    (left + right) * 0.5,
                    0.0,
                    (right - left) / CHASSIS_TRACK_WIDTH_M,
                )
            }
            WheelMode::Mecanum => {
                let [left_front, right_front, left_rear, right_rear] = wheel_speed;
                let forward = (left_front + right_front + left_rear + right_rear) * 0.25;
                let lateral = (left_front - right_front - left_rear + right_rear) * 0.25;
                let rotation_linear = (left_front - right_front + left_rear - right_rear) * 0.25;
                let rotation_radius = (CHASSIS_TRACK_WIDTH_M + CHASSIS_WHEELBASE_M) * 0.5;
                (forward, lateral, -rotation_linear / rotation_radius)
            }
        };

        if let Some(yaw) = external_yaw_rad {
            self.state.pose.yaw_rad = yaw;
        } else {
            self.state.pose.yaw_rad += yaw_rate * dt_s;
        }
        let yaw = self.state.pose.yaw_rad;
        self.state.pose.x_m += (forward * libm::cosf(yaw) + lateral * libm::sinf(yaw)) * dt_s;
        self.state.pose.y_m += (forward * libm::sinf(yaw) - lateral * libm::cosf(yaw)) * dt_s;
        self.state.velocity = BodyVelocity {
            forward_m_s: forward,
            lateral_m_s: lateral,
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

impl Default for ChassisOdometry {
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
        let mut odometry = ChassisOdometry::new();
        let motors = [motor(1000), motor(-1000), motor(1000), motor(-1000)];
        let state = odometry.update(&motors, WheelMode::Ordinary, true, 1.0, Some(0.0));
        assert!(state.pose.x_m > 0.0);
        assert!(state.pose.y_m.abs() < 1e-6);
        assert!(state.velocity.lateral_m_s.abs() < 1e-6);
        assert!(state.velocity.yaw_rad_s.abs() < 1e-6);
    }

    #[test]
    fn 左右反向产生原地旋转() {
        let mut odometry = ChassisOdometry::new();
        let motors = [motor(-1000), motor(-1000), motor(-1000), motor(-1000)];
        let state = odometry.update(&motors, WheelMode::Ordinary, true, 1.0, None);
        assert!(state.pose.x_m.abs() < 1e-6);
        assert!(state.velocity.yaw_rad_s.abs() > 0.1);
    }

    #[test]
    fn 麦克纳姆轮横移会产生侧向里程() {
        let mut odometry = ChassisOdometry::new();
        let motors = [motor(1000), motor(1000), motor(-1000), motor(-1000)];
        let state = odometry.update(&motors, WheelMode::Mecanum, true, 1.0, Some(0.0));
        assert!(state.velocity.forward_m_s.abs() < 1e-6);
        assert!(state.velocity.lateral_m_s > 0.0);
        assert!(state.pose.y_m < 0.0);
    }

    #[test]
    fn 底盘反馈失联时停止积分但接受外部航向() {
        let mut odometry = ChassisOdometry::new();
        let motors = [motor(1000), motor(-1000), motor(1000), motor(-1000)];
        let moving = odometry.update(&motors, WheelMode::Ordinary, true, 1.0, Some(0.0));
        let stopped = odometry.update(&motors, WheelMode::Ordinary, false, 1.0, Some(1.0));
        assert_eq!(stopped.pose.x_m, moving.pose.x_m);
        assert_eq!(stopped.pose.y_m, moving.pose.y_m);
        assert_eq!(stopped.pose.yaw_rad, 1.0);
        assert_eq!(stopped.velocity, BodyVelocity::default());
    }
}
