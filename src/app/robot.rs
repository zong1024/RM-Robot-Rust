//! 整车控制编排层。
//!
//! 本层只组合纯逻辑模块，不直接读写寄存器，因此可以在电脑上单元测试。

use crate::{
    chassis::{ChassisController, ChassisOutput},
    config::CONTROL_PERIOD_S,
    domain::{
        motor::MotorFeedback,
        remote::{RemoteController, RemoteData},
    },
    estimation::{
        attitude::Attitude,
        odometry::{ChassisOdometry, OdometryState},
    },
    gimbal::{GimbalController, GimbalOutput},
};

#[derive(Clone, Copy, Debug, Default)]
pub struct RobotSensors {
    pub remote: RemoteData,
    pub chassis: [MotorFeedback; 4],
    pub yaw_6623: MotorFeedback,
    pub pitch_6020: MotorFeedback,
    pub attitude: Attitude,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RobotOutput {
    pub chassis: ChassisOutput,
    pub gimbal: GimbalOutput,
    pub odometry: OdometryState,
    pub armed: bool,
}

pub struct RobotController {
    remote: RemoteController,
    chassis: ChassisController,
    gimbal: GimbalController,
    odometry: ChassisOdometry,
}

impl RobotController {
    pub const fn new() -> Self {
        Self {
            remote: RemoteController::new(),
            chassis: ChassisController::new(),
            gimbal: GimbalController::new(),
            odometry: ChassisOdometry::new(),
        }
    }

    pub fn update(&mut self, sensors: &RobotSensors, now_ms: u32) -> RobotOutput {
        let command = self.remote.update(&sensors.remote, now_ms);
        let chassis =
            self.chassis
                .update(command.chassis, &sensors.chassis, command.enabled, now_ms);
        let gimbal = self.gimbal.update(
            command.gimbal,
            sensors.yaw_6623,
            sensors.pitch_6020,
            command.enabled,
            now_ms,
        );
        let external_yaw = sensors.attitude.valid.then_some(sensors.attitude.yaw_rad);
        let odometry = self.odometry.update(
            &sensors.chassis,
            command.chassis.wheel_mode,
            CONTROL_PERIOD_S,
            external_yaw,
        );

        RobotOutput {
            chassis,
            gimbal,
            odometry,
            armed: command.enabled,
        }
    }
}

impl Default for RobotController {
    fn default() -> Self {
        Self::new()
    }
}
