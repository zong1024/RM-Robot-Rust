//! 整车控制编排层。
//!
//! 本层只组合纯逻辑模块，不直接读写寄存器，因此可以在电脑上单元测试。

use crate::{
    chassis::{ChassisController, ChassisOutput},
    config::{GimbalCalibration, CONTROL_PERIOD_MS, GIMBAL_CALIBRATION, MAX_CONTROL_GAP_MS},
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
    pub timing_fault: bool,
}

pub struct RobotController {
    remote: RemoteController,
    chassis: ChassisController,
    gimbal: GimbalController,
    odometry: ChassisOdometry,
    last_update_ms: Option<u32>,
}

impl RobotController {
    pub const fn new() -> Self {
        Self::with_gimbal_calibration(GIMBAL_CALIBRATION)
    }

    pub const fn with_gimbal_calibration(calibration: GimbalCalibration) -> Self {
        Self {
            remote: RemoteController::new(),
            chassis: ChassisController::new(),
            gimbal: GimbalController::with_calibration(calibration),
            odometry: ChassisOdometry::new(),
            last_update_ms: None,
        }
    }

    pub fn update(&mut self, sensors: &RobotSensors, now_ms: u32) -> RobotOutput {
        let previous_update_ms = self.last_update_ms;
        let elapsed_ms = previous_update_ms
            .map(|last| now_ms.wrapping_sub(last))
            .unwrap_or(CONTROL_PERIOD_MS);
        self.last_update_ms = Some(now_ms);
        let mut command = self.remote.update(&sensors.remote, now_ms);
        let timing_fault = previous_update_ms.is_some() && elapsed_ms > MAX_CONTROL_GAP_MS;
        if timing_fault {
            self.remote.force_disarm();
            command.enabled = false;
        }
        let dt_s = elapsed_ms.max(1) as f32 * 0.001;
        let chassis = self.chassis.update(
            command.chassis,
            &sensors.chassis,
            command.enabled,
            now_ms,
            dt_s,
        );
        let gimbal = self.gimbal.update(
            command.gimbal,
            sensors.yaw_6623,
            sensors.pitch_6020,
            command.enabled,
            now_ms,
            dt_s,
        );
        let external_yaw = sensors.attitude.valid.then_some(sensors.attitude.yaw_rad);
        let odometry = self.odometry.update(
            &sensors.chassis,
            command.chassis.wheel_mode,
            chassis.online && !timing_fault,
            dt_s,
            external_yaw,
        );

        RobotOutput {
            chassis,
            gimbal,
            odometry,
            armed: command.enabled,
            timing_fault,
        }
    }
}

impl Default for RobotController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        config::{GimbalCalibration, ARM_HOLD_MS, REMOTE_CHANNEL_LIMIT},
        domain::{command::WheelMode, remote::Switch},
    };

    fn motor(encoder: u16, now_ms: u32) -> MotorFeedback {
        MotorFeedback {
            encoder,
            frame_count: 1,
            received_at_ms: now_ms,
            ..MotorFeedback::default()
        }
    }

    fn ready_sensors(now_ms: u32) -> RobotSensors {
        let mut remote = RemoteData::new();
        remote.frame_count = 1;
        remote.last_frame_ms = now_ms;
        remote.switch_c = Switch::Middle;
        RobotSensors {
            remote,
            chassis: [motor(1000, now_ms); 4],
            yaw_6623: motor(1000, now_ms),
            pitch_6020: motor(1000, now_ms),
            attitude: Attitude::default(),
        }
    }

    fn arm(robot: &mut RobotController, sensors: &mut RobotSensors) {
        let mut output = RobotOutput::default();
        for now_ms in 0..=ARM_HOLD_MS {
            sensors.remote.last_frame_ms = now_ms;
            refresh_feedback(sensors, now_ms);
            output = robot.update(sensors, now_ms);
        }
        assert!(output.armed);
    }

    fn refresh_feedback(sensors: &mut RobotSensors, now_ms: u32) {
        for motor in &mut sensors.chassis {
            motor.received_at_ms = now_ms;
            motor.frame_count = motor.frame_count.wrapping_add(1);
        }
        sensors.yaw_6623.received_at_ms = now_ms;
        sensors.yaw_6623.frame_count = sensors.yaw_6623.frame_count.wrapping_add(1);
        sensors.pitch_6020.received_at_ms = now_ms;
        sensors.pitch_6020.frame_count = sensors.pitch_6020.frame_count.wrapping_add(1);
    }

    #[test]
    fn 解锁后底盘和云台命令贯穿整车编排() {
        let mut robot = calibrated_robot();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        let now = ARM_HOLD_MS + 1;
        sensors.remote.last_frame_ms = now;
        refresh_feedback(&mut sensors, now);
        sensors.remote.channels[0] = REMOTE_CHANNEL_LIMIT / 2;
        sensors.remote.channels[2] = REMOTE_CHANNEL_LIMIT;
        let output = robot.update(&sensors, now);

        assert!(output.armed);
        assert!(output.chassis.online);
        assert!(output.gimbal.online);
        assert!(output.chassis.current.iter().any(|current| *current != 0));
        assert_ne!(output.gimbal.yaw_current, 0);
    }

    #[test]
    fn 云台离线只关闭云台而不影响在线底盘() {
        let mut robot = calibrated_robot();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        let now = ARM_HOLD_MS + 1;
        sensors.remote.last_frame_ms = now;
        refresh_feedback(&mut sensors, now);
        sensors.remote.channels[2] = REMOTE_CHANNEL_LIMIT;
        sensors.yaw_6623 = MotorFeedback::default();
        sensors.pitch_6020 = MotorFeedback::default();
        let output = robot.update(&sensors, now);

        assert!(output.armed);
        assert!(output.chassis.online);
        assert!(output.chassis.current.iter().any(|current| *current != 0));
        assert!(!output.gimbal.online);
        assert_eq!(output.gimbal.yaw_current, 0);
        assert_eq!(output.gimbal.pitch_current, 0);
    }

    #[test]
    fn 底盘离线只关闭底盘而不影响在线云台() {
        let mut robot = calibrated_robot();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        let now = ARM_HOLD_MS + 1;
        sensors.remote.last_frame_ms = now;
        refresh_feedback(&mut sensors, now);
        sensors.remote.channels[0] = REMOTE_CHANNEL_LIMIT / 2;
        sensors.chassis[3] = MotorFeedback::default();
        let output = robot.update(&sensors, now);

        assert!(output.armed);
        assert!(!output.chassis.online);
        assert_eq!(output.chassis.current, [0; 4]);
        assert!(output.gimbal.online);
        assert_ne!(output.gimbal.yaw_current, 0);
        assert_eq!(output.odometry.velocity, Default::default());
    }

    #[test]
    fn 遥控failsafe同时关闭底盘和云台() {
        let mut robot = RobotController::new();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        let now = ARM_HOLD_MS + 1;
        sensors.remote.last_frame_ms = now;
        sensors.remote.failsafe = true;
        refresh_feedback(&mut sensors, now);
        let output = robot.update(&sensors, now);

        assert!(!output.armed);
        assert_eq!(output.chassis.current, [0; 4]);
        assert_eq!(output.gimbal.yaw_current, 0);
        assert_eq!(output.gimbal.pitch_current, 0);
    }

    #[test]
    fn swb模式从遥控贯穿到麦克纳姆轮速目标() {
        let mut robot = RobotController::new();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        sensors.remote.switch_b = Switch::Low;
        sensors.remote.channels[3] = REMOTE_CHANNEL_LIMIT;
        sensors.remote.last_frame_ms = ARM_HOLD_MS + 1;
        refresh_feedback(&mut sensors, ARM_HOLD_MS + 1);
        let switching = robot.update(&sensors, ARM_HOLD_MS + 1);
        assert_eq!(switching.chassis.wheel_mode, WheelMode::Mecanum);
        assert_eq!(switching.chassis.current, [0; 4]);

        sensors.remote.last_frame_ms = ARM_HOLD_MS + 2;
        refresh_feedback(&mut sensors, ARM_HOLD_MS + 2);
        let output = robot.update(&sensors, ARM_HOLD_MS + 2);
        assert_eq!(output.chassis.wheel_mode, WheelMode::Mecanum);
        assert!(output.chassis.target_rpm[0] > 0.0);
        assert!(output.chassis.target_rpm[1] > 0.0);
        assert!(output.chassis.target_rpm[2] < 0.0);
        assert!(output.chassis.target_rpm[3] < 0.0);
    }

    fn calibrated_robot() -> RobotController {
        RobotController::with_gimbal_calibration(GimbalCalibration {
            calibrated: true,
            pitch_encoder_zero: 1000,
            pitch_encoder_direction: 1.0,
        })
    }

    #[test]
    fn 控制循环严重漏拍时本周期锁零() {
        let mut robot = RobotController::new();
        let mut sensors = ready_sensors(0);
        arm(&mut robot, &mut sensors);

        let now = ARM_HOLD_MS + 20;
        sensors.remote.last_frame_ms = now;
        sensors.remote.channels[2] = REMOTE_CHANNEL_LIMIT;
        refresh_feedback(&mut sensors, now);
        let output = robot.update(&sensors, now);

        assert!(output.timing_fault);
        assert_eq!(output.chassis.current, [0; 4]);
        assert_eq!(output.gimbal.yaw_current, 0);
        assert_eq!(output.gimbal.pitch_current, 0);
    }
}
