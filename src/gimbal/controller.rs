//! 6623 偏航轴与 GM6020 俯仰轴的级联控制器。

use crate::{
    config::{
        GimbalCalibration, ENCODER_COUNTS_PER_REV, GIMBAL_CALIBRATION, MOTOR_FEEDBACK_TIMEOUT_MS,
        PITCH_6020_MAX_CURRENT, PITCH_MAX_ANGLE_RAD, PITCH_MAX_RATE_RAD_S, PITCH_MIN_ANGLE_RAD,
        TWO_PI, YAW_6623_MAX_CURRENT, YAW_MAX_RATE_RAD_S,
    },
    control::pid::{clamp, Pid},
    domain::{
        command::GimbalCommand,
        motor::{EncoderTracker, MotorFeedback},
    },
};

#[derive(Clone, Copy, Debug, Default)]
pub struct GimbalOutput {
    pub yaw_current: i16,
    pub pitch_current: i16,
    pub yaw_target_rad: f32,
    pub pitch_target_rad: f32,
    pub yaw_angle_rad: f32,
    pub pitch_angle_rad: f32,
    pub online: bool,
    pub calibrated: bool,
}

pub struct GimbalController {
    yaw_encoder: EncoderTracker,
    yaw_angle_pid: Pid,
    yaw_speed_pid: Pid,
    pitch_angle_pid: Pid,
    pitch_speed_pid: Pid,
    yaw_target_rad: f32,
    pitch_target_rad: f32,
    filtered_yaw_speed: f32,
    last_yaw_frame_count: u32,
    last_pitch_frame_count: u32,
    was_online: bool,
    was_enabled: bool,
    calibration: GimbalCalibration,
}

impl GimbalController {
    pub const fn new() -> Self {
        Self::with_calibration(GIMBAL_CALIBRATION)
    }

    pub const fn with_calibration(calibration: GimbalCalibration) -> Self {
        Self {
            yaw_encoder: EncoderTracker::new(),
            // 位置环输出角速度，速度环输出电流。
            yaw_angle_pid: Pid::new(8.0, 0.0, 0.05, 0.0, 8.0),
            yaw_speed_pid: Pid::new(800.0, 40.0, 0.0, 20.0, YAW_6623_MAX_CURRENT),
            pitch_angle_pid: Pid::new(10.0, 0.0, 0.08, 0.0, 8.0),
            pitch_speed_pid: Pid::new(1_200.0, 60.0, 0.0, 30.0, PITCH_6020_MAX_CURRENT),
            yaw_target_rad: 0.0,
            pitch_target_rad: 0.0,
            filtered_yaw_speed: 0.0,
            last_yaw_frame_count: 0,
            last_pitch_frame_count: 0,
            was_online: false,
            was_enabled: false,
            calibration,
        }
    }

    pub fn update(
        &mut self,
        command: GimbalCommand,
        yaw: MotorFeedback,
        pitch: MotorFeedback,
        enabled: bool,
        now_ms: u32,
        dt_s: f32,
    ) -> GimbalOutput {
        let online = yaw.is_fresh(now_ms, MOTOR_FEEDBACK_TIMEOUT_MS)
            && pitch.is_fresh(now_ms, MOTOR_FEEDBACK_TIMEOUT_MS);

        if online && !self.was_online {
            self.yaw_encoder
                .resynchronize_timed(yaw.encoder, yaw.received_at_ms);
            self.filtered_yaw_speed = 0.0;
            self.last_yaw_frame_count = yaw.frame_count;
            self.last_pitch_frame_count = pitch.frame_count;
            self.was_online = true;
        } else if online && yaw.frame_count != self.last_yaw_frame_count {
            self.last_yaw_frame_count = yaw.frame_count;
            self.yaw_encoder
                .update_timed(yaw.encoder, yaw.received_at_ms);
            self.filtered_yaw_speed =
                self.filtered_yaw_speed * 0.85 + self.yaw_encoder.speed_rad_s() * 0.15;
        }
        if online && pitch.frame_count != self.last_pitch_frame_count {
            self.last_pitch_frame_count = pitch.frame_count;
        }

        let yaw_angle = self.yaw_encoder.angle_rad();
        let pitch_angle = absolute_encoder_angle(
            pitch.encoder,
            self.calibration.pitch_encoder_zero,
            self.calibration.pitch_encoder_direction,
        );
        if !enabled || !online || !self.calibration.calibrated {
            if !online {
                self.was_online = false;
            }
            self.reset_control(yaw_angle, pitch_angle);
            return GimbalOutput {
                yaw_target_rad: self.yaw_target_rad,
                pitch_target_rad: self.pitch_target_rad,
                yaw_angle_rad: yaw_angle,
                pitch_angle_rad: pitch_angle,
                online,
                calibrated: self.calibration.calibrated,
                ..GimbalOutput::default()
            };
        }

        if !self.was_enabled {
            self.yaw_target_rad = yaw_angle;
            self.pitch_target_rad = pitch_angle;
            self.was_enabled = true;
        }

        self.yaw_target_rad += command.yaw_rate * YAW_MAX_RATE_RAD_S * dt_s;
        self.pitch_target_rad = clamp(
            self.pitch_target_rad + command.pitch_rate * PITCH_MAX_RATE_RAD_S * dt_s,
            PITCH_MIN_ANGLE_RAD,
            PITCH_MAX_ANGLE_RAD,
        );

        let yaw_speed_target = self
            .yaw_angle_pid
            .step(self.yaw_target_rad, yaw_angle, dt_s);
        let pitch_speed_target =
            self.pitch_angle_pid
                .step(self.pitch_target_rad, pitch_angle, dt_s);
        let pitch_speed_rad_s = pitch.speed_rpm as f32 * TWO_PI / 60.0;

        let yaw_current = self
            .yaw_speed_pid
            .step(yaw_speed_target, self.filtered_yaw_speed, dt_s);
        let pitch_current = self
            .pitch_speed_pid
            .step(pitch_speed_target, pitch_speed_rad_s, dt_s);

        GimbalOutput {
            yaw_current: clamp(yaw_current, -YAW_6623_MAX_CURRENT, YAW_6623_MAX_CURRENT) as i16,
            pitch_current: clamp(
                pitch_current,
                -PITCH_6020_MAX_CURRENT,
                PITCH_6020_MAX_CURRENT,
            ) as i16,
            yaw_target_rad: self.yaw_target_rad,
            pitch_target_rad: self.pitch_target_rad,
            yaw_angle_rad: yaw_angle,
            pitch_angle_rad: pitch_angle,
            online: true,
            calibrated: self.calibration.calibrated,
        }
    }

    fn reset_control(&mut self, yaw_angle: f32, pitch_angle: f32) {
        self.yaw_angle_pid.reset();
        self.yaw_speed_pid.reset();
        self.pitch_angle_pid.reset();
        self.pitch_speed_pid.reset();
        self.yaw_target_rad = yaw_angle;
        self.pitch_target_rad = pitch_angle;
        self.was_enabled = false;
    }
}

impl Default for GimbalController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibrated_controller() -> GimbalController {
        GimbalController::with_calibration(GimbalCalibration {
            calibrated: true,
            pitch_encoder_zero: 1000,
            pitch_encoder_direction: 1.0,
        })
    }

    fn fresh(encoder: u16) -> MotorFeedback {
        MotorFeedback {
            encoder,
            frame_count: 1,
            received_at_ms: 10,
            ..MotorFeedback::default()
        }
    }

    #[test]
    fn 俯仰目标不会越过机械限位() {
        let mut controller = calibrated_controller();
        let yaw = fresh(1000);
        let pitch = fresh(1000);
        controller.update(GimbalCommand::default(), yaw, pitch, true, 10, 0.001);
        let mut output = GimbalOutput::default();
        for _ in 0..10_000 {
            output = controller.update(
                GimbalCommand {
                    yaw_rate: 0.0,
                    pitch_rate: 1.0,
                },
                yaw,
                pitch,
                true,
                10,
                0.001,
            );
        }
        assert_eq!(output.pitch_target_rad, PITCH_MAX_ANGLE_RAD);
    }

    #[test]
    fn 离线时输出电流为零() {
        let mut controller = GimbalController::new();
        let output = controller.update(
            GimbalCommand {
                yaw_rate: 1.0,
                pitch_rate: 1.0,
            },
            MotorFeedback::default(),
            MotorFeedback::default(),
            true,
            1000,
            0.001,
        );
        assert_eq!(output.yaw_current, 0);
        assert_eq!(output.pitch_current, 0);
    }

    #[test]
    fn 失联后编码器跳变重连不会产生电流尖峰() {
        let mut controller = GimbalController::new();
        let yaw = fresh(1000);
        let pitch = fresh(1000);
        controller.update(GimbalCommand::default(), yaw, pitch, true, 10, 0.001);

        controller.update(
            GimbalCommand::default(),
            MotorFeedback::default(),
            MotorFeedback::default(),
            true,
            1000,
            0.001,
        );

        let reconnected_yaw = MotorFeedback {
            encoder: 5000,
            frame_count: 2,
            received_at_ms: 1001,
            ..MotorFeedback::default()
        };
        let reconnected_pitch = MotorFeedback {
            encoder: 3000,
            frame_count: 2,
            received_at_ms: 1001,
            ..MotorFeedback::default()
        };
        let output = controller.update(
            GimbalCommand::default(),
            reconnected_yaw,
            reconnected_pitch,
            true,
            1001,
            0.001,
        );
        assert_eq!(output.yaw_current, 0);
        assert_eq!(output.pitch_current, 0);
    }

    #[test]
    fn 未标定时云台始终锁零() {
        let mut controller = GimbalController::new();
        let mut yaw = fresh(1000);
        let mut pitch = fresh(1000);
        controller.update(GimbalCommand::default(), yaw, pitch, true, 10, 0.001);
        yaw.frame_count += 1;
        yaw.received_at_ms = 11;
        pitch.frame_count += 1;
        pitch.received_at_ms = 11;
        let output = controller.update(
            GimbalCommand {
                yaw_rate: 1.0,
                pitch_rate: 1.0,
            },
            yaw,
            pitch,
            true,
            11,
            0.001,
        );
        assert!(!output.calibrated);
        assert_eq!(output.yaw_current, 0);
        assert_eq!(output.pitch_current, 0);
    }

    #[test]
    fn 俯仰绝对角度由机械零点和方向决定() {
        assert!(
            (absolute_encoder_angle(3072, 2048, -1.0) + core::f32::consts::FRAC_PI_4).abs() < 1e-6
        );
    }
}

fn absolute_encoder_angle(raw: u16, zero: u16, direction: f32) -> f32 {
    let mut delta = raw as i32 - zero as i32;
    if delta > 4096 {
        delta -= 8192;
    } else if delta < -4096 {
        delta += 8192;
    }
    delta as f32 * TWO_PI / ENCODER_COUNTS_PER_REV * direction
}
