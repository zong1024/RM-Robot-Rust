//! 6623 偏航轴与 GM6020 俯仰轴的级联控制器。

use crate::{
    config::{
        CONTROL_PERIOD_S, DEVICE_TIMEOUT_MS, PITCH_6020_MAX_CURRENT, PITCH_MAX_ANGLE_RAD,
        PITCH_MAX_RATE_RAD_S, PITCH_MIN_ANGLE_RAD, TWO_PI, YAW_6623_MAX_CURRENT,
        YAW_MAX_RATE_RAD_S,
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
}

pub struct GimbalController {
    yaw_encoder: EncoderTracker,
    pitch_encoder: EncoderTracker,
    yaw_angle_pid: Pid,
    yaw_speed_pid: Pid,
    pitch_angle_pid: Pid,
    pitch_speed_pid: Pid,
    yaw_target_rad: f32,
    pitch_target_rad: f32,
    filtered_yaw_speed: f32,
    last_yaw_frame_count: u32,
    last_pitch_frame_count: u32,
    was_enabled: bool,
}

impl GimbalController {
    pub const fn new() -> Self {
        Self {
            yaw_encoder: EncoderTracker::new(),
            pitch_encoder: EncoderTracker::new(),
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
            was_enabled: false,
        }
    }

    pub fn update(
        &mut self,
        command: GimbalCommand,
        yaw: MotorFeedback,
        pitch: MotorFeedback,
        enabled: bool,
        now_ms: u32,
    ) -> GimbalOutput {
        if yaw.frame_count != self.last_yaw_frame_count {
            self.last_yaw_frame_count = yaw.frame_count;
            self.yaw_encoder.update(yaw.encoder);
            self.filtered_yaw_speed =
                self.filtered_yaw_speed * 0.85 + self.yaw_encoder.speed_rad_s() * 0.15;
        }
        if pitch.frame_count != self.last_pitch_frame_count {
            self.last_pitch_frame_count = pitch.frame_count;
            self.pitch_encoder.update(pitch.encoder);
        }

        let yaw_angle = self.yaw_encoder.angle_rad();
        let pitch_angle = self.pitch_encoder.angle_rad();
        let online =
            yaw.is_fresh(now_ms, DEVICE_TIMEOUT_MS) && pitch.is_fresh(now_ms, DEVICE_TIMEOUT_MS);

        if !enabled || !online {
            self.reset_control(yaw_angle, pitch_angle);
            return GimbalOutput {
                yaw_target_rad: self.yaw_target_rad,
                pitch_target_rad: self.pitch_target_rad,
                yaw_angle_rad: yaw_angle,
                pitch_angle_rad: pitch_angle,
                online,
                ..GimbalOutput::default()
            };
        }

        if !self.was_enabled {
            self.yaw_target_rad = yaw_angle;
            self.pitch_target_rad = pitch_angle;
            self.was_enabled = true;
        }

        self.yaw_target_rad += command.yaw_rate * YAW_MAX_RATE_RAD_S * CONTROL_PERIOD_S;
        self.pitch_target_rad = clamp(
            self.pitch_target_rad + command.pitch_rate * PITCH_MAX_RATE_RAD_S * CONTROL_PERIOD_S,
            PITCH_MIN_ANGLE_RAD,
            PITCH_MAX_ANGLE_RAD,
        );

        let yaw_speed_target =
            self.yaw_angle_pid
                .step(self.yaw_target_rad, yaw_angle, CONTROL_PERIOD_S);
        let pitch_speed_target =
            self.pitch_angle_pid
                .step(self.pitch_target_rad, pitch_angle, CONTROL_PERIOD_S);
        let pitch_speed_rad_s = pitch.speed_rpm as f32 * TWO_PI / 60.0;

        let yaw_current =
            self.yaw_speed_pid
                .step(yaw_speed_target, self.filtered_yaw_speed, CONTROL_PERIOD_S);
        let pitch_current =
            self.pitch_speed_pid
                .step(pitch_speed_target, pitch_speed_rad_s, CONTROL_PERIOD_S);

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
        let mut controller = GimbalController::new();
        let yaw = fresh(1000);
        let pitch = fresh(1000);
        controller.update(GimbalCommand::default(), yaw, pitch, true, 10);
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
        );
        assert_eq!(output.yaw_current, 0);
        assert_eq!(output.pitch_current, 0);
    }
}
