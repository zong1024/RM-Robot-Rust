//! 普通四轮差速底盘控制器。

use crate::{
    config::{
        CHASSIS_MAX_CURRENT, CHASSIS_MAX_RPM, CHASSIS_MOTOR_DIRECTION, CONTROL_PERIOD_S,
        DEVICE_TIMEOUT_MS,
    },
    control::pid::{clamp, Pid},
    domain::{command::ChassisCommand, motor::MotorFeedback},
};

#[derive(Clone, Copy, Debug, Default)]
pub struct ChassisOutput {
    pub target_rpm: [f32; 4],
    pub current: [i16; 4],
    pub online: bool,
}

pub struct ChassisController {
    speed_pid: [Pid; 4],
}

impl ChassisController {
    pub const fn new() -> Self {
        const PID: Pid = Pid::new(10.0, 0.6, 0.0, 4_000.0, CHASSIS_MAX_CURRENT);
        Self {
            speed_pid: [PID; 4],
        }
    }

    pub fn update(
        &mut self,
        command: ChassisCommand,
        feedback: &[MotorFeedback; 4],
        enabled: bool,
        now_ms: u32,
    ) -> ChassisOutput {
        let online = feedback
            .iter()
            .all(|motor| motor.is_fresh(now_ms, DEVICE_TIMEOUT_MS));
        if !enabled || !online {
            self.reset();
            return ChassisOutput {
                online,
                ..ChassisOutput::default()
            };
        }

        let (left, right) = differential_mix(command.forward, command.turn);
        let side_target = [left, right, left, right];
        let mut output = ChassisOutput {
            online: true,
            ..ChassisOutput::default()
        };

        for index in 0..4 {
            output.target_rpm[index] =
                side_target[index] * CHASSIS_MAX_RPM * CHASSIS_MOTOR_DIRECTION[index];
            let current = self.speed_pid[index].step(
                output.target_rpm[index],
                feedback[index].speed_rpm as f32,
                CONTROL_PERIOD_S,
            );
            output.current[index] =
                clamp(current, -CHASSIS_MAX_CURRENT, CHASSIS_MAX_CURRENT) as i16;
        }
        output
    }

    pub fn reset(&mut self) {
        for pid in &mut self.speed_pid {
            pid.reset();
        }
    }
}

impl Default for ChassisController {
    fn default() -> Self {
        Self::new()
    }
}

pub fn differential_mix(forward: f32, turn: f32) -> (f32, f32) {
    let mut left = forward + turn;
    let mut right = forward - turn;
    let peak = left.abs().max(right.abs()).max(1.0);
    left /= peak;
    right /= peak;
    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 电机排列符合一二三四物理位置() {
        let (left, right) = differential_mix(1.0, 0.0);
        let side = [left, right, left, right];
        let targets =
            core::array::from_fn::<_, 4, _>(|index| side[index] * CHASSIS_MOTOR_DIRECTION[index]);
        assert_eq!(targets, [1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn 混控会整体归一化而不是单边截断() {
        let (left, right) = differential_mix(1.0, 1.0);
        assert_eq!(left, 1.0);
        assert_eq!(right, 0.0);
    }
}
