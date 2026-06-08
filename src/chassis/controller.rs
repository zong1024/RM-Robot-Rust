//! 四轮底盘速度控制器。

use crate::{
    chassis::kinematics::wheel_mix,
    config::{
        CHASSIS_MAX_CURRENT, CHASSIS_MAX_RPM, CHASSIS_MOTOR_DIRECTION, CONTROL_PERIOD_S,
        DEVICE_TIMEOUT_MS,
    },
    control::pid::{clamp, Pid},
    domain::{
        command::{ChassisCommand, WheelMode},
        motor::MotorFeedback,
    },
};

#[derive(Clone, Copy, Debug, Default)]
pub struct ChassisOutput {
    pub target_rpm: [f32; 4],
    pub current: [i16; 4],
    pub online: bool,
    /// bit0..bit3 分别表示 ID1..ID4 是否在线。
    pub online_mask: u8,
    pub wheel_mode: WheelMode,
}

pub struct ChassisController {
    speed_pid: [Pid; 4],
    last_wheel_mode: WheelMode,
}

impl ChassisController {
    pub const fn new() -> Self {
        const PID: Pid = Pid::new(10.0, 0.6, 0.0, 4_000.0, CHASSIS_MAX_CURRENT);
        Self {
            speed_pid: [PID; 4],
            last_wheel_mode: WheelMode::Ordinary,
        }
    }

    pub fn update(
        &mut self,
        command: ChassisCommand,
        feedback: &[MotorFeedback; 4],
        enabled: bool,
        now_ms: u32,
    ) -> ChassisOutput {
        let online_mask = feedback
            .iter()
            .enumerate()
            .fold(0u8, |mask, (index, motor)| {
                if motor.is_fresh(now_ms, DEVICE_TIMEOUT_MS) {
                    mask | (1 << index)
                } else {
                    mask
                }
            });
        let online = online_mask != 0;
        if !enabled || !online {
            self.reset();
            return ChassisOutput {
                online,
                online_mask,
                wheel_mode: command.wheel_mode,
                ..ChassisOutput::default()
            };
        }

        if command.wheel_mode != self.last_wheel_mode {
            self.reset();
            self.last_wheel_mode = command.wheel_mode;
            return ChassisOutput {
                online: true,
                online_mask,
                wheel_mode: command.wheel_mode,
                ..ChassisOutput::default()
            };
        }

        let wheel_target = wheel_mix(command);
        let mut output = ChassisOutput {
            online: true,
            online_mask,
            wheel_mode: command.wheel_mode,
            ..ChassisOutput::default()
        };

        for index in 0..4 {
            if online_mask & (1 << index) == 0 {
                self.speed_pid[index].reset();
                continue;
            }
            output.target_rpm[index] =
                wheel_target[index] * CHASSIS_MAX_RPM * CHASSIS_MOTOR_DIRECTION[index];
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 电机排列符合一二三四物理位置() {
        let wheels = wheel_mix(ChassisCommand {
            forward: 1.0,
            ..ChassisCommand::default()
        });
        let targets =
            core::array::from_fn::<_, 4, _>(|index| wheels[index] * CHASSIS_MOTOR_DIRECTION[index]);
        assert_eq!(targets, [1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn 切换轮胎模式时先输出一周期零电流() {
        let mut controller = ChassisController::new();
        let feedback = [MotorFeedback {
            frame_count: 1,
            received_at_ms: 10,
            ..MotorFeedback::default()
        }; 4];
        let output = controller.update(
            ChassisCommand {
                forward: 1.0,
                wheel_mode: WheelMode::Mecanum,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            10,
        );
        assert_eq!(output.current, [0; 4]);
        assert_eq!(output.wheel_mode, WheelMode::Mecanum);
    }

    fn fresh() -> MotorFeedback {
        MotorFeedback {
            frame_count: 1,
            received_at_ms: 10,
            ..MotorFeedback::default()
        }
    }

    #[test]
    fn 任意单台电机在线时只驱动该电机() {
        for active_index in 0..4 {
            let mut controller = ChassisController::new();
            let mut feedback = [MotorFeedback::default(); 4];
            feedback[active_index] = fresh();
            let output = controller.update(
                ChassisCommand {
                    forward: 0.5,
                    ..ChassisCommand::default()
                },
                &feedback,
                true,
                10,
            );
            assert!(output.online);
            assert_eq!(output.online_mask, 1 << active_index);
            for index in 0..4 {
                if index == active_index {
                    assert_ne!(output.current[index], 0);
                } else {
                    assert_eq!(output.current[index], 0);
                }
            }
        }
    }

    #[test]
    fn 多台在线时分别驱动且离线槽位为零() {
        let mut controller = ChassisController::new();
        let feedback = [
            fresh(),
            MotorFeedback::default(),
            fresh(),
            MotorFeedback::default(),
        ];
        let output = controller.update(
            ChassisCommand {
                forward: 0.5,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            10,
        );
        assert!(output.online);
        assert_eq!(output.online_mask, 0b0101);
        assert_ne!(output.current[0], 0);
        assert_eq!(output.current[1], 0);
        assert_ne!(output.current[2], 0);
        assert_eq!(output.current[3], 0);
    }

    #[test]
    fn 全部离线时底盘锁零() {
        let mut controller = ChassisController::new();
        let output = controller.update(
            ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            },
            &[MotorFeedback::default(); 4],
            true,
            1000,
        );
        assert!(!output.online);
        assert_eq!(output.online_mask, 0);
        assert_eq!(output.current, [0; 4]);
    }
}
