//! 四轮底盘速度控制器。

use crate::{
    chassis::kinematics::wheel_mix,
    config::{
        CHASSIS_CURRENT_SLEW_PER_S, CHASSIS_MAX_CURRENT, CHASSIS_MAX_RPM, CHASSIS_MOTOR_DIRECTION,
        CHASSIS_TARGET_RPM_SLEW_PER_S, CHASSIS_TOTAL_CURRENT_LIMIT, MOTOR_FEEDBACK_TIMEOUT_MS,
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
    ramped_target_rpm: [f32; 4],
    ramped_current: [f32; 4],
    last_wheel_mode: WheelMode,
}

impl ChassisController {
    pub const fn new() -> Self {
        const PID: Pid = Pid::new(10.0, 0.6, 0.0, 4_000.0, CHASSIS_MAX_CURRENT);
        Self {
            speed_pid: [PID; 4],
            ramped_target_rpm: [0.0; 4],
            ramped_current: [0.0; 4],
            last_wheel_mode: WheelMode::Ordinary,
        }
    }

    pub fn update(
        &mut self,
        command: ChassisCommand,
        feedback: &[MotorFeedback; 4],
        enabled: bool,
        now_ms: u32,
        dt_s: f32,
    ) -> ChassisOutput {
        let online_mask = feedback
            .iter()
            .enumerate()
            .fold(0u8, |mask, (index, motor)| {
                if motor.is_fresh(now_ms, MOTOR_FEEDBACK_TIMEOUT_MS) {
                    mask | (1 << index)
                } else {
                    mask
                }
            });
        let online = online_mask == 0b1111;
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
            let requested_target =
                wheel_target[index] * CHASSIS_MAX_RPM * CHASSIS_MOTOR_DIRECTION[index];
            self.ramped_target_rpm[index] = slew(
                self.ramped_target_rpm[index],
                requested_target,
                CHASSIS_TARGET_RPM_SLEW_PER_S * dt_s,
            );
            output.target_rpm[index] = self.ramped_target_rpm[index];
            let requested_current = self.speed_pid[index].step(
                self.ramped_target_rpm[index],
                feedback[index].speed_rpm as f32,
                dt_s,
            );
            self.ramped_current[index] = slew(
                self.ramped_current[index],
                clamp(requested_current, -CHASSIS_MAX_CURRENT, CHASSIS_MAX_CURRENT),
                CHASSIS_CURRENT_SLEW_PER_S * dt_s,
            );
        }
        apply_total_current_limit(&mut self.ramped_current);
        output.current = core::array::from_fn(|index| self.ramped_current[index] as i16);
        output
    }

    pub fn reset(&mut self) {
        for pid in &mut self.speed_pid {
            pid.reset();
        }
        self.ramped_target_rpm = [0.0; 4];
        self.ramped_current = [0.0; 4];
    }
}

impl Default for ChassisController {
    fn default() -> Self {
        Self::new()
    }
}

fn slew(current: f32, target: f32, max_step: f32) -> f32 {
    current + clamp(target - current, -max_step, max_step)
}

fn apply_total_current_limit(currents: &mut [f32; 4]) {
    let total = currents.iter().map(|current| current.abs()).sum::<f32>();
    if total > CHASSIS_TOTAL_CURRENT_LIMIT {
        let scale = CHASSIS_TOTAL_CURRENT_LIMIT / total;
        for current in currents {
            *current *= scale;
        }
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
            0.001,
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
    fn 正式模式要求四台电机全部在线() {
        let mut controller = ChassisController::new();
        let mut feedback = [fresh(); 4];
        feedback[3] = MotorFeedback::default();
        let output = controller.update(
            ChassisCommand {
                forward: 0.5,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            10,
            0.001,
        );
        assert!(!output.online);
        assert_eq!(output.online_mask, 0b0111);
        assert_eq!(output.current, [0; 4]);
    }

    #[test]
    fn 电机反馈超过二十毫秒立即离线() {
        let mut controller = ChassisController::new();
        let output = controller.update(
            ChassisCommand {
                forward: 0.5,
                ..ChassisCommand::default()
            },
            &[fresh(); 4],
            true,
            31,
            0.001,
        );
        assert!(!output.online);
        assert_eq!(output.current, [0; 4]);
    }

    #[test]
    fn 四台在线时目标和电流逐周期缓升() {
        let mut controller = ChassisController::new();
        let feedback = [fresh(); 4];
        let first = controller.update(
            ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            10,
            0.001,
        );
        let second = controller.update(
            ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            11,
            0.001,
        );
        assert!(first.online);
        assert_eq!(first.online_mask, 0b1111);
        assert_eq!(first.target_rpm, [2.0, -2.0, 2.0, -2.0]);
        assert_eq!(first.current, [20, -20, 20, -20]);
        assert_eq!(second.target_rpm, [4.0, -4.0, 4.0, -4.0]);
        assert_eq!(second.current, [40, -40, 40, -40]);
    }

    #[test]
    fn 总电流预算按比例限制四路输出() {
        let mut currents = [6_000.0; 4];
        apply_total_current_limit(&mut currents);
        assert_eq!(currents, [3_000.0; 4]);
        assert_eq!(
            currents.iter().map(|current| current.abs()).sum::<f32>(),
            CHASSIS_TOTAL_CURRENT_LIMIT
        );
    }

    #[test]
    fn 掉线后清零斜坡和pid() {
        let mut controller = ChassisController::new();
        let feedback = [fresh(); 4];
        controller.update(
            ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            },
            &feedback,
            true,
            10,
            0.001,
        );
        let offline = controller.update(
            ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            },
            &[MotorFeedback::default(); 4],
            true,
            1000,
            0.001,
        );
        assert_eq!(offline.current, [0; 4]);
        assert_eq!(controller.ramped_target_rpm, [0.0; 4]);
        assert_eq!(controller.ramped_current, [0.0; 4]);
    }
}
