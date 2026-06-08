//! 普通轮与麦克纳姆轮运动学。

use crate::domain::command::{ChassisCommand, WheelMode};

/// 输出顺序固定为左前、右前、左后、右后，对应电机 ID 1、2、3、4。
pub fn wheel_mix(command: ChassisCommand) -> [f32; 4] {
    let raw = match command.wheel_mode {
        WheelMode::Ordinary => [
            command.forward + command.turn,
            command.forward - command.turn,
            command.forward + command.turn,
            command.forward - command.turn,
        ],
        WheelMode::Mecanum => [
            command.forward + command.strafe + command.turn,
            command.forward - command.strafe - command.turn,
            command.forward - command.strafe + command.turn,
            command.forward + command.strafe - command.turn,
        ],
    };
    normalize(raw)
}

fn normalize(mut wheels: [f32; 4]) -> [f32; 4] {
    let peak = wheels
        .iter()
        .fold(1.0_f32, |maximum, value| maximum.max(value.abs()));
    for wheel in &mut wheels {
        *wheel /= peak;
    }
    wheels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn 普通轮前进与转向符合左右差速() {
        assert_eq!(
            wheel_mix(ChassisCommand {
                forward: 1.0,
                ..ChassisCommand::default()
            }),
            [1.0, 1.0, 1.0, 1.0]
        );
        assert_eq!(
            wheel_mix(ChassisCommand {
                turn: 1.0,
                ..ChassisCommand::default()
            }),
            [1.0, -1.0, 1.0, -1.0]
        );
    }

    #[test]
    fn 麦克纳姆轮可以横移() {
        assert_eq!(
            wheel_mix(ChassisCommand {
                strafe: 1.0,
                wheel_mode: WheelMode::Mecanum,
                ..ChassisCommand::default()
            }),
            [1.0, -1.0, -1.0, 1.0]
        );
    }

    #[test]
    fn 麦克纳姆混控整体归一化() {
        assert_eq!(
            wheel_mix(ChassisCommand {
                forward: 1.0,
                strafe: 1.0,
                wheel_mode: WheelMode::Mecanum,
                ..ChassisCommand::default()
            }),
            [1.0, 0.0, 0.0, 1.0]
        );
    }
}
