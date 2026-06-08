//! 模块之间传递的控制命令，单位在类型定义处固定。

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum WheelMode {
    /// 普通轮：左摇杆横向控制底盘转向。
    #[default]
    Ordinary = 0,
    /// 麦克纳姆轮：左摇杆横向控制底盘横移。
    Mecanum = 1,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChassisCommand {
    /// 归一化前进速度，范围 -1 到 1。
    pub forward: f32,
    /// 归一化横移速度，向右为正，范围 -1 到 1。
    pub strafe: f32,
    /// 归一化原地转向速度，范围 -1 到 1。
    pub turn: f32,
    /// 当前安装的轮胎类型，由遥控器 SwA 选择。
    pub wheel_mode: WheelMode,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GimbalCommand {
    /// 偏航角速度，单位 rad/s。
    pub yaw_rate: f32,
    /// 俯仰角速度，单位 rad/s。
    pub pitch_rate: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RobotCommand {
    pub chassis: ChassisCommand,
    pub gimbal: GimbalCommand,
    pub enabled: bool,
}
