//! 模块之间传递的控制命令，单位在类型定义处固定。

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ChassisCommand {
    /// 归一化前进速度，范围 -1 到 1。
    pub forward: f32,
    /// 归一化原地转向速度，范围 -1 到 1。
    pub turn: f32,
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
