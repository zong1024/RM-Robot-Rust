//! 底盘机械参数、限幅和功率保护配置。

/// CAN 电流帧按电机 ID 1、2、3、4 排列。
/// 实际机械位置为：左前 1、右前 2、左后 3、右后 4。
pub const CHASSIS_MOTOR_DIRECTION: [f32; 4] = [1.0, -1.0, 1.0, -1.0];
pub const CHASSIS_MAX_RPM: f32 = 4500.0;

/// 正式整车采用保守功率参数，装车测量母线电压和电流后再逐步提高。
pub const CHASSIS_MAX_CURRENT: f32 = 6_000.0;
pub const CHASSIS_TOTAL_CURRENT_LIMIT: f32 = 12_000.0;
pub const CHASSIS_TARGET_RPM_SLEW_PER_S: f32 = 2_000.0;
pub const CHASSIS_CURRENT_SLEW_PER_S: f32 = 20_000.0;

/// 轮径、减速比、轮距和轴距用于里程计，装车后应实测校准。
pub const WHEEL_RADIUS_M: f32 = 0.076;
pub const CHASSIS_TRACK_WIDTH_M: f32 = 0.42;
pub const CHASSIS_WHEELBASE_M: f32 = 0.42;
pub const M3508_GEAR_RATIO: f32 = 19.0;
