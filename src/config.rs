//! 整车集中配置。
//!
//! 所有与机械安装、方向、限位有关的参数集中在这里，避免散落在控制代码中。

pub const CONTROL_PERIOD_S: f32 = 0.001;
pub const CONTROL_PERIOD_MS: u32 = 1;
pub const DEVICE_TIMEOUT_MS: u32 = 100;

pub const CAN_CHASSIS_COMMAND_ID: u16 = 0x200;
pub const CAN_GIMBAL_COMMAND_ID: u16 = 0x1ff;
pub const CHASSIS_FEEDBACK_IDS: [u16; 4] = [0x201, 0x202, 0x203, 0x204];
pub const YAW_6623_FEEDBACK_ID: u16 = 0x205;
pub const PITCH_6020_FEEDBACK_ID: u16 = 0x206;

/// CAN 电流帧按电机 ID 1、2、3、4 排列。
/// 实际机械位置为：左前 1、右前 2、左后 3、右后 4。
pub const CHASSIS_MOTOR_DIRECTION: [f32; 4] = [1.0, -1.0, 1.0, -1.0];
/// 当前台架只连接右前 ID2。整车四电机接齐后改为 `[true; 4]`。
pub const CHASSIS_MOTOR_ENABLED: [bool; 4] = [false, true, false, false];
pub const CHASSIS_MAX_RPM: f32 = 4500.0;
pub const CHASSIS_MAX_CURRENT: f32 = 12_000.0;

/// 轮径、减速比和轮距用于里程计，装车后应实测校准。
pub const WHEEL_RADIUS_M: f32 = 0.076;
pub const CHASSIS_TRACK_WIDTH_M: f32 = 0.42;
pub const CHASSIS_WHEELBASE_M: f32 = 0.42;
pub const M3508_GEAR_RATIO: f32 = 19.0;

pub const YAW_MAX_RATE_RAD_S: f32 = 2.5;
pub const PITCH_MAX_RATE_RAD_S: f32 = 1.8;
pub const PITCH_MIN_ANGLE_RAD: f32 = -0.45;
pub const PITCH_MAX_ANGLE_RAD: f32 = 0.45;
pub const YAW_6623_MAX_CURRENT: f32 = 5_000.0;
pub const PITCH_6020_MAX_CURRENT: f32 = 20_000.0;

pub const ENCODER_COUNTS_PER_REV: f32 = 8192.0;
pub const TWO_PI: f32 = core::f32::consts::PI * 2.0;

pub const REMOTE_CHANNEL_LIMIT: i16 = 784;
pub const REMOTE_DEADZONE: i16 = 30;
pub const REMOTE_TIMEOUT_MS: u32 = 100;
pub const ARM_HOLD_MS: u32 = 1000;
/// 本机 FS-i6 辅助通道映射：CH6 为 SwB，CH5 为 SwC。
pub const REMOTE_SWB_CHANNEL_INDEX: usize = 5;
pub const REMOTE_SWC_CHANNEL_INDEX: usize = 4;
