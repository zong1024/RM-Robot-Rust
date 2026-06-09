//! FS-i6/FS-A8S 通道、安全门和失联配置。

pub const REMOTE_CHANNEL_LIMIT: i16 = 784;
pub const REMOTE_DEADZONE: i16 = 30;
pub const REMOTE_TIMEOUT_MS: u32 = 100;
pub const ARM_HOLD_MS: u32 = 1000;

/// 本机 FS-i6 辅助通道映射：CH6 为 SwB，CH5 为 SwC。
pub const REMOTE_SWB_CHANNEL_INDEX: usize = 5;
pub const REMOTE_SWC_CHANNEL_INDEX: usize = 4;
