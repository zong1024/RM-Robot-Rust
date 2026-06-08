//! 云台子系统。
//!
//! 当前包含 6623 偏航轴和 GM6020 俯仰轴的级联控制。后续世界坐标模式、
//! 自动瞄准目标管理和云台标定代码都应继续放在本目录中。

mod controller;

pub use controller::{GimbalController, GimbalOutput};
