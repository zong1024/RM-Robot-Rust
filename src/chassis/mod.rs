//! 底盘子系统。
//!
//! 当前包含普通四轮差速混控与 M3508 速度环。后续底盘运动学、功率限制、
//! 转向模式和底盘标定代码都应继续放在本目录中。

mod controller;

pub use controller::{differential_mix, ChassisController, ChassisOutput};
