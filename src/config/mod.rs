//! 整车集中配置。
//!
//! 按子系统拆分参数，同时在本模块统一重导出，避免调用方依赖具体文件布局。

mod can;
mod chassis;
mod gimbal;
mod remote;
mod system;

pub use can::*;
pub use chassis::*;
pub use gimbal::*;
pub use remote::*;
pub use system::*;
