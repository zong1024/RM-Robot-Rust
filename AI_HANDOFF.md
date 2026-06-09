# AI 交接文档

## 工程定位

这是 `RM-Robot-Rust` 完整小车固件，不是单电机测试工程。
基础功能完成范围以 `docs/BASELINE_STATUS.md` 为准。

硬件：

- STM32F407VGT6 RoboMaster C 型开发板。
- CAN1：四个 M3508，机械位置左前 1、右前 2、左后 3、右后 4。
- CAN2：偏航 RoboMaster 6623 (`0x205`)；俯仰 GM6020 (`0x206`)。
- USART3 PC11：FS-i6 + FS-A8S S.BUS。

正式底盘要求 ID1～4 全部在线。任一反馈超时会整体锁零并清 PID/软启动状态。
功率保护参数：单路 `6000`、四路总预算 `12000`、目标转速斜坡
`2000 RPM/s`、电流斜坡 `20000/s`。
`CHASSIS_MOTOR_ONLINE_MASK` 的 bit0～bit3 对应 ID1～ID4。
四电机 CAN 排查使用 `CAN1_ID_RX_COUNT`、`CAN1_ERROR_STATUS`、
`CAN1_TX_ERROR_COUNT` 和 `CAN1_RX_ERROR_COUNT`。

## 不可混淆的协议细节

6623 与 GM6020 共用 `0x1FF` 控制帧，但反馈格式不同：

- 6623：编码器、实际转矩电流、给定转矩电流，无直接转速。
- GM6020：编码器、转速、电流、温度。
- 6623 官方电流命令范围为 `-5000..5000`。

不要把 6623 当作标准 GM6020 解析。

## 遥控映射

- 左竖 CH3：底盘前后。
- 左横 CH4：普通轮转向 / 麦克纳姆轮横移。
- 右横 CH1：云台偏航。
- 右竖 CH2：云台俯仰。
- SwB CH6：上档普通轮，下档麦克纳姆轮。
- SwC CH5：高档锁车；中档且所有主摇杆居中 1 秒解锁。

麦克纳姆模式下左横 CH4 改为横移，当前不从摇杆提供独立底盘旋转命令；
右摇杆继续完整控制云台。切换轮型时底盘控制器会清 PID 并零电流一个周期。

调试量：

- `SWB_CHANNEL_RAW`：SwB 的 CH6 原始值。
- `CHASSIS_WHEEL_MODE`：`0` 普通轮，`1` 麦克纳姆轮。

## 扩展路线

目录边界：

- 底盘专属功能放入 `src/chassis/`，两种轮型运动学位于 `kinematics.rs`。
- 云台专属功能放入 `src/gimbal/`。
- 配置按子系统放入 `src/config/`，调用方继续从 `crate::config` 导入。
- `src/control/` 只放多个子系统共享的控制算法。
- 不要把具体底盘或云台控制器重新放回 `src/control/`。

接入 IMU 时：

1. 在 `estimation/attitude.rs` 实现 `AttitudeProvider`。
2. 在 `main.rs` 构造有效的 `Attitude`。
3. 里程计会自动使用外部 yaw。
4. 云台世界系控制应在 `src/gimbal/` 新增控制模式，不要把 IMU 读取塞进
   `controller.rs`。

增加视觉或导航时，向 `RobotSensors` 增加明确的数据类型，并在 `app` 层融合。
不要让 UART/CAN 中断直接修改 PID 目标。

## 验证

```sh
make check
arm-none-eabi-size target/thumbv7em-none-eabihf/release/rm_robot
arm-none-eabi-readelf -h target/thumbv7em-none-eabihf/release/rm_robot
```

实机烧录前必须先确认六个电机均已正确接线和设置 ID。当前项目位于原 RM
仓库内部，`make build` 已处理父级 Cargo 配置叠加问题。
