# RM 完整小车 Rust 固件

面向 RoboMaster C 型开发板（STM32F407VGT6）的全 Rust `no_std` 整车框架。
工程包含普通轮/麦克纳姆轮可切换底盘、双轴云台、FS-i6/FS-A8S 遥控、
安全门和双模式里程计，并为 IMU、世界坐标系和后续状态估计预留稳定接口。

## 当前硬件定义

### 底盘

底盘使用四个 M3508 + C620，CAN1 速率 1 Mbps，可安装普通轮或麦克纳姆轮：

| 机械位置 | 电机 ID | 反馈 ID | 控制帧位置 |
| --- | ---: | ---: | ---: |
| 左前 | 1 | `0x201` | `DATA[0..1]` |
| 右前 | 2 | `0x202` | `DATA[2..3]` |
| 左后 | 3 | `0x203` | `DATA[4..5]` |
| 右后 | 4 | `0x204` | `DATA[6..7]` |

四个电机由 `0x200` 群发控制。左右电机机械安装方向相反，默认方向配置为
`[+1, -1, +1, -1]`，位于 `src/config/chassis.rs`。

正式整车模式要求 ID1～4 四台 M3508 全部在线才允许底盘动作。
`CHASSIS_MOTOR_ONLINE_MASK` 的 bit0～bit3 分别表示 ID1～ID4 在线状态。

为避免四台同时启动造成电源母线压降，底盘包含三层功率保护：

- 目标转速按 `2000 RPM/s` 斜坡变化。
- 每路电流命令按 `20000/s` 斜坡变化，单路限幅 `6000`。
- 四路绝对电流之和限幅 `12000`，超限时按比例缩放。

轮胎模式由 FS-i6 的两档 `SwB` 选择：

| SwB 档位 | 模式 | 左摇杆竖直 | 左摇杆水平 |
| --- | --- | --- | --- |
| 上档 | 普通轮 | 前后 | 转向 |
| 下档 | 麦克纳姆轮 | 前后 | 横移 |

SwB 映射到 CH6（`channels[5]`）。模式切换时底盘会清空速度 PID，并输出
一个控制周期的零电流。麦克纳姆轮按 X 型辊子方向安装；若横移方向错误，应先检查
轮子安装位置，再检查 `src/chassis/kinematics.rs` 的横移符号。

### 云台

云台使用 CAN2，速率 1 Mbps：

| 轴 | 电机 | 反馈 ID | 控制 ID | 电流限幅 |
| --- | --- | ---: | ---: | ---: |
| 偏航 | RoboMaster 6623 | `0x205` | `0x1FF` | `±5000` |
| 俯仰 | GM6020 | `0x206` | `0x1FF` | `±20000` |

6623 的反馈格式与 GM6020 不同：它不直接返回转速，框架使用 8192 线绝对编码器
差分并低通滤波估算偏航速度。协议实现位于
`src/domain/can_protocol.rs`。

### 遥控器

沿用已经实机验证的 FS-i6 + FS-A8S S.BUS：

- USART3 RX：PC11
- 100000 baud，8E2（STM32 配置为 9 位字长、偶校验、2 停止位）
- 左摇杆竖直 CH3：底盘前后
- 左摇杆水平 CH4：普通轮转向 / 麦克纳姆轮横移
- 右摇杆水平 CH1：云台偏航
- 右摇杆竖直 CH2：云台俯仰
- 两档 SwB CH6：上档普通轮，下档麦克纳姆轮
- 三档 SwC CH5：高档立即锁车；中档且四个主摇杆居中 1 秒后解锁

遥控失联、S.BUS failsafe 或帧超时 100 ms 会立即让全部控制电流归零。
底盘或云台电机反馈超过 20 ms 未更新也会使对应模块归零并清空 PID。
控制循环间隔超过 5 ms 时会锁零并要求重新解锁；主循环卡死约 500 ms 后由独立看门狗复位。

云台采用默认失效安全配置。首次装车必须在 `src/config/gimbal.rs` 填写俯仰机械零点和方向，
架空验证无误后再将 `GIMBAL_CALIBRATION.calibrated` 改为 `true`。未标定时云台始终零电流，
底盘仍可独立工作。

## 软件架构

```text
src/
├── chassis/       底盘子系统：双模式运动学与四路 M3508 速度环
├── config/        CAN、底盘、云台、遥控和系统周期配置
├── gimbal/        云台子系统：6623/GM6020 级联控制
├── control/       底盘与云台共享的 PID 等基础算法
├── domain/        电机反馈、CAN 协议、遥控数据、模块间命令
├── estimation/    姿态接口、双模式里程计、世界坐标位姿
├── app/           整车 1 kHz 编排，不直接访问硬件
├── platform/      STM32 寄存器、CAN 中断、S.BUS 中断
└── main.rs        时钟、引脚、中断入口、RGB 和周期调度
```

控制路径不使用堆分配。中断只收发定长数据，所有 PID 和状态估计在固定 1 kHz
主循环执行，便于测量最坏执行时间，也便于后续迁移到 RTIC 或 Embassy。

底盘和云台是顶层独立子系统。以后增加底盘运动学、功率限制时放入
`src/chassis/`；增加云台世界坐标模式、自动瞄准和标定时放入
`src/gimbal/`。只有确实被多个子系统复用的算法才放入 `src/control/`。

详见 [架构说明](docs/ARCHITECTURE.md) 与
[接线和调参](docs/HARDWARE_AND_TUNING.md)。基础功能完成范围、装车标定项和
后续扩展边界见 [基础框架完成状态](docs/BASELINE_STATUS.md)。

## 构建

```sh
rustup target add thumbv7em-none-eabihf
make check
```

烧录：

```sh
make flash
```

`make build` 特意从 `/tmp` 启动 Cargo，避免本仓库位于另一个 Rust 固件目录中时，
父目录的 Cargo 配置被重复合并。独立 clone 后同样可用。

GitHub Actions 会对每次推送执行格式检查、主机单元测试、Clippy 严格检查和
Cortex-M4F 发布构建。

## RGB 状态

- 绿灯：500 ms 心跳，表示主循环运行。
- 红灯：底盘或云台任一必需电机离线。
- 蓝灯：遥控安全门已解锁。

RGB 为低电平点亮：PH12 红、PH11 绿、PH10 蓝。

通过 SWD 可查看 `SWB_CHANNEL_RAW` 和 `CHASSIS_WHEEL_MODE`：后者为 `0` 时是
普通轮模式，为 `1` 时是麦克纳姆轮模式。

安全诊断量包括 `CAN_INVALID_FRAME_COUNT`、`CAN_RX_BUDGET_EXHAUSTED_COUNT`、
`CONTROL_TIMING_FAULT_COUNT` 和 `GIMBAL_CALIBRATED`。

## 首次装车必须确认

1. 抬起车轮和云台，先在无负载状态测试。
2. 检查四个底盘电机 ID 与机械位置。
3. 检查 `CHASSIS_MOTOR_DIRECTION`，保证正向命令时四轮物理方向一致。
4. 检查 6623 拨码为 Yaw（反馈 `0x205`），GM6020 为 ID 2（反馈 `0x206`）。
5. 实测轮半径、轮距和减速比后更新 `src/config/chassis.rs`。
6. 填写俯仰编码器机械零点和方向，架空确认后启用 `GIMBAL_CALIBRATION`。
7. 重新标定底盘和云台 PID；仓库内参数只是保守起点。
8. 根据实际机械限位修改俯仰最小/最大角。

## 参考资料

- [DJI RoboMaster 6623 电调说明书](https://rm-static.djicdn.com/tem/1f53d24dad94e151687436607599258.pdf)
- [DJI RoboMaster 6623 电机产品页](https://www.robomaster.com/zh-CN/products/components/detail/131)
- 本机 `Development-Board-C-Examples/20.standard_robot` 官方示例
