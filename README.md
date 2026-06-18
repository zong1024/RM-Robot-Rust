# RM Robot Rust

[![Rust 固件持续集成](https://github.com/zong1024/RM-Robot-Rust/actions/workflows/ci.yml/badge.svg)](https://github.com/zong1024/RM-Robot-Rust/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

面向 RoboMaster C 型开发板（STM32F407VGT6）和 Linux SBC 的整车控制框架。

项目提供一套可测试、无堆分配的机器人控制框架，覆盖四轮底盘、双轴云台、
FS-i6/FS-A8S 遥控、安全门与里程计，并为 IMU、Linux SBC 视觉链路、
世界坐标控制和状态估计保留清晰的扩展边界。

当前架构按“大脑/小脑”拆分：

- 香橙派 AI Pro 8T 作为“大脑”，接入 Orbbec DaBai DCW 深度相机，后续负责感知、
  自动驾驶和高层决策。
- STM32F407 C 板作为“小脑”，负责 1 kHz 底盘/云台实时控制、遥控安全门和电机输出。

## 核心能力

- 普通轮差速与麦克纳姆轮 X 型运动学，可通过遥控器切换。
- 四台 M3508 独立速度环，以及目标转速、电流斜坡和总电流预算。
- RoboMaster 6623 偏航轴与 GM6020 俯仰轴级联控制。
- FS-i6 + FS-A8S S.BUS 解码、摇杆映射和长按解锁状态机。
- 普通轮/麦克纳姆轮双模式里程计，可接入外部姿态 yaw。
- Linux SBC 到 C 板的 RGB-D 视觉摘要协议解析与整车状态透传。
- `sbc/orange_pi_vision/` 提供 Orange Pi AI Pro 8T 视觉发送端框架。
- CAN 精确过滤、反馈新鲜度检查、控制漏拍保护和独立看门狗。
- 固定 1 kHz 控制循环；中断只处理定长通信数据。
- 纯逻辑模块可在主机执行单元测试，固件由 CI 交叉编译到 Cortex-M4F。

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407VGT6 |
| 语言 | Rust 2021、`no_std` |
| 目标 | `thumbv7em-none-eabihf` |
| 底盘 | 4 × M3508 + C620 |
| 云台 | RoboMaster 6623 + GM6020 |
| 遥控 | FS-i6 + FS-A8S S.BUS |
| SBC | 香橙派 AI Pro 8T |
| 深度相机 | Orbbec DaBai DCW |
| 调度 | 1 kHz 固定周期主循环 |
| 许可证 | MIT |

## 安全设计

> [!CAUTION]
> 首次测试必须架空车轮和云台。仓库中的机械参数与 PID 只是保守起点，不能替代实车标定。

固件按“失效时输出归零”设计：

- 上电默认锁定；SwC 高档立即锁车。
- SwC 中档且四个主摇杆居中持续 1 秒后解锁。
- 遥控失联、S.BUS failsafe、丢帧或超过 `100 ms` 未收到有效帧时整车锁零。
- 电机反馈超过 `20 ms` 未更新时，对应子系统清空 PID 并锁零。
- 四台底盘电机必须全部在线，底盘才允许输出。
- 控制循环间隔超过 `5 ms` 时整车锁零，并要求重新解锁。
- 主循环卡死约 `500 ms` 后由独立看门狗复位。
- CAN 只接收预期标准数据帧；扩展帧、远程帧、错误 DLC 和错误总线映射会被拒绝。

云台默认处于未标定状态：

```rust
pub const GIMBAL_CALIBRATION: GimbalCalibration = GimbalCalibration {
    calibrated: false,
    pitch_encoder_zero: 0,
    pitch_encoder_direction: 1.0,
};
```

必须在 [`src/config/gimbal.rs`](src/config/gimbal.rs) 填写俯仰机械零点和方向，
架空确认角度、方向与机械限位正确后，才能将 `calibrated` 改为 `true`。
未标定时云台始终输出零电流，底盘仍可独立运行。

## 硬件拓扑

```text
Orbbec DaBai DCW ── USB ── Orange Pi AI Pro 8T
                                │ frame summary, UART/UDP/SPI 等链路
                                ▼
FS-i6 / FS-A8S ── S.BUS ── STM32F407VGT6 RoboMaster C 型开发板
                                ├── CAN1 1 Mbps ── 4 × C620 / M3508 底盘
                                └── CAN2 1 Mbps ── 6623 偏航 + GM6020 俯仰
```

### 底盘 CAN1

| 机械位置 | 电机 ID | 反馈 ID | `0x200` 控制帧位置 |
| --- | ---: | ---: | --- |
| 左前 | 1 | `0x201` | `DATA[0..1]` |
| 右前 | 2 | `0x202` | `DATA[2..3]` |
| 左后 | 3 | `0x203` | `DATA[4..5]` |
| 右后 | 4 | `0x204` | `DATA[6..7]` |

默认电机方向为 `[+1, -1, +1, -1]`，配置位于
[`src/config/chassis.rs`](src/config/chassis.rs)。

底盘默认功率保护：

- 目标转速斜坡：`2000 RPM/s`
- 单路电流斜坡：`20000/s`
- 单路电流限幅：`6000`
- 四路绝对电流总预算：`12000`

### 云台 CAN2

| 轴 | 电机 | 反馈 ID | 控制 ID | 电流限幅 |
| --- | --- | ---: | ---: | ---: |
| 偏航 | RoboMaster 6623 | `0x205` | `0x1FF` | `±5000` |
| 俯仰 | GM6020 | `0x206` | `0x1FF` | `±20000` |

6623 不直接返回转速。固件根据 8192 线绝对编码器和真实反馈时间间隔进行差分测速，
再经过低通滤波输入速度环。6623 与 GM6020 使用不同的反馈解析逻辑，协议实现位于
[`src/domain/can_protocol.rs`](src/domain/can_protocol.rs)。

## 遥控映射

S.BUS 接收使用 USART3 RX（PC11），配置为 `100000 baud, 8E2`。

| 输入 | 通道 | 普通轮模式 | 麦克纳姆轮模式 |
| --- | --- | --- | --- |
| 左摇杆竖直 | CH3 | 前后 | 前后 |
| 左摇杆水平 | CH4 | 转向 | 横移 |
| 右摇杆水平 | CH1 | 云台偏航 | 云台偏航 |
| 右摇杆竖直 | CH2 | 云台俯仰 | 云台俯仰 |
| SwB | CH6 | 上档选择普通轮 | 下档选择麦克纳姆轮 |
| SwC | CH5 | 高档锁车 | 中档长按解锁 |

切换 SwB 时，底盘清空四路速度 PID，并输出一个控制周期的零电流。

## 软件架构

```text
src/
├── app/           整车命令、安全状态与子系统编排
├── chassis/       双模式运动学、四路 M3508 速度控制与功率限制
├── config/        CAN、底盘、云台、遥控与系统参数
├── control/       多个子系统共享的基础控制算法
├── domain/        命令、反馈、CAN 协议与 S.BUS 解码
├── estimation/    姿态接口、双模式里程计与二维位姿
├── gimbal/        6623/GM6020 级联控制与机械标定
├── platform/      STM32 CAN、USART 中断与独立看门狗
└── main.rs        时钟、引脚、中断入口、RGB 与周期调度

sbc/
└── orange_pi_vision/
    ├── src/vision_link.rs                 SBC 侧视觉摘要包编码
    └── src/bin/send_camera_to_robot.rs    DaBai DCW 到 C 板的发送器
```

数据流：

```text
USART3 / CAN 中断
        │
        ▼
定长反馈快照
        │
        ▼
RobotController（1 kHz）
   ├── RemoteController
   ├── ChassisController
   ├── GimbalController
   └── ChassisOdometry
        │
        ▼
CAN1 0x200 / CAN2 0x1FF
```

控制路径不使用堆分配。硬件访问集中在 `platform`，纯控制逻辑不依赖 STM32 PAC，
因此可以在主机测试，也便于后续迁移到 RTIC 或 Embassy。

详见 [架构说明](docs/ARCHITECTURE.md) 与
[接线和调参](docs/HARDWARE_AND_TUNING.md)。Linux SBC 到 C 板的视觉数据协议见
[相机数据链路](docs/VISION_LINK.md)。基础功能完成范围、装车标定项和
后续扩展边界见 [基础框架完成状态](docs/BASELINE_STATUS.md)。

## 快速开始

### 环境

- Rust `1.95.0`（由 `rust-toolchain.toml` 固定）
- `thumbv7em-none-eabihf`
- `gcc-arm-none-eabi`
- 烧录时需要 OpenOCD 与 ST-Link

### 检查与构建

```sh
git clone https://github.com/zong1024/RM-Robot-Rust.git
cd RM-Robot-Rust

rustup target add thumbv7em-none-eabihf
make check
```

`make check` 依次执行：

- `cargo fmt --check`
- 主机单元测试
- 主机库 Clippy（`-D warnings`）
- ARM 固件 binary Clippy（`-D warnings`）
- Cortex-M4F release 构建
- Orange Pi 视觉协议子项目测试和 Clippy

烧录：

```sh
make flash
```

`make build` 会从 `/tmp` 启动 Cargo，避免父目录中的 Cargo 配置被重复合并。

### Orange Pi AI Pro 8T 视觉发送器

协议库不需要 Orbbec SDK，可直接测试：

```sh
cargo test --manifest-path sbc/orange_pi_vision/Cargo.toml
```

在香橙派 AI Pro 8T 上实际采集 DaBai DCW，需要安装 aarch64 OrbbecSDK v1 并启用
`orbbec-sdk` feature：

```sh
export ORBBEC_SDK_V1_DIR=/opt/OrbbecSDK_v1.10.18/SDK
export LD_LIBRARY_PATH="$ORBBEC_SDK_V1_DIR/lib:$LD_LIBRARY_PATH"

cargo build --release \
  --manifest-path sbc/orange_pi_vision/Cargo.toml \
  --features orbbec-sdk \
  --bin send_camera_to_robot
```

运行示例：

```sh
sbc/orange_pi_vision/target/release/send_camera_to_robot \
  --serial /dev/ttyUSB0 \
  --baud 921600 \
  --rate-hz 10 \
  --rgb-size 640x480
```

## 首次装车

1. 架空车轮和云台，在无负载状态测试。
2. 检查 CAN 总线两端终端电阻、供电和线序。
3. 确认底盘 ID1～4 分别对应左前、右前、左后、右后。
4. 确认 6623 拨码为 Yaw（反馈 `0x205`），GM6020 为 ID 2（反馈 `0x206`）。
5. 校验 `CHASSIS_MOTOR_DIRECTION`，保证正向命令对应整车前进。
6. 实测轮径、轮距、轴距和减速比，更新 `src/config/chassis.rs`。
7. 填写俯仰编码器机械零点和方向，确认机械限位后启用 `GIMBAL_CALIBRATION`。
8. 从低电流开始重新标定底盘与云台 PID。

完整接线、排障和调参流程见
[`docs/HARDWARE_AND_TUNING.md`](docs/HARDWARE_AND_TUNING.md)。

## 运行诊断

RGB 为低电平点亮：

| 指示灯 | 引脚 | 含义 |
| --- | --- | --- |
| 红 | PH12 | 底盘或云台任一必需电机离线 |
| 绿 | PH11 | 每 500 ms 翻转，表示主循环运行 |
| 蓝 | PH10 | 遥控安全门已解锁 |

可通过 SWD 查看以下诊断变量：

| 变量 | 含义 |
| --- | --- |
| `ROBOT_ARMED` | 整车安全门状态 |
| `CHASSIS_MOTOR_ONLINE_MASK` | bit0～bit3 对应底盘 ID1～4 |
| `SWB_CHANNEL_RAW` | SwB 原始通道值 |
| `CHASSIS_WHEEL_MODE` | `0` 普通轮，`1` 麦克纳姆轮 |
| `CAN1_ID_RX_COUNT` | 四个底盘反馈 ID 的接收计数 |
| `CAN_INVALID_FRAME_COUNT` | 被协议校验拒绝的 CAN 帧数 |
| `CAN_RX_BUDGET_EXHAUSTED_COUNT` | CAN 中断达到单次处理预算的次数 |
| `CONTROL_TIMING_FAULT_COUNT` | 控制循环间隔超过 5 ms 的次数 |
| `GIMBAL_CALIBRATED` | 云台是否已显式标定 |

## 文档

- [架构说明](docs/ARCHITECTURE.md)
- [接线和调参](docs/HARDWARE_AND_TUNING.md)
- [基础框架完成状态](docs/BASELINE_STATUS.md)
- [控制安全加固设计](docs/SAFETY_HARDENING_DESIGN.md)
- [AI 交接文档](AI_HANDOFF.md)

## 参考资料

- [DJI RoboMaster 6623 电调说明书](https://rm-static.djicdn.com/tem/1f53d24dad94e151687436607599258.pdf)
- [DJI RoboMaster 6623 电机产品页](https://www.robomaster.com/zh-CN/products/components/detail/131)
- RoboMaster C 型开发板官方示例：`Development-Board-C-Examples/20.standard_robot`

## License

本项目基于 [MIT License](LICENSE) 开源。
