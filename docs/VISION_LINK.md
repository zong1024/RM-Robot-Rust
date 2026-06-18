# Linux SBC 视觉数据链路

本仓库现在预留了 Linux SBC 到 C 板的数据入口：Orbbec DaBai DCW 接在 Linux SBC
上，由 SBC 完成 RGB-D 采集和预处理，再把紧凑视觉摘要发给 RoboMaster C 型开发板。
协议实现位于
`src/domain/vision.rs`。它是 `no_std`、无堆分配、逐字节解析的二进制协议，可挂到
任意后续确定的 UART、USB CDC、SPI 或 CAN 分片接收层。

## 设计取舍

DaBai DCW 当前可用的深度流是 `640x360@30fps`，单帧 16-bit 深度约 450 KiB；
再加 RGB 后远超 STM32F407 控制板适合直接接收的实时数据量。因此 SBC 侧先计算
紧凑的帧摘要：

- RGB 是否有效、宽高。
- Depth 是否有效、宽高。
- 深度最小值、最大值、中心点、均值，单位 mm。
- 4x4 深度采样网格，单位 mm。
- 相机端序号和采集时间戳。

完整 RGB/depth 图像留在 Linux SBC 处理，C 板只接收控制所需的
低带宽感知结果。

## 数据包

Header 固定 16 字节：

| Offset | Size | 字段 |
| ---: | ---: | --- |
| 0 | 2 | magic: `OB` |
| 2 | 1 | version: `1` |
| 3 | 1 | packet kind: `1` 表示 frame summary |
| 4 | 4 | sequence, little-endian |
| 8 | 4 | captured_at_ms, little-endian |
| 12 | 2 | payload_len, little-endian |
| 14 | 2 | CRC16-CCITT-FALSE over header[0..14] + payload |

Frame summary payload 固定 50 字节：

| Offset | Size | 字段 |
| ---: | ---: | --- |
| 0 | 2 | flags: bit0 depth valid, bit1 RGB valid |
| 2 | 2 | depth width |
| 4 | 2 | depth height |
| 6 | 2 | RGB width |
| 8 | 2 | RGB height |
| 10 | 2 | depth min mm |
| 12 | 2 | depth max mm |
| 14 | 2 | depth center mm |
| 16 | 2 | depth mean mm |
| 18 | 32 | 4x4 depth grid, row-major, u16 mm |

## 固件接入方式

平台层拿到字节后逐个调用：

```rust
use rm_robot::domain::vision::{VisionPacket, VisionPacketParser};

let mut parser = VisionPacketParser::new();
if let Ok(Some(VisionPacket::FrameSummary(summary))) = parser.push(byte) {
    // 保存到平台快照，随后填入 RobotSensors.vision。
}
```

当前 `RobotSensors` 和 `RobotOutput` 已包含 `vision: VisionFrameSummary`。在实际接线
确定前，`main.rs` 暂时填入默认值，不影响现有底盘/云台控制。
