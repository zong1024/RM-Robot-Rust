# Orange Pi AI Pro Vision Sender

这个子项目是 RM-Robot-Rust 的 Linux SBC 侧框架。目标硬件是香橙派 AI Pro 8T，
负责接入 Orbbec DaBai DCW 深度相机，后续承载感知、自动驾驶和高层决策。

在整车架构里：

- Orange Pi AI Pro 8T 是“大脑”：处理 RGB-D 感知、路径/目标决策和高层策略。
- STM32F407 C 板是“小脑”：执行底盘、云台、遥控安全门和电机实时控制。
- 两者之间先用低带宽 frame summary 协议连接，避免 STM32 接收全量图像。

## 当前功能

- `src/vision_link.rs`：SBC 到 C 板的 66 字节视觉摘要包编码。
- `src/bin/send_camera_to_robot.rs`：通过 OrbbecSDK v1 采集 DaBai DCW 深度帧，
  计算 4x4 深度网格和 min/max/center/mean，再经串口、UDP 或 stdout 发送。

协议字段必须和 C 板侧 `src/domain/vision.rs` 保持一致。

## 开发机验证

不需要 Orbbec SDK，只验证协议编码：

```sh
cargo test --manifest-path sbc/orange_pi_vision/Cargo.toml
```

## 香橙派 AI Pro 8T 构建

香橙派通常是 Linux aarch64。必须安装 aarch64 OrbbecSDK v1，并设置 SDK 路径：

```sh
export ORBBEC_SDK_V1_DIR=/opt/OrbbecSDK_v1.10.18/SDK
export LD_LIBRARY_PATH="$ORBBEC_SDK_V1_DIR/lib:$LD_LIBRARY_PATH"

cargo build --release \
  --manifest-path sbc/orange_pi_vision/Cargo.toml \
  --features orbbec-sdk \
  --bin send_camera_to_robot
```

如果没有设置 `ORBBEC_SDK_V1_DIR`，ARM64 构建会直接失败，避免误链接 x86_64 SDK。

## 运行示例

串口发送给 C 板：

```sh
target/release/send_camera_to_robot \
  --serial /dev/ttyUSB0 \
  --baud 921600 \
  --rate-hz 10 \
  --rgb-size 640x480
```

UDP 调试：

```sh
target/release/send_camera_to_robot \
  --udp 192.168.1.20:5000 \
  --rate-hz 10 \
  --rgb-size 640x480
```

不传 `--serial` 或 `--udp` 时，程序会把二进制包写到 stdout，便于管道测试。

## systemd 部署

仓库提供运行脚本和服务模板：

- `scripts/run_camera_sender.sh`
- `vision_sender.env.example`
- `systemd/rm-vision-sender.service`

部署到树莓派或香橙派后：

```sh
cd ~/rm_robot_rust/sbc/orange_pi_vision
cp vision_sender.env.example vision_sender.env
vim vision_sender.env

sudo cp systemd/rm-vision-sender.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start rm-vision-sender.service
sudo journalctl -u rm-vision-sender.service -f
```

确认 C 板串口接线和设备节点后，再设置开机自启：

```sh
sudo systemctl enable rm-vision-sender.service
```

默认环境文件使用 `/dev/ttyAMA0` 和 `921600` baud。未确认串口前不要启用开机自启。

## 硬件检查

```sh
uname -m
lsusb | grep -iE '2bc5|orbbec'
ls -l /dev/ttyUSB* /dev/ttyAMA* /dev/ttyS* 2>/dev/null
```

如需非 root 访问 USB 深度设备，安装 Orbbec SDK 自带 udev 规则后重新插拔相机：

```sh
sudo "$ORBBEC_SDK_V1_DIR/../Script/install_udev_rules.sh"
sudo usermod -aG dialout "$USER"
```
