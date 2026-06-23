#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${VISION_ENV_FILE:-$ROOT_DIR/vision_sender.env}"

if [[ -f "$ENV_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$ENV_FILE"
fi

: "${ORBBEC_SDK_V1_DIR:?Set ORBBEC_SDK_V1_DIR to the OrbbecSDK v1 SDK directory}"

BIN="${VISION_SENDER_BIN:-$ROOT_DIR/target/aarch64-unknown-linux-gnu/release/send_camera_to_robot}"
RATE_HZ="${VISION_RATE_HZ:-10}"
RGB_SIZE="${VISION_RGB_SIZE:-640x480}"
BAUD="${VISION_BAUD:-921600}"

export LD_LIBRARY_PATH="$ORBBEC_SDK_V1_DIR/lib:${LD_LIBRARY_PATH:-}"

args=("--rate-hz" "$RATE_HZ")

if [[ "${VISION_NO_RGB:-0}" == "1" ]]; then
  args+=("--no-rgb")
else
  args+=("--rgb-size" "$RGB_SIZE")
fi

if [[ -n "${VISION_SERIAL:-}" ]]; then
  args+=("--serial" "$VISION_SERIAL" "--baud" "$BAUD")
elif [[ -n "${VISION_UDP:-}" ]]; then
  args+=("--udp" "$VISION_UDP")
fi

exec "$BIN" "${args[@]}"
