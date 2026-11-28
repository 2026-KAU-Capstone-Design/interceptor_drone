#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR/ros2_ws"
source install/setup.bash

ARGS=(
  --model "$PROJECT_DIR/ros2_ws/src/interceptor_control/models/target_yolov8n_v12_real_balloon_best.pt"
  --topic /camera/image
  --conf 0.4
  --target-class 0
  --min-red-ratio 0.03
  --max-box-area-ratio 0.85
  --max-jump-px 600
  --reset-after-misses 5
)

# MONITOR_DEST_IP must be the laptop's actual Wi-Fi IPv4 address.
if [[ -n "${MONITOR_DEST_IP:-}" ]]; then
  ARGS+=(
    --monitor-dest "$MONITOR_DEST_IP"
    --monitor-fps "${MONITOR_FPS:-10}"
    --monitor-bitrate "${MONITOR_BITRATE_K:-1200}"
  )
fi

# Headless Jetson: default OFF; desktop Gazebo: YOLO_SHOW=1
if [[ "${YOLO_SHOW:-0}" == "1" ]]; then
  ARGS+=(--show)
fi

python3 "$PROJECT_DIR/tools/yolo_live_detect.py" "${ARGS[@]}"
