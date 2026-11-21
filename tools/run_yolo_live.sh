#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$PROJECT_DIR/ros2_ws"
source install/setup.bash

python3 "$PROJECT_DIR/tools/yolo_live_detect.py" \
  --model "$PROJECT_DIR/ros2_ws/src/interceptor_control/models/target_yolov8n_v11_real_balloon_best.pt" \
  --topic /camera/image \
  --conf 0.4 \
  --target-class "${1:-0}" \
  --show \
  --min-red-ratio 0.03 \
  --max-box-area-ratio 0.85 \
  --max-jump-px 600 \
  --reset-after-misses 5
