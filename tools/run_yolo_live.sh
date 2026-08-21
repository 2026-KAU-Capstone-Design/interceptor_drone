#!/usr/bin/env bash
set -e

# Repository root
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${REPO_DIR}/ros2_ws"
source install/setup.bash

python3 "${REPO_DIR}/tools/yolo_live_detect.py" \
  --model "${REPO_DIR}/ros2_ws/src/interceptor_control/models/target_yolov8n_v9_standard_vtol_front_best.pt" \
  --topic /camera/image \
  --conf 0.1 \
  --show \
  --min-red-ratio 0.03 \
  --max-box-area-ratio 0.85 \
  --max-jump-px 600 \
  --reset-after-misses 5
