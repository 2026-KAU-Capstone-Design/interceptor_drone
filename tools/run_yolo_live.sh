#!/usr/bin/env bash
set -e

cd ~/Documents/interceptor_drone/ros2_ws
source install/setup.bash

python3 ~/Documents/interceptor_drone/tools/yolo_live_detect.py \
  --model /home/ljy0313/Documents/interceptor_drone/ros2_ws/runs/detect/balloon_yolov8n_v3/weights/best.pt \
  --topic /camera/image \
  --conf 0.4 \
  --show \
  --min-red-ratio 0.03 \
  --max-box-area-ratio 0.85 \
  --max-jump-px 600 \
  --reset-after-misses 5
