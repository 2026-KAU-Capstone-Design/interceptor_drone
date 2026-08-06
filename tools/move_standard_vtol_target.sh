#!/usr/bin/env bash

WORLD="${1:-simple_standard_vtol_target}"
MODEL="${2:-target_fixed_wing}"

echo "[move_target] world: ${WORLD}"
echo "[move_target] model: ${MODEL}"

t=0

while true; do
  values=$(python3 - <<PY
import math

t = ${t}

# 표적 위치 궤적
x = 9.0 + 2.0 * math.sin(t * 0.04)
y = 1.5 * math.sin(t * 0.025)
z = 3.5

# yaw 회전
yaw = 3.14 + 0.25 * math.sin(t * 0.03)

# yaw -> quaternion
qx = 0.0
qy = 0.0
qz = math.sin(yaw / 2.0)
qw = math.cos(yaw / 2.0)

print(f"{x} {y} {z} {qx} {qy} {qz} {qw}")
PY
)

  read x y z qx qy qz qw <<< "$values"

  gz service -s /world/${WORLD}/set_pose \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req "name: '${MODEL}', position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {x: ${qx}, y: ${qy}, z: ${qz}, w: ${qw}}" > /dev/null

  t=$((t + 1))
  sleep 0.05
done
