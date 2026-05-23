#!/usr/bin/env bash

echo "=== topic list ==="
ros2 topic list | grep target

echo
echo "=== /target/distance_m ==="
timeout 3 ros2 topic echo /target/distance_m || true

echo
echo "=== /target/near_2m ==="
timeout 3 ros2 topic echo /target/near_2m || true

echo
echo "=== /target/lock_on ==="
timeout 3 ros2 topic echo /target/lock_on || true

echo
echo "=== /target/balloon_bbox ==="
timeout 3 ros2 topic echo /target/balloon_bbox || true
