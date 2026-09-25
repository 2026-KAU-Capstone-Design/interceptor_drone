#!/bin/bash

echo "=== Rocket Drone Simulation Full Reset ==="

pkill -9 -f high_speed_velocity_mission 2>/dev/null
pkill -9 -f hover_mission 2>/dev/null
pkill -9 -f point_mission 2>/dev/null
pkill -9 -f circle_mission 2>/dev/null

pkill -9 -f MicroXRCEAgent 2>/dev/null

pkill -9 -f px4 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f gz-sim 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null

ros2 daemon stop 2>/dev/null

sleep 2

rm -rf /tmp/px4* 2>/dev/null
rm -rf /tmp/gz* 2>/dev/null
rm -rf /tmp/ign* 2>/dev/null

echo "=== Remaining Simulation Processes ==="
ps -ef | grep -E '[p]x4|[g]z|MicroXRCEAgent'

echo "=== RESET COMPLETE ==="
