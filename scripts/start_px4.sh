#!/usr/bin/env bash
# T2-B: PX4 SITL standalone 실행 스크립트
# 반드시 T2-A (start_sim.sh) 로 Gazebo 가 먼저 떠 있어야 합니다.
#
# 사용: bash scripts/start_px4.sh

set -e

PX4_DIR="${HOME}/dev/PX4-Autopilot"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
WORLD="${PX4_GZ_WORLD:-simple_windy_balloon}"

# 모델 경로 설정 (없으면 gz_bridge 가 model.sdf 를 못 찾음)
source "${BUILD_DIR}/rootfs/gz_env.sh"

export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="${WORLD}"
export PX4_SIM_MODEL=gz_x500_mono_cam
export GZ_IP=127.0.0.1

echo "[start_px4] world: ${WORLD}"
echo "[start_px4] PX4 binary: ${BUILD_DIR}/bin/px4"

cd "${BUILD_DIR}"
exec ./bin/px4 rootfs -s etc/init.d-posix/rcS -i 0 -d
