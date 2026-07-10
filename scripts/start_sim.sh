#!/usr/bin/env bash
# 시뮬레이션 환경 기동 스크립트
# 사용: bash scripts/start_sim.sh
#
# T1에서 먼저 실행할 것:
#   MicroXRCEAgent udp4 -p 8888
#
# 이 스크립트가 종료되면 Ctrl+C 후 재실행

set -e

WORLD="${PX4_GZ_WORLD:-simple_windy_balloon}"
PX4_DIR="${HOME}/dev/PX4-Autopilot"
WORLD_FILE="${PX4_DIR}/Tools/simulation/gz/worlds/${WORLD}.sdf"

echo "[start_sim] GZ world: ${WORLD}"
echo "[start_sim] SDF: ${WORLD_FILE}"

# 모델 경로 설정 (x500_mono_cam SDF 탐색에 필요)
GZ_ENV="${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh"
if [ -f "${GZ_ENV}" ]; then
    # shellcheck disable=SC1090
    source "${GZ_ENV}"
fi

# NVIDIA EGL surfaceless + headless 렌더링
unset DISPLAY
export GZ_IP=127.0.0.1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

exec gz sim -r -s --headless-rendering "${WORLD_FILE}"
