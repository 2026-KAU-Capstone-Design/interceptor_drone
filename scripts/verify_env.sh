#!/usr/bin/env bash
# =============================================================================
# Interceptor Drone — Environment Verification
# =============================================================================
# 팀원 3명의 환경이 동일한지 비교하기 위한 출력 스크립트.
# 출력 결과를 캡처해서 회의 때 비교하세요.
# =============================================================================

# 일부 도구가 없어도 끝까지 진행
set +e

# ROS2가 source되지 않은 새 셸에서도 동작하도록 시도
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

GRN='\033[0;32m'; RED='\033[0;31m'; YLW='\033[1;33m'; NC='\033[0m'

print_row() { printf "  %-22s : %s\n" "$1" "$2"; }
ok_mark()   { echo -e "${GRN}OK${NC}"; }
fail_mark() { echo -e "${RED}MISSING${NC}"; }

echo "============================================================"
echo " Interceptor Drone — Environment Verification"
echo " host : $(hostname)"
echo " user : $(whoami)"
echo " date : $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "============================================================"

echo ""
echo "[ System ]"
print_row "OS"          "$(. /etc/os-release && echo "${PRETTY_NAME}")"
print_row "Kernel"      "$(uname -r)"
print_row "Arch"        "$(uname -m)"
print_row "CPU cores"   "$(nproc)"
print_row "RAM"         "$(free -h | awk '/^Mem:/ {print $2}')"

echo ""
echo "[ ROS2 ]"
if command -v ros2 >/dev/null 2>&1; then
    print_row "ROS_DISTRO"      "${ROS_DISTRO:-(not sourced)}"
    print_row "ros2 cli"        "$(ros2 --help >/dev/null 2>&1 && ok_mark || fail_mark)"
    print_row "colcon"          "$(command -v colcon >/dev/null && ok_mark || fail_mark)"
    print_row "rosdep"          "$(command -v rosdep >/dev/null && ok_mark || fail_mark)"
    print_row "vcstool"         "$(command -v vcs    >/dev/null && ok_mark || fail_mark)"
else
    print_row "ros2 cli"        "$(fail_mark)"
fi

echo ""
echo "[ Gazebo ]"
if command -v gz >/dev/null 2>&1; then
    GZ_VER="$(gz sim --version 2>/dev/null | tail -1 | tr -d ' ')"
    print_row "gz sim version"  "${GZ_VER:-unknown}"
    print_row "ros_gz bridge"   "$(dpkg -l 2>/dev/null | grep -q ros-humble-ros-gzharmonic && ok_mark || fail_mark)"
else
    print_row "gz cli"          "$(fail_mark)"
fi

echo ""
echo "[ PX4 ]"
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/dev/PX4-Autopilot}"
if [ -d "${PX4_DIR}/.git" ]; then
    PX4_TAG="$(git -C "${PX4_DIR}" describe --tags --always 2>/dev/null)"
    PX4_SHA="$(git -C "${PX4_DIR}" rev-parse --short HEAD 2>/dev/null)"
    print_row "PX4 path"        "${PX4_DIR}"
    print_row "PX4 git tag"     "${PX4_TAG}"
    print_row "PX4 git commit"  "${PX4_SHA}"
else
    print_row "PX4-Autopilot"   "$(fail_mark) (expected at ${PX4_DIR})"
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
    print_row "MicroXRCEAgent"  "$(command -v MicroXRCEAgent)"
else
    print_row "MicroXRCEAgent"  "$(fail_mark)"
fi

echo ""
echo "[ Workspace ]"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../ros2_ws" 2>/dev/null && pwd)"
if [ -n "${WS_DIR}" ] && [ -d "${WS_DIR}/install" ]; then
    print_row "workspace path"  "${WS_DIR}"
    print_row "build status"    "$(ok_mark)"
    if [ -d "${WS_DIR}/src/px4_msgs/.git" ]; then
        PXM_BRANCH="$(git -C "${WS_DIR}/src/px4_msgs" rev-parse --abbrev-ref HEAD 2>/dev/null)"
        print_row "px4_msgs branch" "${PXM_BRANCH}"
    else
        print_row "px4_msgs"     "$(fail_mark)"
    fi
else
    print_row "workspace"       "$(fail_mark) (run setup_dev_env.sh)"
fi

echo ""
echo "[ Python ]"
print_row "python3"             "$(python3 --version 2>&1 | awk '{print $2}')"
print_row "pip3"                "$(pip3 --version 2>&1 | awk '{print $2}' || echo missing)"

echo ""
echo "[ GPU (참고용 — 팀원마다 다를 수 있음) ]"
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null \
        | sed 's/^/  /'
else
    lspci 2>/dev/null | grep -iE 'vga|3d|display' | head -3 | sed 's/^/  /'
fi

echo ""
echo "============================================================"
echo " 위 결과를 캡처해서 팀원들과 비교하세요."
echo " [ System ] ~ [ Workspace ] 의 값이 모두 같아야 합니다."
echo " GPU 항목은 달라도 무방합니다."
echo "============================================================"
