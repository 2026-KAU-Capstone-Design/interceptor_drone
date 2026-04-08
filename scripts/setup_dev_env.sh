#!/usr/bin/env bash
# =============================================================================
# Interceptor Drone — Dev Environment Bootstrap
# =============================================================================
# 한 번 실행하면 다음을 모두 설치/구성한다:
#   - ROS2 Humble Hawksbill (desktop + dev tools)
#   - Gazebo Harmonic (LTS, osrfoundation 저장소)
#   - ros-humble-ros-gzharmonic (ROS↔Gazebo 브리지)
#   - PX4-Autopilot v1.16.1 + 의존성
#   - Micro XRCE-DDS Agent v2.4.3 (PX4↔ROS2 브리지)
#   - ROS2 워크스페이스 빌드 (px4_msgs release/1.16 포함)
#   - ~/.bashrc 환경변수 설정
#
# 멱등(idempotent): 두 번 이상 실행해도 안전.
#
# 사용법:
#   bash scripts/setup_dev_env.sh
#
# 사전 조건:
#   - Ubuntu 22.04.5 LTS (jammy)
#   - sudo 권한
#   - 인터넷 연결
#   - 홈 디렉터리 30GB+ 여유 공간
# =============================================================================

set -euo pipefail

# ---------- 색상 출력 ----------
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
log()   { echo -e "${BLU}[INFO]${NC} $*"; }
ok()    { echo -e "${GRN}[ OK ]${NC} $*"; }
warn()  { echo -e "${YLW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[FAIL]${NC} $*" >&2; }
stage() { echo -e "\n${BLU}========== $* ==========${NC}"; }

# ---------- 핀(pin) 버전 — 변경은 PR로만 ----------
ROS_DISTRO="humble"
PX4_VERSION="v1.16.1"
PX4_MSGS_BRANCH="release/1.16"
XRCE_AGENT_VERSION="v2.4.3"

# ---------- 경로 ----------
DEV_DIR="${HOME}/dev"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"

# =============================================================================
# 사전 점검
# =============================================================================
stage "Pre-flight checks"

if [[ "$EUID" -eq 0 ]]; then
    err "이 스크립트는 root로 실행하지 마세요. 일반 사용자로 실행하면 sudo가 필요할 때 비밀번호를 묻습니다."
    exit 1
fi

if ! grep -q '22.04' /etc/os-release; then
    err "이 스크립트는 Ubuntu 22.04 전용입니다."
    err "현재 OS: $(. /etc/os-release && echo "${PRETTY_NAME}")"
    exit 1
fi
ok "Ubuntu 22.04 확인"

if ! ping -c1 -W2 packages.ros.org >/dev/null 2>&1; then
    warn "packages.ros.org에 도달할 수 없습니다. 인터넷을 확인하세요."
fi
ok "네트워크 확인"

AVAIL_GB=$(df -BG "$HOME" | awk 'NR==2 {gsub("G",""); print $4}')
if [ "${AVAIL_GB}" -lt 30 ]; then
    warn "홈 디렉터리 여유 공간이 ${AVAIL_GB}GB 입니다. 30GB 이상 권장."
    read -rp "그래도 계속하시겠습니까? [y/N] " ans
    [[ "${ans}" =~ ^[Yy]$ ]] || exit 1
fi
ok "디스크 여유 공간 ${AVAIL_GB}GB"

log "sudo 비밀번호를 미리 받습니다 (이후 단계에서 비밀번호 안 물음)..."
sudo -v
( while true; do sudo -n true; sleep 60; kill -0 "$$" 2>/dev/null || exit; done ) &
SUDO_KEEPALIVE_PID=$!
trap 'kill ${SUDO_KEEPALIVE_PID} 2>/dev/null || true' EXIT

# =============================================================================
# 1/7 시스템 기본 패키지
# =============================================================================
stage "1/7 시스템 기본 패키지"

sudo apt-get update
sudo apt-get install -y \
    curl wget gnupg lsb-release ca-certificates \
    software-properties-common apt-transport-https \
    build-essential cmake git \
    python3-pip python3-venv python3-dev \
    locales

if ! locale 2>/dev/null | grep -q "LANG=en_US.UTF-8"; then
    sudo locale-gen en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
fi
ok "기본 패키지 완료"

# =============================================================================
# 2/7 ROS2 Humble
# =============================================================================
stage "2/7 ROS2 Humble"

if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-desktop "; then
    sudo add-apt-repository universe -y

    if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-desktop \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    ok "ROS2 ${ROS_DISTRO} 설치 완료"
else
    ok "ROS2 ${ROS_DISTRO} 이미 설치됨 — 건너뜀"
fi

# =============================================================================
# 3/7 Gazebo Harmonic + ROS-Gazebo 브리지
# =============================================================================
stage "3/7 Gazebo Harmonic"

if ! dpkg -l | grep -q "^ii  gz-harmonic "; then
    if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
        sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
            -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y gz-harmonic
    ok "Gazebo Harmonic 설치 완료"
else
    ok "Gazebo Harmonic 이미 설치됨 — 건너뜀"
fi

if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-ros-gzharmonic "; then
    sudo apt-get install -y ros-${ROS_DISTRO}-ros-gzharmonic
    ok "ros-${ROS_DISTRO}-ros-gzharmonic 설치 완료"
else
    ok "ROS-Gazebo 브리지 이미 설치됨 — 건너뜀"
fi

# =============================================================================
# 4/7 PX4-Autopilot
# =============================================================================
stage "4/7 PX4-Autopilot ${PX4_VERSION}"

mkdir -p "${DEV_DIR}"

if [ ! -d "${DEV_DIR}/PX4-Autopilot/.git" ]; then
    log "PX4-Autopilot clone 중 (시간이 걸립니다)..."
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive "${DEV_DIR}/PX4-Autopilot"
fi

cd "${DEV_DIR}/PX4-Autopilot"
git fetch --tags --quiet
CURRENT_TAG="$(git describe --tags --exact-match 2>/dev/null || echo "")"
if [ "${CURRENT_TAG}" != "${PX4_VERSION}" ]; then
    log "PX4 체크아웃: ${PX4_VERSION}"
    git checkout "${PX4_VERSION}"
    git submodule update --init --recursive
fi

# PX4 의존성 설치 (Gazebo는 우리가 직접 osrfoundation에서 설치했으므로 --no-sim-tools)
log "PX4 의존성 설치 중 (NuttX 툴체인 포함, 시간이 걸립니다)..."
bash ./Tools/setup/ubuntu.sh --no-sim-tools
ok "PX4 ${PX4_VERSION} 준비 완료"

# =============================================================================
# 5/7 Micro XRCE-DDS Agent
# =============================================================================
stage "5/7 Micro XRCE-DDS Agent ${XRCE_AGENT_VERSION}"

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
    if [ ! -d "${DEV_DIR}/Micro-XRCE-DDS-Agent/.git" ]; then
        git clone -b "${XRCE_AGENT_VERSION}" \
            https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
            "${DEV_DIR}/Micro-XRCE-DDS-Agent"
    fi
    cd "${DEV_DIR}/Micro-XRCE-DDS-Agent"
    mkdir -p build && cd build
    cmake ..
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig /usr/local/lib/
    ok "Micro XRCE-DDS Agent 설치 완료"
else
    ok "MicroXRCEAgent 이미 존재 — 건너뜀 ($(command -v MicroXRCEAgent))"
fi

# =============================================================================
# 6/7 ROS2 워크스페이스
# =============================================================================
stage "6/7 ROS2 워크스페이스 빌드"

mkdir -p "${WS_DIR}/src"
cd "${WS_DIR}"

# vcstool로 외부 저장소 import (px4_msgs 등)
if [ -f "${REPO_ROOT}/repos/dev.repos" ]; then
    log "외부 저장소 동기화 (vcstool)..."
    vcs import src < "${REPO_ROOT}/repos/dev.repos" || true
    vcs pull src 2>/dev/null || true
fi

# ROS2 환경 source 후 종속성 설치
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src -r -y || \
    warn "rosdep 일부 실패 — 다음 빌드에서 확인 필요"

log "colcon build 중..."
colcon build --symlink-install
ok "워크스페이스 빌드 완료"

# =============================================================================
# 7/7 ~/.bashrc 환경변수
# =============================================================================
stage "7/7 ~/.bashrc 설정"

BASHRC_MARK="# >>> interceptor_drone env >>>"
BASHRC_END="# <<< interceptor_drone env <<<"

if ! grep -qF "${BASHRC_MARK}" "${HOME}/.bashrc"; then
    cat >> "${HOME}/.bashrc" <<EOF

${BASHRC_MARK}
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f ${WS_DIR}/install/setup.bash ]; then
    source ${WS_DIR}/install/setup.bash
fi
export PX4_AUTOPILOT_DIR=${DEV_DIR}/PX4-Autopilot
${BASHRC_END}
EOF
    ok "~/.bashrc 업데이트됨"
else
    ok "~/.bashrc 이미 설정됨 — 건너뜀"
fi

# =============================================================================
# 완료
# =============================================================================
stage "✔ 설치 완료"

cat <<EOF

다음 단계:

1) 새 터미널을 열거나 환경 적용:
       source ~/.bashrc

2) 환경 검증 (팀원 3명 출력이 동일해야 함):
       bash ${REPO_ROOT}/scripts/verify_env.sh

3) 첫 SITL 비행 (3개 터미널):
       # T1 — PX4↔ROS2 브리지
       MicroXRCEAgent udp4 -p 8888

       # T2 — PX4 SITL + Gazebo Harmonic
       cd \$PX4_AUTOPILOT_DIR && make px4_sitl gz_x500

       # T3 — ROS2에서 토픽 확인
       ros2 topic list | grep fmu

주의:
  - PX4 의존성 설치 과정에서 사용자가 dialout 그룹에 추가됩니다.
    실제 Pixhawk USB 통신을 위해서는 한 번 로그아웃→재로그인이 필요합니다.

EOF
