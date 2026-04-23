#!/usr/bin/env bash
# =============================================================================
# Interceptor Drone — Dev Environment Bootstrap
# =============================================================================
# 한 번 실행하면 다음을 모두 설치/구성한다:
#   - ROS2 Humble Hawksbill (desktop + dev tools)
#   - Gazebo Harmonic (LTS, osrfoundation 저장소)
#   - ros-humble-ros-gzharmonic (ROS↔Gazebo 브리지)
#   - PX4-Autopilot v1.16.1 + 의존성 + SITL airframe 패치 (NAV_DLL_ACT=0)
#   - Micro XRCE-DDS Agent v2.4.3 (PX4↔ROS2 브리지)
#   - ~/.bashrc 환경변수 설정
#   - ROS2 워크스페이스 빌드 (px4_msgs release/1.16 포함, 재시도 로직 포함)
#   - verify_env.sh 자동 실행으로 결과 출력
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

# NOTE: set -u (nounset)는 의도적으로 사용하지 않습니다.
# ROS2의 /opt/ros/humble/setup.bash는 내부에서 AMENT_TRACE_SETUP_FILES 등의
# 변수를 default 없이 참조하기 때문에, set -u 상태에서 source하면 즉시 종료됩니다.
# (ERR trap도 parameter expansion error는 잡지 못함)
# set -e와 pipefail만으로 충분히 안전하게 동작합니다.
set -eo pipefail

# ---------- 색상 출력 ----------
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
log()   { echo -e "${BLU}[INFO]${NC} $*"; }
ok()    { echo -e "${GRN}[ OK ]${NC} $*"; }
warn()  { echo -e "${YLW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[FAIL]${NC} $*" >&2; }
stage() { CURRENT_STAGE="$*"; echo -e "\n${BLU}========== $* ==========${NC}"; }

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

# ---------- 에러 트랩 ----------
CURRENT_STAGE="(초기화)"
on_error() {
    local exit_code=$?
    local line_no=$1
    err ""
    err "============================================================"
    err " 스크립트 실패"
    err "============================================================"
    err " 실패 단계 : ${CURRENT_STAGE}"
    err " 라인 번호 : ${line_no}"
    err " 종료 코드 : ${exit_code}"
    err ""
    err " 디버깅:"
    err "   - 위 출력의 마지막 줄에서 에러 메시지를 확인하세요"
    err "   - 인터넷 연결과 디스크 여유 공간을 확인하세요"
    err "   - 같은 스크립트를 다시 실행하면 멱등하게 이어서 진행됩니다"
    err "============================================================"
}
trap 'on_error $LINENO' ERR

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
    if [ -t 0 ]; then
        read -rp "그래도 계속하시겠습니까? [y/N] " ans
        [[ "${ans}" =~ ^[Yy]$ ]] || exit 1
    else
        warn "비대화형 실행이므로 그대로 진행합니다."
    fi
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
# 4/7 PX4-Autopilot + airframe 패치
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

# X500 SITL airframe 패치 — NAV_DLL_ACT=0 (시뮬에서 GCS 없이 시동 가능)
PX4_AIRFRAME_FILE="${DEV_DIR}/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"
PX4_PARAM_MARK="# >>> interceptor_drone params >>>"
PX4_PARAM_END="# <<< interceptor_drone params <<<"

if [ ! -f "${PX4_AIRFRAME_FILE}" ]; then
    err "PX4 X500 airframe 파일이 없습니다: ${PX4_AIRFRAME_FILE}"
    err "PX4 버전이 ${PX4_VERSION}이 맞는지 확인하세요."
    exit 1
fi

if ! grep -qF "${PX4_PARAM_MARK}" "${PX4_AIRFRAME_FILE}"; then
    cat >> "${PX4_AIRFRAME_FILE}" <<EOF

${PX4_PARAM_MARK}
# 시뮬레이션 환경에서 GCS(QGroundControl) 연결 없이 시동 가능하게 함.
# 실기체로 옮길 때는 NAV_DLL_ACT를 1(Hold) 또는 2(Return)로 변경 권장.
# 자세한 설명: docs/px4_params.md
param set-default NAV_DLL_ACT 0
${PX4_PARAM_END}
EOF
    ok "X500 airframe에 NAV_DLL_ACT=0 추가됨"
else
    ok "X500 airframe 패치 이미 적용됨 — 건너뜀"
fi

# X500 모델 패치 — enable_wind 추가 (Gazebo 바람이 드론에 실제로 힘을 가하게 함)
# PX4 기본 X500 모델에는 <enable_wind> 태그가 없어서, 월드에 바람을 설정해도
# 드론이 바람 힘을 받지 못함. base_link에 enable_wind=true 를 추가하면 해결됨.
X500_MODEL_FILE="${DEV_DIR}/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf"

if [ -f "${X500_MODEL_FILE}" ]; then
    if ! grep -q "<enable_wind>" "${X500_MODEL_FILE}"; then
        sed -i 's|<gravity>true</gravity>|<gravity>true</gravity>\n      <enable_wind>true</enable_wind>|' "${X500_MODEL_FILE}"
        ok "X500 모델에 <enable_wind>true</enable_wind> 추가됨"
    else
        ok "X500 모델 enable_wind 이미 적용됨 — 건너뜀"
    fi
else
    warn "X500 모델 파일을 찾을 수 없습니다: ${X500_MODEL_FILE}"
fi

# 커스텀 Gazebo 월드 복사 (baylands_windy, baylands_storm)
PX4_WORLDS_DIR="${DEV_DIR}/PX4-Autopilot/Tools/simulation/gz/worlds"
if [ -d "${REPO_ROOT}/simulation/worlds" ]; then
    cp -n "${REPO_ROOT}"/simulation/worlds/*.sdf "${PX4_WORLDS_DIR}/" 2>/dev/null || true
    ok "커스텀 Gazebo 월드를 PX4 worlds 폴더로 복사"
fi

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
# 6/7 ~/.bashrc 환경변수 (워크스페이스 빌드 *전*에 설정)
# =============================================================================
# bashrc는 colcon build 결과와 무관하게 항상 설정한다.
# 이전 버전에서는 stage 7로 마지막에 두어서, colcon build가 일시적으로 실패하면
# bashrc까지 누락되는 사고가 있었음. 이제는 build 전에 설정하여 build가 실패해도
# 사용자는 환경변수와 ROS2 source가 잡힌 셸을 가질 수 있다.
# =============================================================================
stage "6/7 ~/.bashrc 설정"

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
# 7/7 ROS2 워크스페이스 빌드 (재시도 로직 포함)
# =============================================================================
stage "7/7 ROS2 워크스페이스 빌드"

mkdir -p "${WS_DIR}/src"
cd "${WS_DIR}"

# vcstool로 외부 저장소 import (px4_msgs 등) — 재시도 2회
if [ -f "${REPO_ROOT}/repos/dev.repos" ]; then
    log "외부 저장소 동기화 (vcstool)..."
    VCS_TRIES=0
    VCS_MAX=2
    while [ "${VCS_TRIES}" -lt "${VCS_MAX}" ]; do
        VCS_TRIES=$((VCS_TRIES + 1))
        if vcs import src < "${REPO_ROOT}/repos/dev.repos" 2>/dev/null; then
            break
        fi
        warn "vcs import 시도 ${VCS_TRIES}/${VCS_MAX} 실패, 재시도..."
        sleep 5
    done
    vcs pull src 2>/dev/null || true
fi

# ROS2 환경 source 후 종속성 설치
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || \
    warn "rosdep 일부 실패 — 다음 빌드에서 확인 필요"

# colcon build — 일시적 실패에 대비해 최대 3회 재시도
COLCON_MAX_TRIES=3
COLCON_TRY=0
COLCON_OK=0
while [ "${COLCON_TRY}" -lt "${COLCON_MAX_TRIES}" ]; do
    COLCON_TRY=$((COLCON_TRY + 1))
    log "colcon build 시도 ${COLCON_TRY}/${COLCON_MAX_TRIES}..."
    if colcon build --symlink-install; then
        COLCON_OK=1
        ok "colcon build 성공"
        break
    fi
    warn "colcon build 시도 ${COLCON_TRY} 실패"
    if [ "${COLCON_TRY}" -lt "${COLCON_MAX_TRIES}" ]; then
        log "10초 후 재시도..."
        sleep 10
    fi
done

if [ "${COLCON_OK}" -ne 1 ]; then
    err ""
    err "============================================================"
    err " colcon build가 ${COLCON_MAX_TRIES}회 모두 실패했습니다."
    err "============================================================"
    err " 환경변수와 모든 의존성은 정상 설치되었습니다."
    err " ~/.bashrc도 이미 업데이트되어 있습니다."
    err ""
    err " 수동으로 다시 시도하려면:"
    err "   cd ${WS_DIR}"
    err "   source /opt/ros/${ROS_DISTRO}/setup.bash"
    err "   colcon build --symlink-install"
    err ""
    err " 새 터미널을 열고 'bash scripts/verify_env.sh'로 환경 상태를 확인하세요."
    err "============================================================"
    exit 1
fi

# =============================================================================
# 완료 — verify_env.sh 자동 실행
# =============================================================================
stage "✔ 설치 완료 — 환경 검증"

if [ -x "${SCRIPT_DIR}/verify_env.sh" ]; then
    bash "${SCRIPT_DIR}/verify_env.sh" || true
else
    warn "verify_env.sh를 찾을 수 없습니다 (${SCRIPT_DIR}/verify_env.sh)"
fi

cat <<EOF

============================================================
 ✔ 환경 셋업 완료
============================================================

다음 단계:

1) 새 터미널을 열거나 현재 터미널에서 환경 적용:
       source ~/.bashrc

2) 환경 검증 (위 출력과 동일하게 나오는지 확인):
       bash ${REPO_ROOT}/scripts/verify_env.sh

3) 첫 SITL 비행 (3개 터미널):
       # T1 — PX4↔ROS2 브리지
       MicroXRCEAgent udp4 -p 8888

       # T2 — PX4 SITL + Gazebo Harmonic
       cd \$PX4_AUTOPILOT_DIR && make px4_sitl gz_x500
       # (첫 빌드는 5~10분 소요)

       # T3 — ROS2에서 토픽 확인
       ros2 topic list | grep fmu

4) pxh 프롬프트가 보이면:
       pxh> commander takeoff
       (NAV_DLL_ACT=0가 airframe에 박혀있어 GCS 없이 바로 시동 가능)

주의:
  - PX4 의존성 설치 과정에서 사용자가 dialout 그룹에 추가됩니다.
    실제 Pixhawk USB 통신 단계에서는 한 번 로그아웃→재로그인이 필요합니다.

============================================================

EOF
