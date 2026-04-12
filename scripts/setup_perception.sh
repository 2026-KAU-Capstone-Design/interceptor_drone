#!/usr/bin/env bash
# =============================================================================
# Interceptor Drone — Perception Environment (YOLOv8n + PyTorch)
# =============================================================================
# 기본 환경(setup_dev_env.sh) 설치 후 실행.
# PyTorch(GPU/CPU 자동 감지) + Ultralytics YOLOv8n 을 설치한다.
#
# 사용법:
#   bash scripts/setup_perception.sh
#
# 사전 조건:
#   - setup_dev_env.sh 완료
#   - (선택) NVIDIA GPU + 드라이버 설치됨 → GPU 추론 활성화
#   - 인터넷 연결 (PyTorch ~2GB, Ultralytics ~50MB 다운로드)
# =============================================================================

set -eo pipefail

# ---------- 색상 출력 ----------
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
log()   { echo -e "${BLU}[INFO]${NC} $*"; }
ok()    { echo -e "${GRN}[ OK ]${NC} $*"; }
warn()  { echo -e "${YLW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[FAIL]${NC} $*" >&2; }
stage() { echo -e "\n${BLU}========== $* ==========${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# =============================================================================
# 1/4 GPU 감지 및 PyTorch 설치 전략 결정
# =============================================================================
stage "1/4 GPU 감지"

if command -v nvidia-smi >/dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    DRIVER_VER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
    CUDA_MAX=$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[\d.]+' || echo "unknown")
    ok "NVIDIA GPU: ${GPU_NAME}"
    ok "드라이버: ${DRIVER_VER}, 지원 CUDA: ${CUDA_MAX}"

    # PyTorch CUDA 12.4 바이너리는 드라이버가 CUDA ≥12.4를 지원하면 동작 (하위 호환)
    TORCH_INDEX="https://download.pytorch.org/whl/cu124"
    DEVICE_TAG="cuda"
else
    warn "NVIDIA GPU 미감지 — CPU 전용 PyTorch를 설치합니다."
    warn "추론 속도가 느릴 수 있지만 기능은 동일합니다."
    TORCH_INDEX="https://download.pytorch.org/whl/cpu"
    DEVICE_TAG="cpu"
fi

# =============================================================================
# 2/4 PyTorch 설치
# =============================================================================
stage "2/4 PyTorch 설치 (${DEVICE_TAG})"

log "PyTorch + torchvision 설치 중 (최대 2GB 다운로드, 시간이 걸립니다)..."
pip3 install torch torchvision --index-url "${TORCH_INDEX}"
ok "PyTorch 설치 완료"

# =============================================================================
# 3/4 Ultralytics (YOLOv8) + 의존성 설치
# =============================================================================
stage "3/4 Ultralytics YOLOv8 설치"

pip3 install ultralytics

# numpy 1.x/2.x 호환성 문제 예방 — ultralytics가 설치한 numpy로 통일
# (시스템 cv2가 numpy 1.x로 빌드된 경우 충돌할 수 있음)
pip3 install --force-reinstall numpy 2>/dev/null || true

ok "Ultralytics 설치 완료"

# =============================================================================
# 4/4 YOLOv8n 모델 다운로드 + 검증
# =============================================================================
stage "4/4 YOLOv8n 모델 다운로드 및 검증"

log "YOLOv8n 모델 다운로드 중 (~6MB)..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"

log "추론 테스트 중..."
python3 << 'PYEOF'
import torch
import numpy as np

print("  ┌──────────────────────────────────────────┐")
print(f"  │ PyTorch        : {torch.__version__:<25s}│")
print(f"  │ CUDA available : {str(torch.cuda.is_available()):<25s}│")
if torch.cuda.is_available():
    print(f"  │ GPU            : {torch.cuda.get_device_name(0):<25s}│")
    vram_gb = torch.cuda.get_device_properties(0).total_memory / 1024**3
    print(f"  │ VRAM           : {f'{vram_gb:.1f} GB':<25s}│")

from ultralytics import YOLO
model = YOLO("yolov8n.pt")

# 검정 이미지로 추론 테스트 (인터넷 불필요)
dummy = np.zeros((640, 640, 3), dtype=np.uint8)
results = model.predict(source=dummy, verbose=False)
det_count = len(results[0].boxes)

print(f"  │ YOLOv8n        : {'loaded OK':<25s}│")
print(f"  │ Test inference  : {f'{det_count} detections (black img)':<25s}│")
print("  └──────────────────────────────────────────┘")
PYEOF

ok "Perception 환경 설치 완료"

cat <<EOF

============================================================
 ✔ YOLOv8n 환경 준비 완료
============================================================

사용법 (Python):

  from ultralytics import YOLO
  model = YOLO("yolov8n.pt")
  results = model.predict(source="image.jpg")

ROS2 노드에서 사용할 때:
  - 카메라 토픽 구독 → numpy 변환 → model.predict() → 결과 publish
  - interceptor_perception 패키지로 구현 예정 (Phase 3)

참고:
  - GPU가 있으면 자동으로 GPU 추론 (별도 설정 불필요)
  - Jetson Orin Nano에서는 JetPack의 PyTorch를 사용 (동일 코드 동작)
  - 모델 파일 위치: ./yolov8n.pt 또는 ~/.config/Ultralytics/

============================================================

EOF
