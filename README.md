# Interceptor Drone — Capstone SW

요격 드론 캡스톤 프로젝트의 SW 레포지토리입니다.
시뮬레이션(Gazebo Harmonic + PX4 SITL)에서 검증한 코드를 그대로
실기체(Pixhawk + Jetson Orin Nano)로 이식하는 것이 목표입니다.

---

## 기술 스택

| 레이어 | 버전 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS2 | Humble Hawksbill |
| 자율비행 | PX4-Autopilot **v1.16.1** |
| 시뮬레이터 | Gazebo **Harmonic** (LTS, 2028년까지 지원) |
| ROS↔Gazebo | `ros-humble-ros-gzharmonic` |
| PX4↔ROS2 | Micro XRCE-DDS Agent **v2.4.3** |
| 메시지 | `px4_msgs` (release/1.16) |

전체 결정 근거는 [docs/version_matrix.md](docs/version_matrix.md) 참고.

---

## 빠른 시작 (팀원용)

깨끗한 Ubuntu 22.04.5 LTS에서 다음 세 줄이면 끝납니다.

```bash
git clone <repo-url> ~/Documents/interceptor_drone
cd ~/Documents/interceptor_drone
bash scripts/setup_dev_env.sh
```

설치는 약 30분~1시간 걸립니다 (인터넷 속도와 CPU에 따라). 끝나면 새 터미널을 열고:

```bash
bash scripts/verify_env.sh
```

세 명의 출력 결과(`[ System ]` ~ `[ Workspace ]`)가 모두 동일해야 합니다.
GPU 항목은 달라도 무방합니다.

---

## SITL 첫 비행

세 개의 터미널에서 차례로:

```bash
# T1 — PX4↔ROS2 브리지
MicroXRCEAgent udp4 -p 8888

# T2 — PX4 SITL + Gazebo Harmonic
cd $PX4_AUTOPILOT_DIR && make px4_sitl gz_x500

# T3 — ROS2에서 토픽 확인
ros2 topic list | grep fmu
```

T3에서 `/fmu/out/vehicle_status` 같은 토픽이 보이면 환경 구성 성공.

---

## 디렉터리 구조

```
.
├── README.md               ← 이 파일
├── docs/
│   └── version_matrix.md   ← 모든 외부 컴포넌트 핀 버전
├── scripts/
│   ├── setup_dev_env.sh    ← 팀원 환경 자동 설치 (멱등)
│   └── verify_env.sh       ← 환경 검증 (출력 비교용)
├── repos/
│   └── dev.repos           ← vcstool 외부 저장소 핀
└── ros2_ws/                ← ROS2 워크스페이스
    └── src/                ← 패키지 (build/install/log은 .gitignore)
```

`PX4-Autopilot`, `Micro-XRCE-DDS-Agent` 본체는 git에 포함하지 않습니다.
`scripts/setup_dev_env.sh` 가 `~/dev/` 아래로 clone합니다.

---

## Git 워크플로우

### 브랜치 전략
- **`main`** — 항상 빌드되고 SITL이 도는 안정 브랜치. PR 머지로만 변경.
- **`dev`** — 통합 브랜치. feature 브랜치들이 머지되는 곳.
- **`feature/<author>-<short-desc>`** — 개인 작업 브랜치. 예: `feature/myname-offboard-square`

### 기본 흐름
```bash
git checkout dev
git pull origin dev
git checkout -b feature/myname-새기능
# ... 작업 ...
git add .
git commit -m "feat: 새 기능 설명"
git push -u origin feature/myname-새기능
# GitHub에서 dev 브랜치로 PR
```

### 규칙
- PR은 다른 팀원 1명 이상의 리뷰 후 머지.
- `main` 직접 push 금지 (보호 규칙으로 막을 것).
- 커밋 메시지 prefix: `feat:`, `fix:`, `sim:`, `docs:`, `chore:`, `refactor:`

---

## 팀

- SW Lead: 성소민
- Member 2: 이주연
- Member 3: 정낙훈
