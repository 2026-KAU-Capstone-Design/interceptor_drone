# Project Context — 요격 드론 캡스톤 (SW팀)

> **이 문서의 목적:** 새 협업자(사람 또는 AI)가 이 한 파일만 읽고 프로젝트를
> 이어서 작업할 수 있게 하는 단일 인계 문서. 코드 세부는 각 파일을 참조하되,
> 전체 그림과 "실수하기 쉬운 지점"을 여기에 모았다.

---

## 1. 프로젝트 개요

- **목표:** 요격 드론(interceptor drone) — 표적(풍선 → 실기체 형상)을 카메라로
  검출하고 자율 비행으로 요격. 4학년 캡스톤 종합설계.
- **팀:** HW팀 + SW팀. 이 저장소는 SW팀(3명) 담당.
- **진행 흐름:** Gazebo SITL 검증 → 실기체(Pixhawk + Jetson Orin Nano) 이식.
- **개발 단계 (Phase):**
  - Phase 0: 환경 셋업 ✅
  - Phase 1: ROS2/PX4 기초 ✅
  - Phase 2: 기본 SITL 비행 (offboard, waypoint) ✅ 진행중
  - Phase 3: 표적 인식 (카메라 + YOLO) ← 현재 위치
  - Phase 4: 요격 알고리즘
  - Phase 5: HW 스펙 인수 후 시뮬 적합화
  - Phase 6: 실기체 이식

---

## 2. 기술 스택 (전부 버전 핀 고정)

| 레이어 | 버전 | 비고 |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS | 팀 전원 듀얼부팅, 프로젝트 전용 |
| ROS2 | Humble Hawksbill | 22.04 공식, 2027까지 |
| PX4-Autopilot | v1.16.1 | `~/dev/PX4-Autopilot`, env `$PX4_AUTOPILOT_DIR` |
| Gazebo | Harmonic (LTS) | 2028까지 |
| ROS↔Gazebo | `ros-humble-ros-gzharmonic` | bridge/image 포함 |
| PX4↔ROS2 | Micro XRCE-DDS Agent v2.4.3 | `~/dev/Micro-XRCE-DDS-Agent` |
| 메시지 | `px4_msgs` (release/1.16) | ros2_ws/src에 vendoring |
| 표적검출 | YOLOv8n (Ultralytics) + PyTorch | GPU 있으면 CUDA, 없으면 CPU 자동 |
| 실기체(예정) | Pixhawk(모델 미정) + Jetson Orin Nano | aarch64, JetPack 6.x |

상세 근거: [version_matrix.md](version_matrix.md)

---

## 3. 저장소

- **GitHub:** https://github.com/2026-KAU-Capstone-Design/interceptor_drone
- **로컬:** `~/Documents/interceptor_drone`
- **브랜치:** `main`(안정) / `dev`(통합) / `feature/<이름>-<설명>`
- **워크플로:** feature 브랜치 → PR → dev 머지 (리뷰 1명) → main
- **주의:** `PX4-Autopilot`, `Micro-XRCE-DDS-Agent` 본체는 git에 미포함.
  `scripts/setup_dev_env.sh` 가 `~/dev/` 아래로 clone + 패치.

---

## 4. 핵심 패키지: `interceptor_control` (ament_python)

경로: `ros2_ws/src/interceptor_control/`

### 실행 가능한 노드 (`ros2 run interceptor_control <노드>`)
| 노드 | 파일 | 역할 |
|---|---|---|
| `offboard_hover` | `interceptor_control/offboard_hover.py` | **베이스 노드** — Offboard 호버 + 풍속/자세 안전 모니터. 모든 미션의 템플릿 |
| `hover_land` | `.../missions/mission1_hover/hover_land.py` | M1: 호버 + 착륙 오차 |
| `point_nav` | `.../missions/mission2_nav/point_nav.py` | M2: A→B 이동 |
| `image_probe` | `.../perception/image_probe.py` | 카메라 이미지 수신 검증 (fps/해상도/검붉은비율) |
| `vtol_transition_test` | `.../vtol_transition_test.py` | quad tailsitter 천이비행 자동 검증 |

### launch 파일
- `launch/hover.launch.py` — 호버 미션
- `launch/camera_bridge.launch.py` — gz↔ROS2 카메라 브리지 (ros_gz_image)

### config
- `config/mission_params.yaml` — 풍속 경고/위험 기준, 자세 복원 타임아웃 등

---

## 5. ⚠️ 실수하기 쉬운 지점 (반드시 숙지)

1. **PX4 v1.16 토픽명 변경:** `vehicle_status` → **`/fmu/out/vehicle_status_v1`**
   (메시지 타입은 `px4_msgs/msg/VehicleStatus` 그대로). 다른 토픽은 변경 없음.
   코드/문서의 토픽명을 믿지 말고 `ros2 topic list` 로 실측 확인.

2. **NED 좌표계:** z축은 **아래가 양수**. 5m 고도 = `z = -5.0`.

3. **Offboard 진입 시퀀스:** setpoint를 1초 먼저 흘려보낸 뒤 ARM + OFFBOARD
   전환, 그 후에도 **10Hz로 OffboardControlMode + TrajectorySetpoint 계속 발행**.
   끊기면 PX4가 fail-safe(자동 착륙).

4. **PX4 SITL용 QoS:** `BEST_EFFORT` + `TRANSIENT_LOCAL` + `KEEP_LAST` depth=1.
   (`offboard_hover.py` 의 `PX4_QOS` 참조)

5. **카메라 subscriber QoS:** ros_gz image_bridge는 RELIABLE로 publish. 단순
   `create_subscription(..., 10)` 도 받아지지만, 안 받아지면 `qos_profile_sensor_data`
   (BEST_EFFORT) 로 맞출 것.

6. **카메라는 전용 airframe 필요:** 일반 `gz_x500`/`gz_quadtailsitter` 엔 카메라
   없음. **`gz_x500_mono_cam`** 사용해야 `/camera/image` 가 나옴.

7. **Gazebo Harmonic 바람:** `<wind>` 태그만으론 동작 안 함.
   `gz-sim-wind-effects-system` 플러그인 + 각 link에 `<enable_wind>true</enable_wind>`
   둘 다 있어야 바람 force가 적용됨.

8. **GCS 없이 시동:** PX4 v1.16 SITL은 기본적으로 GCS 연결을 시동 조건으로 봄.
   `setup_dev_env.sh` 가 X500 airframe에 `NAV_DLL_ACT=0` 패치하여 해결.
   실기체 단계에선 1(Hold) 또는 2(Return)로 복원 필요.

9. **VTOL 천이 (pxh 간단 제어):** `commander transition` (MC↔FW 토글).
   프로그래밍: `VEHICLE_CMD_DO_VTOL_TRANSITION(3000)` param1=4(FW)/3(MC).
   상태 감지: `vehicle_status.vehicle_type`(1=MC, 2=FW), `in_transition_mode`.

10. **SITL 종료 습관:** `pkill -9 gz; pkill -9 ruby; pkill -9 px4`.
    안 하면 좀비 프로세스로 "Waiting for Gazebo world..." 무한 루프.

---

## 6. 커스텀 Gazebo 월드 (`simulation/worlds/`)

| 월드 | 풍속 | Fuel 의존 | 용도 |
|---|---|---|---|
| `simple_windy.sdf` | ~5.4 m/s | ❌ | 유풍 미션 (권장) |
| `simple_storm.sdf` | ~11.7 m/s | ❌ | 안전장치 검증 (권장) |
| `simple_windy_balloon.sdf` | ~5.4 m/s | ❌ | 유풍 + 검붉은풍선 표적 |
| `baylands_windy.sdf` | ~5.4 m/s | ✅ | 지형 시각화 (무거움) |
| `baylands_storm.sdf` | ~11.7 m/s | ✅ | 지형 + 강풍 |

> `setup_dev_env.sh` 가 이 sdf들을 PX4 worlds 폴더로 자동 복사.
> 수동: `cp simulation/worlds/*.sdf ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/`

---

## 7. SITL 실행 패턴 (터미널 분리)

```bash
# 좀비 정리 (항상 먼저)
pkill -9 gz; pkill -9 ruby; pkill -9 px4

# T1 — PX4↔ROS2 브리지
MicroXRCEAgent udp4 -p 8888

# T2 — PX4 SITL (월드 + airframe 선택)
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=<월드> make px4_sitl <airframe>
#   기본 비행:   PX4_GZ_WORLD=simple_windy        airframe=gz_x500
#   카메라:      PX4_GZ_WORLD=simple_windy_balloon airframe=gz_x500_mono_cam
#   VTOL:        (월드 생략 가능)                  airframe=gz_quadtailsitter

# T3 — 카메라 쓸 때만
ros2 launch interceptor_control camera_bridge.launch.py

# T4 — 제어/검증 노드
ros2 run interceptor_control <노드>
```

워크스페이스 빌드:
```bash
cd ~/Documents/interceptor_drone/ros2_ws
colcon build --packages-select interceptor_control --symlink-install
source install/setup.bash
```

---

## 8. 환경 구축 스크립트 (`scripts/`)

| 스크립트 | 역할 |
|---|---|
| `setup_dev_env.sh` | 전체 환경 자동 설치 (ROS2+PX4+Gazebo+XRCE+ws) + PX4 패치. 멱등. |
| `setup_perception.sh` | YOLOv8n + PyTorch (GPU/CPU 자동 감지) |
| `verify_env.sh` | 환경 검증 (팀원 간 출력 비교용) |

신규 팀원/머신 온보딩:
```bash
git clone https://github.com/2026-KAU-Capstone-Design/interceptor_drone.git ~/Documents/interceptor_drone
cd ~/Documents/interceptor_drone
bash scripts/setup_dev_env.sh        # 30분~1시간
bash scripts/setup_perception.sh     # 5~15분 (YOLO 필요시)
```

---

## 9. 카메라 이미지 파이프라인 (Phase 3 핵심)

```
Gazebo 카메라 센서 (x500_mono_cam, imager, 1280x960 30Hz)
  → gz transport: /world/<world>/model/<model>/link/camera_link/sensor/imager/image
  → ros_gz_image image_bridge (camera_bridge.launch.py)
  → ROS2: /camera/image (sensor_msgs/Image)
  → 구독 노드 (image_probe, 또는 YOLO 노드)
```

상세: [camera_pipeline.md](camera_pipeline.md)

---

## 10. 현재 상태 & 다음 단계

**완료:**
- 환경 셋업, 팀원 온보딩 자동화
- Offboard 호버 + 풍속/자세 안전 모니터
- 미션 1(호버), 2(A→B) 노드
- 카메라 이미지 파이프라인 (gz_x500_mono_cam → /camera/image)
- 검붉은풍선 표적 월드
- VTOL quad tailsitter 천이비행 검증 노드

**다음 (Phase 3 → 4):**
1. **YOLO 추론 노드** — `/camera/image` 구독 → YOLOv8n → bbox → `/target/bbox`
2. **3D 위치 추정** — bbox + camera_info + 드론 자세 → `/target/position` (NED)
3. **요격 알고리즘** — `/target/position` → offboard setpoint 갱신 → 추격
4. **풍선 표적 동적화** — 바람에 흔들리는 tethered balloon (작업 중, stash 보존)

---

## 11. 알려진 정리 필요 항목 (technical debt)

- `ros2_ws/missions/` 에 팀원이 만든 중복/오위치 파일 존재
  (`hover_land.py` 가 여러 곳에 중복). 단일 위치로 정리 필요.
- `ros2_ws/src/interceptor_control/interceptor_control/missions/mission1_hover/rosbag/`
  의 `metadata.yaml` 이 git에 커밋됨 → rosbag은 gitignore 대상이어야 함.
- 팀원별 미션 폴더 구조 미확정 (user1/2/3 분리 검토했으나 보류).

---

## 12. 참고 문서 인덱스

| 파일 | 내용 |
|---|---|
| `README.md` | 빠른 시작, git 워크플로 |
| `docs/version_matrix.md` | 버전 핀 + 결정 근거 |
| `docs/tutorial_offboard.md` | Offboard 제어 기초 (필독) |
| `docs/camera_pipeline.md` | 카메라 파이프라인 |
| `docs/px4_params.md` | PX4 파라미터 변경 이력 |
| `docs/project_context.md` | 이 문서 (전체 인계) |
