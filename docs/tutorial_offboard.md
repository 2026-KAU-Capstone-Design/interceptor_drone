# Tutorial — PX4 Offboard 제어 기초

이 문서는 **PX4 + ROS2 Offboard 제어**를 처음 접하는 팀원을 위한 튜토리얼입니다.
미션 코드를 작성하기 전에 반드시 이 문서를 읽고, 직접 따라해보세요.

---

## 1. Offboard 모드란?

PX4에는 여러 비행 모드가 있습니다:
- **Manual** — 사람이 RC로 직접 조종
- **Position** — 사람 입력 + PX4가 GPS로 위치 유지
- **Mission** — 미리 짜둔 waypoint 자동 비행
- **Offboard** — **외부 컴퓨터(ROS2 노드)가 매 순간 목표 좌표를 전송**

**Offboard가 우리 프로젝트의 핵심**입니다. "어디로 가라"는 고수준 결정은 우리 ROS2 노드가,
"어떻게 모터를 돌릴지"는 PX4가 담당합니다.

---

## 2. 핵심 개념 — 3-메시지 패턴

Offboard 제어에 필요한 메시지는 **딱 3개**입니다:

### (1) OffboardControlMode — "어떤 종류의 명령을 보낼지 선언"
```python
msg = OffboardControlMode()
msg.position = True      # 위치 setpoint를 보낼 것임
msg.velocity = False
msg.acceleration = False
msg.attitude = False
msg.body_rate = False
```
**10Hz 이상으로 계속 보내야 합니다.** 끊기면 PX4가 fail-safe(자동 착륙)로 들어갑니다.

### (2) TrajectorySetpoint — "실제 목표 좌표"
```python
msg = TrajectorySetpoint()
msg.position = [0.0, 0.0, -5.0]  # x, y, z (NED 좌표)
msg.yaw = 0.0                     # 방향 (라디안)
```
**NED 좌표 주의!** z축은 **아래가 양수**, 위가 음수입니다.
- `z = -5.0` → 지면에서 5m **위**
- `z = 5.0` → 지면에서 5m **아래** (지하!)

### (3) VehicleCommand — "시동, 모드 전환 등 이벤트성 명령"
```python
# ARM (시동)
msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
msg.param1 = 1.0  # 1=arm, 0=disarm

# OFFBOARD 모드 전환
msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
msg.param1 = 1.0
msg.param2 = 6.0  # 6 = OFFBOARD

# 착륙
msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND

# 자동 귀환
msg.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
```

---

## 3. Offboard 진입 순서 (암기!)

```
시간 →
[t=0~1s]   OffboardControlMode + TrajectorySetpoint 를 10Hz로 전송 (아직 모드 전환 X)
[t=1s]     VehicleCommand(OFFBOARD 모드 전환) 전송
[t=1s]     VehicleCommand(ARM) 전송
[t>1s]     ★ 계속 10Hz로 OffboardControlMode + TrajectorySetpoint 전송 ★
```

**왜 미리 1초 보내야 하나?** PX4는 setpoint가 안정적으로 들어오는지 확인한 후에야
Offboard 진입을 허용합니다. 비어있는 상태로 모드 전환하면 거부됩니다.

**왜 계속 보내야 하나?** Setpoint 스트림이 0.5초 이상 끊기면 PX4가
"외부 통신 끊김"으로 판단해서 fail-safe에 들어갑니다.

---

## 4. 토픽 이름 (PX4 v1.16 기준)

| 토픽 | 방향 | 메시지 타입 |
|---|---|---|
| `/fmu/in/offboard_control_mode` | ROS2 → PX4 | `OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | ROS2 → PX4 | `TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | ROS2 → PX4 | `VehicleCommand` |
| `/fmu/out/vehicle_local_position` | PX4 → ROS2 | `VehicleLocalPosition` |
| `/fmu/out/vehicle_status_v1` | PX4 → ROS2 | `VehicleStatus` |
| `/fmu/out/vehicle_odometry` | PX4 → ROS2 | `VehicleOdometry` |

**주의:** PX4 v1.16에서 `vehicle_status` 가 `vehicle_status_v1` 로 이름 변경되었습니다.

---

## 5. 실습 — 직접 따라하기

### 5.1 SITL 띄우기 (3개 터미널)

```bash
# T1 — 브리지
MicroXRCEAgent udp4 -p 8888

# T2 — PX4 SITL (무풍)
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands make px4_sitl gz_x500

# T3 — 토픽 확인
ros2 topic list | grep fmu
```

### 5.2 워크스페이스 빌드

```bash
cd ~/Documents/interceptor_drone/ros2_ws
colcon build --packages-select interceptor_control --symlink-install
source install/setup.bash
```

### 5.3 Offboard Hover 실행 (T3에서)

```bash
ros2 run interceptor_control offboard_hover
```

Gazebo에서 드론이:
1. 시동 걸리고
2. 5m까지 상승
3. 30초 호버
4. 자동 착륙

이 과정이 보이면 성공입니다.

### 5.4 파라미터 변경 테스트

```bash
# 10m 고도, 60초 호버
ros2 run interceptor_control offboard_hover --ros-args \
  -p target_altitude:=-10.0 \
  -p hover_duration:=60.0
```

### 5.5 유풍 환경 테스트

```bash
# T2에서 SITL 종료 (Ctrl+C) 후:
# 커스텀 월드를 PX4에 복사 (최초 1회)
cp ~/Documents/interceptor_drone/simulation/worlds/*.sdf \
   ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/

# 보통풍 (5.4m/s)
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands_windy make px4_sitl gz_x500

# 또는 강풍 (11.7m/s)
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands_storm make px4_sitl gz_x500
```

T3에서 다시 `ros2 run interceptor_control offboard_hover` 실행.
강풍에서 `[WIND WARNING]`, `[WIND CRITICAL]` 로그가 뜨는지 확인하세요.

---

## 6. 코드 구조 이해

`offboard_hover.py` 의 핵심 구조:

```
class OffboardHover(Node)
    │
    ├── __init__()          파라미터, pub/sub, 타이머 설정
    │
    ├── _cb_local_pos()     위치 수신 → 풍속 추정
    ├── _cb_status()        비행 상태 수신
    ├── _cb_odometry()      쿼터니언 → 기울기 계산 → 자세 안전 체크
    │
    ├── _check_wind_safety()      풍속 기반: 8m/s 경고, 15m/s RTL
    ├── _check_attitude_safety()  기울기 > 30°, 2초 미복원 → 비상착륙
    │
    ├── publish_offboard_mode()   OffboardControlMode 발행
    ├── publish_setpoint()        TrajectorySetpoint 발행
    ├── send_command()            VehicleCommand 발행
    │
    ├── _control_loop()     ★ 10Hz 메인 루프 (상태 머신) ★
    │   ├── IDLE → PREFLIGHT   setpoint 1초 사전 전송
    │   ├── PREFLIGHT → ARMING  ARM + OFFBOARD
    │   ├── ARMING → TAKEOFF   목표 고도 상승
    │   ├── TAKEOFF → HOVER    고도 도달 후 호버
    │   ├── HOVER → LANDING    호버 시간 완료
    │   ├── RTL                 강풍 시 자동 귀환
    │   └── LANDING → DONE     착륙 명령 후 종료
    │
    └── _print_hover_stats() / _print_landing_stats()  오차 출력
```

### 미션 코드 작성 시 수정할 부분

1. **`FlightState`** — 필요한 상태 추가 (예: `MOVE_TO_B`, `HOLD_B`)
2. **`_control_loop()`** — 새 상태에 대한 setpoint 로직 추가
3. **파라미터** — 필요한 좌표/시간 추가
4. **통계 출력** — 미션별 측정 항목 추가

### 수정하지 말아야 할 부분

- PX4 QoS 설정 (`PX4_QOS`)
- `publish_offboard_mode()` — 항상 10Hz로 호출되어야 함
- 안전 모니터 (`_check_wind_safety`, `_check_attitude_safety`)
- PX4 command 헬퍼 함수들

---

## 7. 자주 하는 실수

| 실수 | 증상 | 해결 |
|---|---|---|
| setpoint를 안 보내고 ARM | Offboard 진입 거부 | 1초 사전 전송 필수 |
| z를 양수로 설정 | 드론이 땅속으로 | NED: z 음수가 위 |
| setpoint 발행 멈춤 | 0.5초 후 fail-safe | 모든 상태에서 항상 발행 |
| `vehicle_status` 토픽명 | "topic not found" | v1.16은 `vehicle_status_v1` |
| colcon build 안 하고 실행 | "package not found" | 코드 수정 후 반드시 빌드 |
| workspace source 안 함 | "package not found" | `source install/setup.bash` |

---

## 8. 참고 자료

- [PX4 ROS2 Offboard Control](https://docs.px4.io/main/en/ros2/offboard_control.html)
- [px4_msgs 메시지 정의](https://github.com/PX4/px4_msgs/tree/release/1.16/msg)
- [ROS2 Humble Tutorial](https://docs.ros.org/en/humble/Tutorials.html)
- 우리 프로젝트의 `docs/version_matrix.md` — 핀 버전 확인
- 우리 프로젝트의 `docs/px4_params.md` — PX4 파라미터 변경 이력
