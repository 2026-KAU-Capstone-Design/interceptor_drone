# PX4–Gazebo–YOLO–Depth–IBVS Yaw 폐루프 통합 시험

> 상태: **Yaw 1축 폐루프 SITL 시험 성공 / 전진 접근 및 요격 미검증**  
> 시험 환경: Ubuntu 22.04, ROS 2 Humble, PX4 SITL, Gazebo, YOLOv8n, RGB·Depth 카메라  
> 시험일: 2026-10-16 (사용자 시험 로그 기준)

## 1. 목적 및 범위

고정익 정지 표적을 대상으로 RGB 카메라 → YOLO 검출 → Depth 거리 측정 → ROS 2 IBVS 제어 → PX4 SITL → Gazebo 기체 자세 변화 → RGB 카메라의 **Yaw 1축 폐루프**가 실제로 동작하는지 확인한다.

첫 시험에서 수평 이동·전진 접근·충돌/요격 기능은 비활성화하고, 목표 고도까지 이륙한 다음 Yaw 각속도만 제한해 제어한다. **실물 드론에서 실행하지 않는다.**

## 2. 입력 및 제어 구성

| 구성 요소 | 내용 |
|---|---|
| 표적 | Gazebo 정지 고정익 모델 `target_standard_vtol_static`, 클래스 ID `1` |
| YOLO 입력 | `/rgb_camera` |
| 표적 검출 결과 | `/target/detection` (`TargetDetection`) |
| 거리 측정 결과 | `/target/distance_stamped` (`TargetDistance`) |
| 측정값 연결 | RGB `header.stamp` 기준 검출·거리 메시지 매칭 및 유효성 확인 |
| PX4 상태 수신 | `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_local_position` |
| PX4 제어 발행 | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` |
| 제어 주기 | 10 Hz |
| 이륙 목표 | 시작 고도 대비 1.5 m |
| Yaw 명령 | 정규화 영상 수평 오차 `ex`에 비례, `Kp=0.4`, 최대 절댓값 `0.15 rad/s` |
| Yaw 시험 구간 | 최대 15초, 이후 자동 착륙 요청 |
| 이동 축 | Yaw 제어 구간에서 수평·수직 속도 명령 0 m/s |

**주의:** `OBSERVE`는 비행 명령을 발행하지 않는다. 비행은 SITL 전용 `--enable-flight` 옵션을 명시한 경우에만 가능하며 ARM·이륙을 포함한다. 테스트 당시 실제 기체가 아닌 PX4 SITL 연결을 확인했다.

## 3. 시험 절차 및 결과

### 3.1 첫 번째 실행 — 검출 유실

- 이륙 전 `ex=0.502`, `distance=5.71 m`로 검출·거리 측정 정상.
- 이륙 중 `ex=nan`, `distance=nan`으로 바뀌면서 Yaw 명령이 `0 rad/s`로 제한됨.
- 원인 분석 과정에서 초기 표적 배치 `(6, 0, 1)`과 카메라 시야가 맞지 않는 것을 발견함.
- 15초 후 LAND 명령 요청, PX4 `vehicle_land_detected.landed=true` 및 정지 상태 확인.
- **판정:** 폐루프 정렬 실패; 유효한 입력이 없을 때 Yaw 명령을 0으로 만드는 동작은 관찰됨.

### 3.2 두 번째 실행 — Yaw 폐루프 정렬 성공

- 표적을 Gazebo 기준 `(-2.3, 5.5, 1.5)` m에 배치해 가시성 확보.
- 이륙 및 `STATE=YAW_CONTROL` 진입, 검출·거리값 유지.
- `YAW_CONTROL` 시작 시 `ex=-0.188`, `yaw_cmd=-0.075 rad/s`.
- 약 5.1초 후 `ex=-0.002`, `yaw_cmd=-0.001 rad/s`.
- 이후에도 `ex`가 대체로 0 근처를 유지했고, 마지막 Yaw 제어 로그는 `ex=+0.005`, `yaw_cmd=+0.002 rad/s`.
- 약 15초 후 `LAND REQUEST: 15-second yaw test complete`, 사용자가 카메라를 통해 착륙 확인.
- **판정: 정지 표적에 대한 Yaw 1축 영상 기반 폐루프 정렬 성공.**

#### Yaw 제어 구간의 대표 관측값

| Yaw 제어 시작 후 대략 경과 | 수평 오차 `ex` | Yaw 명령 (rad/s) |
|---|---:|---:|
| 0초 | -0.188 | -0.075 |
| 1초 | -0.047 | -0.019 |
| 2초 | -0.021 | -0.008 |
| 3초 | -0.017 | -0.007 |
| 5.1초 | -0.002 | -0.001 |
| 14초 부근 | +0.005 | +0.002 |

정규화 수평 영상 오차의 **절댓값 감소율**(0초 대비 5.1초 시점):

`(0.188 - 0.002) / 0.188 × 100 ≈ 98.9%`

이 값은 해당 두 시점의 비교이며 전체 비행 구간의 평균 오차나 반복 시험 성공률을 의미하지 않는다. 거리값은 약 5.5~5.6 m 수준으로 측정됐으며, 거리 기반 전진 접근은 실행하지 않았다.

## 4. 코드 및 통합 상태

| 경로 | 용도 및 검증 범위 |
|---|---|
| `tools/yaw_closed_loop_sitl.py` | **실제 SITL Yaw 폐루프 비행 성공한 독립 시험 코드** |
| `ros2_ws/src/interceptor_control/interceptor_control/intercept.py` | 기존 요격 미션 코드 보존 + Yaw 시험 진입 모드 통합. **기본 관찰 모드 실행 확인**, 통합 파일의 실제 비행은 아직 재검증하지 않음 |
| `tools/yolo_live_detect.py` | `/target/detection` 발행 |
| `tools/depth_distance_node.py` | `/target/distance_stamped` 발행 |
| `tools/distance_safety_monitor.py` | 거리 유효성·최신성·수신 중단 확인용 읽기 전용 모니터 |
| `tools/target_data_validator.py` | 검출·거리 동일 촬영 시각 매칭 및 유효성 검증용 읽기 전용 노드 |

**현재 `intercept.py`는 새 센서 입력을 Yaw 시험 모드에 연결한 단계이며 기존 요격 상태 머신 전체를 새 토픽으로 전환한 것이 아니다.** 따라서 현재 코드를 완성된 자동 요격 시스템으로 평가해서는 안 된다.

### 시험 재현 관련 주의

- 필요한 선행 조건: Gazebo/PX4 SITL, Micro XRCE-DDS Agent(UDP 8888), RGB/Depth/Clock ROS 2 브리지, YOLO 노드, Depth 노드 구동 및 표적 가시성 확인.
- 독립 코드 기본 실행은 비행 없는 관찰 모드이다.

```bash
# 비행 없음: 데이터 및 Yaw 계산값 관찰
cd ~/Documents/interceptor_drone
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tools/yaw_closed_loop_sitl.py --ros-args -p use_sim_time:=true
```

- `--enable-flight`는 **Gazebo SITL에서만** 사용하고, 자동 ARM·이륙을 수행하므로 무인 실행 금지. 비행 중단 시 PX4/QGroundControl에서 착륙 명령을 내릴 수 있도록 준비해야 한다. 기존 `intercept.py`의 요격 미션 모드는 본 결과의 검증 대상이 아니다.

## 5. 한계 및 다음 작업

1. 정적 표적을 대상으로 한 단일 Yaw 축 제어만 검증했다. 전진 접근, 이동 표적 추적, 충돌·요격은 미검증이다.
2. 이륙 중 표적이 카메라 시야에서 사라질 수 있어, 시험 전 Gazebo 표적 배치와 카메라 가시성을 확인해야 한다.
3. 실제 Yaw 각도 추적 오차, 고도 유지 오차, 반복 시행 통계 등 정량 지표는 별도 로그 분석이 필요하다.
4. 통합 `intercept.py`의 실제 비행 모드는 관찰 모드 확인 이후 재시험하지 않았다.
5. 다음 단계는 안전 조건을 유지한 상태에서 Depth 거리를 사용하는 저속 전진 접근 제어를 단계적으로 통합하는 것이다.

## 6. 결론

**정지 고정익 표적에 대해 Gazebo–PX4 SITL–YOLO–Depth–ROS 2 IBVS를 연결한 첫 Yaw 1축 폐루프 정렬을 확인했다.** Yaw 제어 구간에서 수평 오차 절댓값이 `0.188`에서 약 5.1초 후 `0.002`로 감소했고, 제어 명령은 0에 가까워졌으며, 시험 종료 후 착륙을 카메라로 확인했다.
