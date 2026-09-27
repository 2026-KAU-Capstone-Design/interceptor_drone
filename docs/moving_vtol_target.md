# Moving Standard VTOL Target

Gazebo 환경에서 `standard_vtol` 형상의 표적을 지정된 경로와 속도로 이동시키기 위한 시뮬레이션 환경입니다.

YOLO 기반 표적 인식 및 추적 제어 테스트를 위해 제작하였으며, PX4 SITL의 `x500_mono_cam`과 함께 사용할 수 있습니다.

---

## 1. 주요 기능

- PX4 `standard_vtol` 형상의 표적 사용
- 표적 이동 속도 설정 가능 (`m/s`)
- 표적 고도 설정 가능
- 이동 경로 크기 및 중심 위치 설정 가능
- 이동 방향에 맞춰 기체 yaw 자동 변경
- 다음 3가지 이동 경로 지원
  - Circle
  - Ellipse
  - Rectangle
- Gazebo Transport Python API를 이용한 pose 제어
- 기본 pose update rate: 30 Hz
- 50 m/s (180 km/h) 이동 테스트 완료

---

## 2. 파일 구성

```text
interceptor_drone/
├── simulation/
│   └── worlds/
│       └── simple_moving_vtol_target.sdf
│
├── tools/
│   └── move_standard_vtol_target.py
│
└── docs/
    └── moving_vtol_target.md
```

### `simple_moving_vtol_target.sdf`

`x500_mono_cam`과 이동 표적을 함께 사용하기 위한 Gazebo world입니다.

표적은 `target_standard_vtol`이라는 이름으로 생성됩니다.

### `move_standard_vtol_target.py`

Gazebo의 `/set_pose` 서비스를 이용하여 표적의 위치와 방향을 연속적으로 변경합니다.

지원 경로:

- `circle`
- `ellipse`
- `rectangle`

---

## 3. 테스트 환경

현재 다음 환경에서 동작을 확인했습니다.

- Ubuntu 22.04
- ROS 2 Humble
- PX4 SITL
- Gazebo
- `gz-transport13`
- `gz-msgs10`
- `python3-gz-transport13`
- `python3-gz-msgs10`

설치 여부는 다음 명령으로 확인할 수 있습니다.

```bash
dpkg -l | grep -E 'gz-transport|gz-msgs'
```

---

## 4. Target 모델 준비

PX4의 기본 `standard_vtol`은 동적 비행체 모델이기 때문에 공중에 배치만 하면 중력의 영향을 받아 떨어집니다.

따라서 이동 표적용 복사본을 생성하고 static 모델로 변경합니다.

### 4.1 standard_vtol 복사

```bash
cd ~/dev/PX4-Autopilot/Tools/simulation/gz/models

cp -r standard_vtol target_standard_vtol
```

복사가 완료되면 다음 경로가 생성됩니다.

```text
~/dev/PX4-Autopilot/Tools/simulation/gz/models/target_standard_vtol
```

### 4.2 target 모델을 static으로 변경

다음 파일을 엽니다.

```bash
nano ~/dev/PX4-Autopilot/Tools/simulation/gz/models/target_standard_vtol/model.sdf
```

`<model>` 바로 아래에 다음을 추가합니다.

```xml
<static>true</static>
```

예:

```xml
<sdf version='1.10'>
  <model name='standard_vtol'>
    <static>true</static>
    <pose>0 0 0.246 0 0 0</pose>
```

이 표적은 PX4 비행제어로 비행시키는 모델이 아니라, 지정한 trajectory를 따라 강제로 이동시키는 kinematic target으로 사용합니다.

---

## 5. World 설치

프로젝트에 있는 world 파일을 PX4 Gazebo world 디렉터리로 복사합니다.

```bash
cp ~/Documents/interceptor_drone/simulation/worlds/simple_moving_vtol_target.sdf \
   ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/
```

---

## 6. PX4 + Gazebo 실행

PX4 디렉터리로 이동합니다.

```bash
cd ~/dev/PX4-Autopilot
```

Gazebo가 PX4 model과 world를 찾을 수 있도록 resource path를 설정합니다.

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/dev/PX4-Autopilot/Tools/simulation/gz/models:$HOME/dev/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
```

다음 명령으로 `x500_mono_cam`과 이동 표적 world를 실행합니다.

```bash
PX4_GZ_WORLD=simple_moving_vtol_target make px4_sitl gz_x500_mono_cam
```

정상적으로 실행되면 Gazebo에서 다음 두 모델을 확인할 수 있습니다.

```text
x500_mono_cam
target_standard_vtol
```

---

## 7. 이동 표적 실행

새 터미널을 열고 프로젝트 디렉터리로 이동합니다.

```bash
cd ~/Documents/interceptor_drone
```

### 7.1 원형 경로

```bash
python3 tools/move_standard_vtol_target.py \
  --path circle \
  --speed 50 \
  --radius 200 \
  --center-x 60 \
  --center-y 0 \
  --z 10 \
  --rate 30
```

위 설정에서는 표적이 다음 조건으로 이동합니다.

```text
경로       : 원형
속도       : 50 m/s (180 km/h)
반지름     : 200 m
경로 중심  : (60, 0)
고도       : 10 m
갱신 주기  : 30 Hz
```

---

### 7.2 타원형 경로

```bash
python3 tools/move_standard_vtol_target.py \
  --path ellipse \
  --speed 50 \
  --a 200 \
  --b 100 \
  --center-x 60 \
  --center-y 0 \
  --z 10 \
  --rate 30
```

`a`, `b`는 각각 타원의 장반경과 단반경입니다.

타원 경로는 누적 경로 길이를 기준으로 위치를 계산하여 경로상의 이동 속도가 가능한 일정하게 유지되도록 구현되어 있습니다.

---

### 7.3 직사각형 경로

```bash
python3 tools/move_standard_vtol_target.py \
  --path rectangle \
  --speed 50 \
  --width 300 \
  --height 150 \
  --center-x 60 \
  --center-y 0 \
  --z 10 \
  --rate 30
```

위 설정에서는 다음 크기의 직사각형 경로를 이동합니다.

```text
가로 : 300 m
세로 : 150 m
```

> `rectangle` 경로는 모서리에서 기체의 진행 방향이 즉시 90° 변경됩니다.
> 따라서 실제 고정익의 비행역학을 재현하기 위한 경로보다는 추적 알고리즘 테스트용 경로로 사용합니다.

---

## 8. 주요 실행 옵션

| 옵션 | 설명 | 예시 |
|---|---|---|
| `--path` | 이동 경로 | `circle` |
| `--speed` | 표적 이동 속도 [m/s] | `50` |
| `--rate` | pose update rate [Hz] | `30` |
| `--z` | 표적 고도 [m] | `10` |
| `--center-x` | 경로 중심 X 좌표 [m] | `60` |
| `--center-y` | 경로 중심 Y 좌표 [m] | `0` |
| `--radius` | 원형 경로 반지름 [m] | `200` |
| `--a` | 타원 장반경 [m] | `200` |
| `--b` | 타원 단반경 [m] | `100` |
| `--width` | 직사각형 가로 길이 [m] | `300` |
| `--height` | 직사각형 세로 길이 [m] | `150` |
| `--yaw-offset` | 기체 방향 보정 [deg] | `90` |

---

## 9. 동작 방식

표적 이동 프로그램은 Gazebo의 다음 서비스를 사용합니다.

```text
/world/simple_moving_vtol_target/set_pose
```

초기 구현에서는 Python에서 매 update마다 `gz service` 명령을 subprocess로 실행하였으나, 이 방식에서는 약 3~4 Hz 수준의 낮은 update rate가 측정되었습니다.

따라서 현재 구현에서는 Gazebo Transport Python API를 직접 사용합니다.

```python
from gz.transport13 import Node
```

프로그램 시작 시 Transport Node를 한 번 생성하고 동일한 Node를 계속 재사용하여 pose 요청을 전송합니다.

구조는 다음과 같습니다.

```text
move_standard_vtol_target.py
          │
          ├── 경로 계산
          │
          ├── 위치 (x, y, z) 계산
          │
          ├── 진행 방향 yaw 계산
          │
          └── Gazebo Transport
                    │
                    ▼
              /set_pose
                    │
                    ▼
          target_standard_vtol
```

---

## 10. 속도와 Update Rate

표적의 이동 거리는 실제 경과 시간을 기준으로 계산합니다.

예를 들어 표적 속도가 50 m/s일 경우:

```text
30 Hz → 약 1.67 m/update
60 Hz → 약 0.83 m/update
```

현재 50 m/s 조건에서 30 Hz pose update 동작을 확인했습니다.

---

## 11. 동작 확인

현재 다음 항목을 확인했습니다.

- `x500_mono_cam` 정상 spawn
- `target_standard_vtol` 정상 spawn
- static target의 고도 유지
- Gazebo `/set_pose` 서비스 정상 동작
- Python Gazebo Transport API 정상 동작
- 원형 경로 이동
- 타원형 경로 이동
- 직사각형 경로 이동
- 이동 방향에 따른 yaw 변경
- 50 m/s (180 km/h) 표적 이동
- 30 Hz pose update

---

## 12. 종료

이동 스크립트는 다음 키로 종료할 수 있습니다.

```text
Ctrl + C
```

PX4 SITL과 Gazebo도 각각 실행 중인 터미널에서 `Ctrl + C`로 종료합니다.

---

## 13. 사용 목적

본 환경은 실제 `standard_vtol`의 비행역학을 정밀하게 재현하기 위한 시뮬레이션이 아닙니다.

주요 목적은 다음과 같습니다.

- YOLO 기반 이동 표적 검출 테스트
- 고속 이동 표적에 대한 검출 안정성 확인
- Visual Servoing / 추적 제어 알고리즘 테스트
- 다양한 표적 경로에서의 추적 성능 비교
- 표적 속도 변화에 따른 인식 및 제어 성능 검증

따라서 `target_standard_vtol`은 실제 PX4 제어를 통해 비행하지 않고, Gazebo의 pose를 직접 갱신하여 지정된 trajectory를 따라 이동합니다.
