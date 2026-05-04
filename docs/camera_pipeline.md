# Camera Pipeline — 드론 카메라 이미지 수신

Gazebo SITL의 드론 카메라에서 ROS2 노드까지 이미지가 전달되는 전 과정.

---

## 데이터 흐름 (한눈에)

```
 ┌──────────────────────────────────────────────────────────────────┐
 │  Gazebo Harmonic                                                 │
 │  ┌───────────────────────────────────────────────────────────┐   │
 │  │  X500 드론 모델 (x500_mono_cam)                            │   │
 │  │     └── camera_link                                        │   │
 │  │           └── sensor name="imager" type="camera"           │   │
 │  │                 1280x960, FOV 1.74rad (~99°), 30Hz         │   │
 │  └───────────────────────────────────────────────────────────┘   │
 │            │                                                     │
 │            ▼  Gazebo 자체 transport (gz transport)               │
 │  /world/<world>/model/x500_mono_cam_0/                           │
 │     link/camera_link/sensor/imager/image      (gz.msgs.Image)    │
 │     link/camera_link/sensor/imager/camera_info(gz.msgs.CameraInfo)│
 └────────────────┬─────────────────────────────────────────────────┘
                  │
                  ▼  ros_gz_bridge / ros_gz_image
 ┌──────────────────────────────────────────────────────────────────┐
 │  ROS2                                                            │
 │  /camera/image       (sensor_msgs/Image)        ★ 우리 토픽 ★    │
 │  /camera/camera_info (sensor_msgs/CameraInfo)                    │
 └────────────────┬─────────────────────────────────────────────────┘
                  │
                  ▼  ROS2 노드들이 구독
       ┌──────────┴────────────┐
       │                       │
  image_probe           perception 노드 (예정)
   (진단)              cv_bridge → numpy → YOLO → /target/position
```

---

## 사용한 컴포넌트

| 컴포넌트 | 역할 |
|---|---|
| `x500_mono_cam` | PX4 제공 카메라 장착 X500 airframe (`4010_gz_x500_mono_cam`) |
| `mono_cam` 모델 | 카메라 센서 정의 (1280x960, 30Hz, 99° FOV) |
| `simple_windy_balloon.sdf` | 우리가 만든 월드 (sky+ground+wind+검붉은풍선) |
| `ros_gz_image` | gz Image → ROS2 sensor_msgs/Image 전용 브리지 (이미지 압축 효율적) |
| `ros_gz_bridge` | 일반 메시지 브리지 (CameraInfo 등) |
| `interceptor_control/launch/camera_bridge.launch.py` | 두 브리지를 한 번에 띄우는 launch |
| `image_probe` 노드 | 이미지 수신 검증 (fps/해상도/검붉은색 비율 출력) |

---

## 실행 절차 (4개 터미널)

```bash
# T1 — uXRCE-DDS Agent (드론 명령용, 카메라와는 무관하지만 비행에 필수)
MicroXRCEAgent udp4 -p 8888

# T2 — PX4 SITL: 카메라 장착 + 풍선 월드
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=simple_windy_balloon make px4_sitl gz_x500_mono_cam

# T3 — Gazebo ↔ ROS2 카메라 브리지
ros2 launch interceptor_control camera_bridge.launch.py

# T4 — 이미지 수신 검증
ros2 run interceptor_control image_probe
# 또는 시각적으로 확인:
ros2 run rqt_image_view rqt_image_view /camera/image
```

---

## 검증 방법

### 1. 토픽 존재 여부
```bash
ros2 topic list | grep camera
# 기대 출력:
#   /camera/image
#   /camera/camera_info
```

### 2. 이미지가 실제로 흐르는지
```bash
ros2 topic hz /camera/image
# 기대: ~30Hz (카메라 update_rate)
```

### 3. 이미지 정보
```bash
ros2 topic echo /camera/image --once --field "[width, height, encoding]"
# 기대: width=1280, height=960, encoding="rgb8" (또는 bgr8)
```

### 4. image_probe 출력 (가장 빠른 종합 확인)
```bash
ros2 run interceptor_control image_probe
```
정상이면 1초마다:
```
fps= 30  1280x960 rgb8     ~3686.4KB/frame  red_ratio=0.012  (total 30)
```
- `fps≈30` 이면 이미지가 실시간으로 흐르는 중
- `red_ratio` 가 0보다 크면 검붉은 영역이 카메라에 잡힘 → 풍선 시야 확보

### 5. 시각적 확인 (rqt_image_view)
```bash
ros2 run rqt_image_view rqt_image_view /camera/image
```
드론 카메라가 보는 이미지가 GUI 창에 떠야 함. 처음엔 지면이 보이고, 비행하면서 풍선이 시야에 들어와야 함.

---

## 주요 토픽 이름 규칙 (Gazebo → ROS2)

Gazebo가 자동 생성하는 카메라 토픽 경로:
```
/world/<world_name>/model/<model_name>/link/<link_name>/sensor/<sensor_name>/<msg_type>
```

이번 프로젝트에서:
- `<world_name>` = `simple_windy_balloon`
- `<model_name>` = `x500_mono_cam_0` ← PX4 SITL이 spawn할 때 `_0` 접미사 자동 추가
- `<link_name>` = `camera_link`
- `<sensor_name>` = `imager`

따라서 실제 토픽은:
```
/world/simple_windy_balloon/model/x500_mono_cam_0/link/camera_link/sensor/imager/image
```

이 긴 이름을 `/camera/image` 로 단축한 것이 launch 파일의 `remappings` 역할.

다른 월드/모델로 바꾸면 launch 인자 수정:
```bash
ros2 launch interceptor_control camera_bridge.launch.py \
  world:=다른월드 model:=x500_depth_0
```

---

## 풍선 모델 정보

`simulation/worlds/simple_windy_balloon.sdf` 안에 정의:

| 속성 | 값 |
|---|---|
| 모델명 | `balloon_red` |
| 위치 | (5, 0, 4) Gazebo ENU (드론에서 동쪽 5m, 고도 4m) |
| 형상 | sphere, 반지름 0.5m |
| 색상 | 검붉은색 RGB(0.45, 0.05, 0.05) |
| 정적 | static (위치 고정) |

PX4 NED 좌표로 환산: 드론에서 북쪽(N) 0m, 동쪽(E) 5m, 위(z=-4 in NED) 4m.

---

## 실패 시 체크리스트

| 증상 | 원인 | 해결 |
|---|---|---|
| `/camera/image` 토픽이 안 보임 | x500_mono_cam이 아닌 그냥 x500 사용 | T2 명령에 `gz_x500_mono_cam` 사용 |
| Gazebo는 카메라 보이지만 ROS2엔 없음 | bridge 미실행 | T3에서 launch 실행 |
| bridge는 떴는데 image 0Hz | 모델명 불일치 (model 인자) | Gazebo 측 모델명 확인 후 launch 인자 수정 |
| rqt_image_view가 까만 화면 | 카메라 위치/방향 문제 | Gazebo에서 카메라 시점으로 봐서 확인 |
| 풍선이 안 보임 | 풍선 위치 / 카메라 시야 밖 | 드론 yaw=0이면 동쪽 5m에 풍선이 정확히 정면 |

---

## 다음 단계

이 파이프라인이 동작하면 다음으로:
1. **YOLO 추론 노드** (`perception/`) — `/camera/image` 구독 → YOLOv8n → `/target/bounding_box`
2. **3D 위치 추정** — bbox 중심 + camera_info + 드론 자세 → `/target/position` (NED 좌표)
3. **요격 노드** — `/target/position` 구독 → offboard setpoint 갱신 → 드론 이동
