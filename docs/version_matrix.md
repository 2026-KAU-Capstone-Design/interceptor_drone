# Version Matrix

이 문서는 프로젝트가 의존하는 모든 외부 컴포넌트의 **고정 버전**을 기록합니다.
변경은 반드시 PR을 통해서만 합니다.
`scripts/setup_dev_env.sh` 와 `repos/dev.repos` 가 이 표와 동기화되어 있어야 합니다.

---

## 핀(Pin) 버전

| 컴포넌트 | 버전 | 출처 | 비고 |
|---|---|---|---|
| Ubuntu | 22.04.5 LTS (jammy) | apt | 듀얼부팅, 프로젝트 전용 |
| ROS2 | Humble Hawksbill | packages.ros.org | LTS, 2027-05까지 |
| Gazebo | Harmonic | packages.osrfoundation.org | **LTS, 2028-09까지** |
| ros-humble-ros-gzharmonic | (apt) | osrfoundation | ROS↔Gazebo 브리지 |
| PX4-Autopilot | **v1.16.1** | github.com/PX4/PX4-Autopilot | 2026-01-21 stable |
| px4_msgs | **release/1.16** | github.com/PX4/px4_msgs | PX4 펌웨어와 메시지 매칭 |
| Micro-XRCE-DDS-Agent | **v2.4.3** | github.com/eProsima/Micro-XRCE-DDS-Agent | PX4 v1.16 권장 |
| Jetson 컴패니언 PC | Jetson Orin Nano (JetPack 6.x) | (실기체 단계) | Ubuntu 22.04 기반 → ROS2 Humble 네이티브 |

---

## 결정 근거

### 1. Gazebo Harmonic (Garden 아님)
- Gazebo Garden은 **2024-11 EOL**. 더 이상 보안/버그 패치 없음.
- Gazebo Harmonic은 **2023-09부터 2028-09까지 지원되는 LTS**.
- ROS2 Humble의 Tier 1 공식 짝은 엄밀히는 Fortress이지만, `packages.osrfoundation.org`에서
  공식 `ros-humble-ros-gzharmonic` 데비안 패키지를 배포하여 production 수준 사용 가능.

### 2. PX4 v1.16.1
- 2026-01-21 릴리스, 현재 stable.
- Ubuntu 22.04에서 PX4의 `Tools/setup/ubuntu.sh`가 Gazebo Harmonic 환경을 지원.
- **Pixhawk 보드 모델과 무관**. PX4는 단일 소스 트리이고 보드별 빌드 타깃만 다름.
  (예: `px4_fmu-v6c_default`, `px4_fmu-v6x_default`)
  → SW팀은 Pixhawk 모델 확정 전에도 SITL에서 작업 가능.

### 3. px4_msgs release/1.16
- `px4_msgs`의 메시지 정의는 PX4 펌웨어 버전과 1:1 강결합.
- 같은 릴리스 라인을 쓰지 않으면 Message Translation Node를 따로 운영해야 함.

### 4. Micro-XRCE-DDS-Agent v2.4.3
- PX4 v1.16 공식 ROS2 가이드가 권장하는 태그.

### 5. Docker 미사용
- GPU 렌더링 차이는 Docker로 해결되지 않음 (호스트 GPU 드라이버 의존).
- Pixhawk USB 패스스루, Gazebo GUI X11 forwarding 등 운영 비용이 큼.
- 학부 4학년 3명 팀의 인지 부하 최소화.
- 재현성은 `setup_dev_env.sh` + `verify_env.sh` + 버전 핀으로 확보.

---

## 변경 이력

| 날짜 | 변경 | 이유 |
|---|---|---|
| 2026-04-08 | 초기 핀 결정 | 프로젝트 시작. Gazebo Garden 후보를 Harmonic으로 정정. PX4 v1.14 후보를 v1.16.1로 정정. |
