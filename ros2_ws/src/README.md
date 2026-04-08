# ros2_ws/src

이 디렉터리에 ROS2 패키지를 둡니다.

## 자동으로 가져오는 외부 패키지 (gitignore 대상)

`scripts/setup_dev_env.sh` 를 실행하면 `repos/dev.repos` 에 정의된 외부 저장소가
이 디렉터리로 clone됩니다. 현재는 다음이 포함됩니다:

- `px4_msgs/` — PX4 v1.16 메시지 정의 (release/1.16 브랜치)

이들 외부 저장소는 `.gitignore` 에 의해 git에 포함되지 않습니다.

## 우리가 직접 작성하는 패키지 (Phase 2 이후 추가 예정)

- `interceptor_bringup/` — launch 파일 모음
- `interceptor_control/` — offboard 제어 노드
- `interceptor_perception/` — 표적 검출/추적
- `interceptor_planning/` — 요격 경로 계획
- `interceptor_msgs/` — 커스텀 메시지/서비스

## 빌드

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```
