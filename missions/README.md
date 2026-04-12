# Missions — 팀원별 비행 미션

## 개요

이 폴더에 각 미션의 코드, rosbag 기록, 분석 결과를 업로드합니다.

```
missions/
├── README.md                 ← 이 파일
├── mission1_hover/           ← M1: 정밀 호버 + 착륙 오차
│   ├── hover_land.py         ← 팀원이 작성할 코드
│   ├── rosbags/              ← 기록된 rosbag
│   └── analysis/             ← 오차 분석 그래프
├── mission2_nav/             ← M2: A→B 포인트 이동
│   ├── point_nav.py
│   ├── rosbags/
│   └── analysis/
└── mission3_square/          ← M3: 사각 비행
    ├── square_flight.py
    ├── rosbags/
    └── analysis/
```

## 실행 환경

### 무풍 (baylands)
```bash
# 커스텀 월드를 PX4에 복사 (최초 1회)
cp simulation/worlds/*.sdf ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/

# T1
MicroXRCEAgent udp4 -p 8888
# T2
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands make px4_sitl gz_x500
# T3
ros2 run interceptor_control offboard_hover
```

### 보통풍 (baylands_windy, ~5.4m/s)
```bash
# T2만 변경
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands_windy make px4_sitl gz_x500
```

### 강풍 (baylands_storm, ~11.7m/s)
```bash
# T2만 변경
cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands_storm make px4_sitl gz_x500
```

## rosbag 기록 방법

미션 실행 중 별도 터미널에서:
```bash
ros2 bag record /fmu/out/vehicle_local_position /fmu/out/vehicle_status_v1 -o missions/missionN_xxx/rosbags/run1
```

## 제출 규칙

1. 각자 `feature/<이름>-mission-<N>` 브랜치에서 작업
2. 코드 + rosbag + 분석 결과를 해당 미션 폴더에 저장
3. `dev` 브랜치로 PR → 다른 팀원 1명 리뷰 후 머지
4. PR 본문에: 실행 결과 캡처, 오차 수치, 소감/어려웠던 점

## 풍속 안전 기준

| 풍속 범위 | 분류 | 드론 동작 |
|---|---|---|
| 0 ~ 8 m/s | 정상 운용 | 미션 수행 |
| 8 ~ 15 m/s | 강풍 경고 | 로그 경고 출력, 모니터링 강화 |
| > 15 m/s | 위험 | **자동 RTL (귀환)** |
| 기울기 > 30° + 2초 미복원 | 자세 이상 | **비상 착륙** |

이 기준은 `config/mission_params.yaml` 에서 조정 가능합니다.
