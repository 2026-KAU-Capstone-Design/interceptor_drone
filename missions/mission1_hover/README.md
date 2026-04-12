# Mission 1 — 정밀 호버 + 착륙 오차 검증 ★

## 목표

Offboard 모드로 이륙 → 5m 호버 30초 → 착륙 → 착륙 지점 오차 측정

## 시나리오

```
     이륙(0,0)
        ↑
        │ 5m
        │
   [30초 호버]
        │
        ↓
     착륙(?,?)  ← 이 오차를 측정
```

## 측정 항목

1. **호버 중 x,y 위치 분산** — 위치가 얼마나 흔들리는지 (±m)
2. **착륙 지점 오차** = `√((x_land - x_takeoff)² + (y_land - y_takeoff)²)`
3. **무풍 vs 유풍(5m/s) vs 강풍(12m/s)** 조건별 비교

## 실행

### 기본 (템플릿 그대로)
```bash
ros2 run interceptor_control offboard_hover
```

### 파라미터 변경
```bash
ros2 run interceptor_control offboard_hover --ros-args \
  -p target_altitude:=-5.0 \
  -p hover_duration:=30.0
```

## 코드 작성 가이드

`offboard_hover.py` 를 복사해서 `missions/mission1_hover/hover_land.py` 로 만드세요.
기본 템플릿에 다음을 추가하면 됩니다:

- [ ] 호버 중 매 1초마다 현재 (x,y,z) 를 리스트에 저장
- [ ] 착륙 후 위치 분산(std) 및 최대 편차 출력
- [ ] (선택) matplotlib 으로 호버 중 x-y 산점도 + 착륙 오차 그래프 저장

## 성공 기준

| 조건 | 착륙 오차 | 호버 드리프트 |
|---|---|---|
| 무풍 | < 0.3m | < 0.2m (std) |
| 보통풍 (5m/s) | < 0.5m | < 0.5m (std) |
| 강풍 (12m/s) | RTL 또는 < 1.0m | 안전 모니터 발동 확인 |

## 제출 체크리스트

- [ ] `hover_land.py` 작성 완료
- [ ] 무풍 rosbag 1회 이상
- [ ] 유풍 rosbag 1회 이상
- [ ] 오차 분석 결과 (표 또는 그래프)
- [ ] PR 제출
