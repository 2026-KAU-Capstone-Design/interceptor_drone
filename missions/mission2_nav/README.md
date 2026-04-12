# Mission 2 — A→B 포인트 이동 ★★

## 목표

Offboard 이륙 → A(0,0,-5) → B(20,0,-5) 이동 → 5초 hold → A로 귀환 → 착륙

## 시나리오

```
  A(0,0)──────────────────B(20,0)
    ↑     20m 수평 비행      ↑
  이륙                     5초 정지
    ↓                        ↓
  착륙 ←──────────────── 귀환
```

## 새로 배우는 것

- **상태 머신 확장**: TAKEOFF → MOVE_TO_B → HOLD_B → MOVE_TO_A → LANDING
- **위치 도달 판정**: 목표 좌표와의 거리가 threshold 이하이면 다음 상태로 전환
- **NED 좌표 감각**: x=북, y=동, z=아래(음수가 위)

## 코드 작성 가이드

`offboard_hover.py` 를 복사해서 `missions/mission2_nav/point_nav.py` 로 만드세요.

수정할 부분:
1. `FlightState` 에 `MOVE_TO_B`, `HOLD_B`, `RETURN_TO_A` 추가
2. `_control_loop()` 에 해당 상태별 setpoint 로직 추가
3. 각 상태 전환 시 조건: `horizontal_distance_to(target_x, target_y) < threshold`

힌트 — setpoint만 바꾸면 PX4가 알아서 이동합니다:
```python
# A→B 이동: setpoint를 B의 좌표로 변경
self.publish_setpoint(x=20.0, y=0.0, z=self.target_alt)
```

## 측정 항목

1. **B 도착 시 위치 오차** — 목표 (20,0) 과의 차이
2. **이동 중 경로 직진성** — y축 편차 (이상: y=0 직선)
3. **총 비행 시간** — 이륙~착륙
4. **무풍 vs 유풍 비교**

## 성공 기준

| 조건 | B 도착 오차 | 경로 y편차 |
|---|---|---|
| 무풍 | < 0.5m | < 0.3m (max) |
| 보통풍 (5m/s) | < 1.0m | < 0.8m (max) |

## 제출 체크리스트

- [ ] `point_nav.py` 작성 완료
- [ ] 무풍 rosbag 1회 이상
- [ ] 유풍 rosbag 1회 이상
- [ ] 2D 경로 플롯 (x-y 평면, 이상 경로 vs 실제 경로)
- [ ] 오차 수치 표
- [ ] PR 제출
