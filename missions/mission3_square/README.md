# Mission 3 — 사각 비행 + 귀환 ★★★

## 목표

Offboard 이륙 → 10m×10m 사각형 비행 → 시작점 귀환 → 착륙

## 시나리오

```
  A(0,0)─────────────B(10,0)
    │                   │
    │    10m × 10m      │
    │                   │
  D(0,10)────────────C(10,10)

  경로: A → B → C → D → A → 착륙
  고도: 5m 유지 (NED z = -5.0)
```

## 새로 배우는 것

- **다중 waypoint 상태 머신**: 4개 꼭짓점 + 귀환 + 착륙
- **Waypoint 리스트 관리**: 하드코딩 대신 리스트로 관리
- **도달 판정 + 전환**: 각 waypoint 도착 → 다음으로 자동 전환

## 코드 작성 가이드

`offboard_hover.py` 를 복사해서 `missions/mission3_square/square_flight.py` 로 만드세요.

핵심 설계:
```python
# waypoint 리스트로 관리 (NED 좌표)
self.waypoints = [
    (10.0,  0.0, -5.0),   # B
    (10.0, 10.0, -5.0),   # C
    ( 0.0, 10.0, -5.0),   # D
    ( 0.0,  0.0, -5.0),   # A (귀환)
]
self.current_wp_idx = 0
```

NAVIGATE 상태에서:
```python
wp = self.waypoints[self.current_wp_idx]
self.publish_setpoint(x=wp[0], y=wp[1], z=wp[2])

if self.horizontal_distance_to(wp[0], wp[1]) < self.pos_threshold:
    self.get_logger().info(f'Waypoint {self.current_wp_idx} 도달')
    self.current_wp_idx += 1
    if self.current_wp_idx >= len(self.waypoints):
        self.state = FlightState.LANDING
```

## 측정 항목

1. **각 꼭짓점 도달 오차** — 4개 포인트 각각
2. **변(辺) 직선성** — A→B, B→C, C→D, D→A 각 구간의 경로 편차
3. **총 비행 시간**
4. **누적 경로 오차** = Σ(이상 경로 - 실제 경로) 적분
5. **무풍 vs 유풍 비교**

## 성공 기준

| 조건 | 꼭짓점 오차 (평균) | 변 직선성 (max 편차) |
|---|---|---|
| 무풍 | < 0.5m | < 0.5m |
| 보통풍 (5m/s) | < 1.0m | < 1.0m |

## 제출 체크리스트

- [ ] `square_flight.py` 작성 완료
- [ ] 무풍 rosbag 1회 이상
- [ ] 유풍 rosbag 1회 이상
- [ ] 2D 경로 플롯 (이상 사각형 vs 실제 경로)
- [ ] 꼭짓점 오차 표
- [ ] PR 제출
