# PX4 Parameters — 우리 프로젝트가 변경하는 파라미터

PX4는 수백 개의 파라미터로 동작이 제어됩니다. 이 문서는 **우리 프로젝트가
기본값과 다르게 설정하는 파라미터**만 정리합니다. 변경 이유와 실기체 단계로
넘어갈 때 어떻게 되돌릴지를 함께 기록해서, 발표/보고서 자료로도 활용합니다.

---

## 시뮬레이션 단계 (현재)

### `NAV_DLL_ACT = 0` — Data Link Loss Action 비활성화

| 항목 | 값 |
|---|---|
| 기본값 | 사용 환경에 따라 다름 (PX4 v1.16 SITL 기본은 시동 차단으로 동작) |
| 우리 값 | **0** (No Action) |
| 적용 위치 | `~/dev/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` (setup 스크립트가 자동 패치) |

**왜 변경했나:**
- PX4의 사전 점검(pre-arm check) 중 "Ground Control Station(GCS) 연결 필요"
  항목이 v1.16 SITL에서 기본 활성화되어 있어, QGroundControl을 띄우지 않으면
  `commander takeoff` 가 `Preflight Fail: No connection to the ground control station`
  으로 막힙니다.
- 시뮬레이션 단계에서는 매번 GCS를 띄우는 것이 번거롭고, 어차피 지상 실험이라
  안전 위험도 없습니다.
- `NAV_DLL_ACT = 0` 으로 설정하면 PX4 commander의 점검 로직이 GCS를
  시동의 필수 조건으로 간주하지 않게 됩니다.
  (코드: `src/modules/commander/HealthAndArmingChecks/checks/rcAndDataLinkCheck.cpp`)

**`NAV_DLL_ACT` 값의 의미:**

| 값 | 동작 |
|---|---|
| 0 | No Action — 데이터 링크 손실 무시 (시뮬용) |
| 1 | Hold — 그 자리에 정지 |
| 2 | Return — 자동 귀환 (RTL) |
| 3 | Land — 즉시 착륙 |
| 5 | Terminate — 비행 종료 (프로펠러 정지) |

---

## 실기체 단계 (Phase 6에서 변경 예정)

실기체로 옮길 때는 위 시뮬용 패치를 **반드시 되돌리거나 안전한 값으로
변경**해야 합니다. 추천 매트릭스:

| 환경 | `NAV_DLL_ACT` | 이유 |
|---|---|---|
| 실내 테스트 (텔레메트리 RC만) | 1 (Hold) | 통신 끊김 시 그 자리에 머묾 |
| 실외 비행 + QGroundControl | 2 (Return) | 통신 끊김 시 자동 귀환 |
| 페일세이프 우선 | 3 (Land) | 통신 끊김 시 가장 가까운 안전 지점에 착륙 |

**되돌리는 방법:**
1. `~/dev/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`
   파일에서 `# >>> interceptor_drone params >>>` ~ `# <<< interceptor_drone params <<<`
   블록을 삭제하거나 `NAV_DLL_ACT` 값을 변경합니다.
2. 실기체용 별도 airframe 파일을 사용하는 경우 그 파일에 직접 설정합니다.
3. QGroundControl에서 파라미터 탭으로 직접 수정 후 `param save` (영구 저장).

---

## 향후 추가 예정 (Phase별)

다음은 작업이 진전되면 이 문서에 추가될 항목들입니다:

| 파라미터 | 단계 | 용도 |
|---|---|---|
| `MPC_XY_VEL_MAX` | Phase 2 | 수평 최대 속도 (요격 기동 튜닝) |
| `MPC_Z_VEL_MAX_UP` | Phase 2 | 상승 최대 속도 |
| `COM_RCL_EXCEPT` | Phase 4 | RC 손실 예외 모드 (Offboard에서 RC 없이 작동) |
| `EKF2_*` | Phase 5 | 실기체 EKF 튜닝 (HW팀에서 받은 IMU 스펙 반영) |

---

## 변경 이력

| 날짜 | 변경 | 이유 |
|---|---|---|
| 2026-04-08 | `NAV_DLL_ACT = 0` 추가 (X500 airframe) | SITL에서 GCS 없이 시동 가능하도록 |
