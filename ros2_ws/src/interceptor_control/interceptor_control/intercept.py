#!/usr/bin/env python3
"""
Interceptor Drone — original mission + proven SITL yaw-only closed-loop mode.

Default: OBSERVE ONLY (read-only, no flight commands).
SITL flight mode: python3 intercept.py --yaw-only --enable-flight --ros-args -p use_sim_time:=true
Legacy balloon-intercept mode: python3 intercept.py --legacy-mission --enable-flight
WARNING: legacy mode has NOT been converted to stamped TargetDetection/TargetDistance.
Physical hardware must not use the SITL yaw test mode.

The original legacy controller class and state transitions are retained.
"""

"""
Interceptor Drone — Mission 3: Balloon Intercept
=================================================

이륙 → 탐색(SEARCH) → 추적(TRACK) → 요격(INTERCEPT) → 착륙

YOLO 감지 토픽을 구독하여 풍선을 추적하고 요격합니다.
yolo_live_detect.py 또는 동등한 감지 노드와 병행 실행해야 합니다.

상태 머신:
  IDLE → PREFLIGHT → ARMING → TAKEOFF → SEARCH → TRACK → INTERCEPT → LANDING → DONE

  SEARCH   : 제자리 yaw 회전하며 풍선 탐색
  TRACK    : 이미지 오차(dx, dy) 기반 velocity 제어로 풍선 추적
             lock_on 확정 후 v_approach 속도로 접근
  INTERCEPT: near_2m + lock_on 조건에서 진입
             유효한 표적 검출 소실 시 제로 속도 명령 발행
             연속 미검출 시 SEARCH 복귀 (충돌 성공 여부 미확인)

제어 방식: velocity setpoint (이미지 기반 비주얼 서보잉, IBVS)
  - yaw_rate ∝ dx (픽셀 → 좌우 yaw 보정)
  - vz       ∝ dy (픽셀 → 상하 고도 보정, NED 기준)
  - vx, vy   = v_forward × (cos/sin yaw) (현재 기수 방향 전진)

구독 토픽 (/target/*):
  balloon_bbox (Float32MultiArray, 17개 필드)
    [0]  detected   (0|1)
    [1]  conf
    [2-5] x1 y1 x2 y2 (smoothed bbox, pixels)
    [6-7] cx cy       (bbox 중심, pixels)
    [8-9] dx dy       (cx/cy - 이미지 중심, pixels)
    [10-11] image_w image_h
    [12] red_ratio
    [13] distance_m   (-1 = 추정 불가)
    [14] near_2m      (0|1)
    [15] class_id  (-1: 미검출, 0: 풍선, 1: 고정익)
    [16] lock_on   (0 또는 1)
  제어 노드는 위 bbox 메시지 하나에서 검출 상태, 거리, 근접 여부,
  lock_on 정보를 함께 읽습니다.
  YOLO 노드의 개별 토픽 발행은 다른 노드와의 호환성을 위해 유지합니다.

파라미터:
  target_altitude        : 이륙 고도 (NED, 음수=위) [기본: -5.0]
  v_approach             : TRACK 접근 속도 (m/s)   [기본: 1.5]
  v_intercept            : INTERCEPT 속도 (m/s)    [기본: 4.0]
  Kp_yaw                 : yaw 제어 이득            [기본: 0.8]
  Kp_vz                  : 고도 제어 이득 (m/s/1)  [기본: 1.0]
  search_yaw_rate        : 탐색 yaw 속도 (rad/s)   [기본: 0.3]
  lock_miss_limit        : TRACK→SEARCH 복귀 미감지 횟수 [기본: 15]
  wind_speed_warn        : 풍속 경고 (m/s)          [기본: 8.0]
  wind_speed_critical    : 강풍 RTL (m/s)           [기본: 15.0]
  attitude_recovery_timeout : 자세 복원 타임아웃(s) [기본: 2.0]
  tilt_threshold_deg     : 기울기 비상착륙 한계 (°) [기본: 30.0]
  position_threshold     : 이륙 완료 판정 거리 (m)  [기본: 0.5]
  expected_target_class   : 허용 표적 클래스 (0=풍선, 1=고정익) [기본: 0]
내부 설정:
  detection_timeout_sec   : bbox 수신 유효기간 (초) [현재 내부 설정: 0.5]

사용법:
  ros2 run interceptor_control intercept
  ros2 run interceptor_control intercept --ros-args \\
      -p target_altitude:=-8.0 -p v_approach:=2.0
"""

import math
import argparse
import time
from collections import OrderedDict
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray
from interceptor_interfaces.msg import TargetDetection, TargetDistance

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)

NAN = float('nan')


class FlightState(Enum):
    IDLE = auto()
    PREFLIGHT = auto()
    ARMING = auto()
    TAKEOFF = auto()
    SEARCH = auto()
    TRACK = auto()
    INTERCEPT = auto()
    RECOVERY_HOLD = auto()
    RTL = auto()
    LANDING = auto()
    DONE = auto()


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class InterceptNode(Node):
    def __init__(self):
        super().__init__('intercept')

        # ── 파라미터 ──
        self.declare_parameter('target_altitude', -5.0)
        self.declare_parameter('expected_target_class', 0)
        self.declare_parameter('v_approach', 1.5)
        self.declare_parameter('v_intercept', 4.0)
        self.declare_parameter('Kp_yaw', 0.8)
        self.declare_parameter('Kp_vz', 1.0)
        self.declare_parameter('search_yaw_rate', 0.3)
        self.declare_parameter('lock_miss_limit', 15)
        self.declare_parameter('wind_speed_warn', 8.0)
        self.declare_parameter('wind_speed_critical', 15.0)
        self.declare_parameter('attitude_recovery_timeout', 2.0)
        self.declare_parameter('tilt_threshold_deg', 30.0)
        self.declare_parameter('position_threshold', 0.5)

        self.target_alt = self.get_parameter('target_altitude').value
        self.expected_target_class = int(
            self.get_parameter('expected_target_class').value
        )
        self.v_approach = self.get_parameter('v_approach').value
        self.v_intercept = self.get_parameter('v_intercept').value
        self.Kp_yaw = self.get_parameter('Kp_yaw').value
        self.Kp_vz = self.get_parameter('Kp_vz').value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.lock_miss_limit = self.get_parameter('lock_miss_limit').value
        self.wind_warn = self.get_parameter('wind_speed_warn').value
        self.wind_critical = self.get_parameter('wind_speed_critical').value
        self.att_timeout = self.get_parameter('attitude_recovery_timeout').value
        self.tilt_threshold = self.get_parameter('tilt_threshold_deg').value
        self.pos_threshold = self.get_parameter('position_threshold').value

        # ── Publishers ──
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── PX4 Subscribers ──
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._cb_local_pos, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._cb_status, PX4_QOS)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._cb_odometry, PX4_QOS)
        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._cb_command_ack,
            PX4_QOS
        )

        # ── 인식 Subscribers ──
        # 인식 정보는 하나의 bbox 메시지에서 수신
        self.create_subscription(
            Float32MultiArray, '/target/balloon_bbox', self._cb_bbox, 10)

        # ── 비행 상태 ──
        self.state = FlightState.IDLE
        self.local_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.last_vehicle_status_time = None

        # PX4 상태 메시지 수신 중단 감지 기준
        self.vehicle_status_loss_timeout_sec = 1.5

        # RTL/LAND 제어권 이양 확인용 상태 최신성 기준
        # 초기에는 같은 값을 사용하고, 향후 SITL 시험을 통해 별도 조정
        self.handover_status_timeout_sec = 1.5

        self.last_command_ack = None
        self.last_command_ack_time = None
        self.pending_command = None
        self.pending_command_sent_time = None
        self.handover_command_sent = False
        self.handover_failed = False
        self.vehicle_status_lost_reported = False
        self.setpoint_counter = 0
        self.takeoff_xy = [0.0, 0.0]

        # ── 인식 상태 ──
        self.detected = False
        self.target_class_id = -1  # -1: 없음, 0: 풍선, 1: 고정익
        self.bbox_dx = 0.0       # 이미지 중심 대비 픽셀 오차 (양수 = 오른쪽/아래)
        self.bbox_dy = 0.0
        self.image_w = 1280.0
        self.image_h = 960.0
        self.lock_on = False
        self.distance_m = -1.0
        self.near_2m = False
        self.miss_count = 0      # TRACK/INTERCEPT 연속 미감지 횟수

        # 최근 YOLO bbox 메시지 수신 시각
        self.last_bbox_time = None
        self.detection_timeout_sec = 0.5

        # ── 자세/풍속 ──
        self.yaw_rad = 0.0
        self.tilt_deg = 0.0
        self.wind_speed_est = None  # 실제 풍속 정보 없음 (미측정)
        self.attitude_disturbed_since = None

        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('=== Mission 3: Balloon Intercept ===')
        self.get_logger().info(f'  이륙 고도    : {abs(self.target_alt):.1f}m')
        self.get_logger().info(f'  접근 속도    : {self.v_approach}m/s')
        self.get_logger().info(f'  요격 속도    : {self.v_intercept}m/s')
        self.get_logger().info(f'  탐색 yaw     : {math.degrees(self.search_yaw_rate):.1f}°/s')
        self.get_logger().info(f'  Kp_yaw={self.Kp_yaw}  Kp_vz={self.Kp_vz}')

    # =========================================================================
    # 인식 콜백
    # =========================================================================

    def _cb_bbox(self, msg: Float32MultiArray):
        d = msg.data

        # 17개 필드가 없거나 제어에 필요한 숫자가 비정상이면 거부
        required_indices = (0, 8, 9, 10, 11, 13, 14, 15, 16)

        invalid_message = (
            len(d) != 17
            or any(
                not math.isfinite(float(d[i]))
                for i in required_indices
            )
        )

        if not invalid_message:
            invalid_message = (
                d[0] > 0.5
                and (
                    d[10] <= 0.0
                    or d[11] <= 0.0
                )
            )

            # 검출 성공 시 클래스 ID는 정확히 0 또는 1이어야 함
        if not invalid_message and d[0] > 0.5:
            if d[15] not in (0.0, 1.0):
                invalid_message = True

        # 풍선 거리와 근접 플래그의 일관성 확인
        if not invalid_message and d[0] > 0.5:
            distance_m = float(d[13])
            near_2m = d[14] > 0.5
            class_id = int(d[15])

            if class_id == 0:
                # 거리 추정 불가 또는 0 이하이면 검출 정보 무효화
                if distance_m <= 0.0:
                    invalid_message = True

                # 현재 YOLO의 근접 기준은 2.0m
                elif near_2m != (distance_m <= 2.0):
                    invalid_message = True

            elif class_id == 1:
                # 고정익에는 아직 거리 추정을 적용하지 않음
                if distance_m != -1.0 or near_2m:
                    invalid_message = True

        if invalid_message:
            self.detected = False
            self.target_class_id = -1
            self.bbox_dx = 0.0
            self.bbox_dy = 0.0
            self.lock_on = False
            self.distance_m = -1.0
            self.near_2m = False
            self.last_bbox_time = None
            return

        # 정상 형식의 메시지를 받은 시각
        self.last_bbox_time = self.get_clock().now()

        # 표적 검출 여부 및 클래스
        self.detected = d[0] > 0.5
        self.target_class_id = int(d[15]) if self.detected else -1

        # 영상 중심 오차
        self.bbox_dx = float(d[8])
        self.bbox_dy = float(d[9])

        # 영상 크기
        if d[10] > 0:
            self.image_w = float(d[10])

        if d[11] > 0:
            self.image_h = float(d[11])

        # 같은 YOLO 메시지에서 나머지 정보 수신
        if self.detected:
            self.distance_m = float(d[13])
            self.near_2m = d[14] > 0.5
            self.lock_on = d[16] > 0.5
        else:
            self.bbox_dx = 0.0
            self.bbox_dy = 0.0
            self.distance_m = -1.0
            self.near_2m = False
            self.lock_on = False

    def _is_detection_fresh(self) -> bool:
        """최근 bbox 메시지가 수신되었는지 확인한다."""
        if self.last_bbox_time is None:
            return False

        elapsed = (
            self.get_clock().now() - self.last_bbox_time
        ).nanoseconds / 1e9

        return 0.0 <= elapsed <= self.detection_timeout_sec

    def _has_valid_detection(self) -> bool:
        """최근 검출된 표적이 기대하는 클래스와 일치하는지 확인한다."""
        return (
            self._is_detection_fresh()
            and self.detected
            and self.target_class_id == self.expected_target_class
        )

    # =========================================================================
    # PX4 콜백
    # =========================================================================

    def _cb_local_pos(self, msg):
        """PX4 위치 및 지면 기준 속도 정보를 저장한다."""
        self.local_pos = msg

        # msg.vx, msg.vy는 기체의 지면 기준 속도이며
        # 실제 풍속이 아니므로 wind_speed_est에 대입하지 않는다.

    def _cb_status(self, msg):
        """PX4 비행 상태와 ROS 2 수신 시각 저장."""
        self.vehicle_status = msg
        self.last_vehicle_status_time = self.get_clock().now()

    def _cb_command_ack(self, msg):
        """PX4 명령 응답과 수신 시각 저장."""
        self.last_command_ack = msg
        self.last_command_ack_time = self.get_clock().now()

    def _is_pending_command_accepted(self) -> bool:
        """현재 대기 중인 명령에 대한 ACCEPTED 응답인지 검사."""
        if self.pending_command is None:
            return False

        if self.pending_command_sent_time is None:
            return False

        if self.last_command_ack is None:
            return False

        if self.last_command_ack_time is None:
            return False

        # 다른 명령에 대한 ACK는 인정하지 않음
        if self.last_command_ack.command != self.pending_command:
            return False

        # 수락된 명령만 인정
        if (
            self.last_command_ack.result
            != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        ):
            return False

        # 요청 이전에 저장된 ACK는 인정하지 않음
        if (
            self.last_command_ack_time.nanoseconds
            < self.pending_command_sent_time.nanoseconds
        ):
            return False

        return True

    def _is_pending_command_rejected(self) -> bool:
        """현재 종료 명령에 대한 명시적인 거부·실패 ACK인지 확인."""

        if self.pending_command is None:
            return False

        if self.pending_command_sent_time is None:
            return False

        if self.last_command_ack is None:
            return False

        if self.last_command_ack_time is None:
            return False

        # 다른 명령에 대한 ACK는 무시
        if self.last_command_ack.command != self.pending_command:
            return False

        # 요청 이전에 수신한 ACK는 무시
        if (
            self.last_command_ack_time.nanoseconds
            < self.pending_command_sent_time.nanoseconds
        ):
            return False

        # 명시적으로 거부되거나 실패한 경우만 True
        return self.last_command_ack.result in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED,
        )

    def _is_handover_timed_out(self, timeout_sec: float = 3.0) -> bool:
        """종료 명령 전송 후 지정된 대기 시간이 지났는지 확인."""

        if self.pending_command is None:
            return False

        if self.pending_command_sent_time is None:
            return False

        elapsed_sec = (
            self.get_clock().now()
            - self.pending_command_sent_time
        ).nanoseconds / 1e9

        # 시간이 역행한 경우에는 시간 초과로 판단하지 않음
        if elapsed_sec < 0:
            return False

        return elapsed_sec >= timeout_sec

    def _is_vehicle_status_fresh(self, timeout_sec=None) -> bool:
        """지정된 시간 이내에 PX4 상태 메시지를 수신했는지 확인."""

        if self.last_vehicle_status_time is None:
            return False

        # 별도 기준을 지정하지 않으면 통신 감시 기준 사용
        if timeout_sec is None:
            timeout_sec = self.vehicle_status_loss_timeout_sec

        now = self.get_clock().now()

        elapsed_sec = (
            now - self.last_vehicle_status_time
        ).nanoseconds / 1e9

        return 0.0 <= elapsed_sec <= timeout_sec

    def _is_vehicle_status_lost(self) -> bool:
        """한 번 이상 수신했던 PX4 상태 정보가 오래되었는지 확인."""

        # 상태 메시지를 한 번도 받지 않은 경우와
        # 수신하다가 끊긴 경우를 구분한다.
        if self.last_vehicle_status_time is None:
            return False

        return not self._is_vehicle_status_fresh()

    def _monitor_vehicle_status(self):
        """PX4 상태 메시지 수신 중단과 복구를 기록한다."""

        # PX4 상태 메시지를 수신하다가 끊긴 경우
        if self._is_vehicle_status_lost():
            if not self.vehicle_status_lost_reported:
                self.vehicle_status_lost_reported = True
                self.get_logger().error(
                    '[PX4 STATUS LOST] 상태 메시지 수신 중단 '
                    '(제어권 상태 확인 불가)'
                )

            # 추적 중 통신이 끊기면 안전 대기 상태로 전환
            if self.state in (
                FlightState.SEARCH,
                FlightState.TRACK,
                FlightState.INTERCEPT,
            ):
                self.state = FlightState.RECOVERY_HOLD
                self.get_logger().warning(
                    '→ RECOVERY_HOLD (PX4 상태 통신 단절)'
                )

        # 이전에 끊겼던 상태 메시지가 다시 수신된 경우
        elif (
            self.vehicle_status_lost_reported
            and self._is_vehicle_status_fresh()
        ):
            self.vehicle_status_lost_reported = False
            self.get_logger().info(
                '[PX4 STATUS RECOVERED] 상태 메시지 수신 복구'
            )

    def _is_pending_mode_active(self) -> bool:
        """최신 PX4 상태가 요청한 종료 비행 모드인지 확인."""

        if self.pending_command is None:
            return False

        # 제어권 이양 확인용 최신성 기준 적용
        if not self._is_vehicle_status_fresh(
            self.handover_status_timeout_sec
        ):
            return False

        # 종료 명령을 요청한 시각이 없으면 판단 불가
        if self.pending_command_sent_time is None:
            return False

        # 종료 명령 이전에 수신한 상태는 인정하지 않음
        if (
            self.last_vehicle_status_time.nanoseconds
            < self.pending_command_sent_time.nanoseconds
        ):
            return False

        nav_state = self.vehicle_status.nav_state

        # RTL 요청 → PX4가 실제 RTL 모드인지 확인
        if (
            self.pending_command
            == VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        ):
            return (
                nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_RTL
            )

        # 착륙 요청 → PX4가 실제 자동 착륙 모드인지 확인
        if (
            self.pending_command
            == VehicleCommand.VEHICLE_CMD_NAV_LAND
        ):
            return (
                nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            )

        return False

    def _is_handover_confirmed(self) -> bool:
        """PX4가 종료 명령을 수락하고 해당 비행 모드로 전환했는지 확인."""

        # 1. 현재 종료 명령에 대한 ACK 수락 여부
        if not self._is_pending_command_accepted():
            return False

        # 2. PX4의 실제 RTL/LAND 모드 전환 여부
        if not self._is_pending_mode_active():
            return False

        return True

    def _cb_odometry(self, msg):
        q = msg.q  # [w, x, y, z]
        if len(q) < 4:
            return

        siny = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        self.yaw_rad = float(math.atan2(siny, cosy))

        sinr = 2.0 * (q[0] * q[1] + q[2] * q[3])
        cosr = 1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q[0] * q[2] - q[3] * q[1])))
        pitch = math.asin(sinp)
        self.tilt_deg = float(math.degrees(math.sqrt(roll ** 2 + pitch ** 2)))
        self._check_attitude_safety()

    # =========================================================================
    # 안전 모니터
    # =========================================================================

    def _enter_handover_state(self, new_state: FlightState):
        """RTL 또는 LANDING 진입 시 종료 명령 관련 상태를 초기화한다."""

        if new_state not in (FlightState.RTL, FlightState.LANDING):
            raise ValueError("RTL 또는 LANDING 상태만 허용합니다.")

        # 동일한 상태로 다시 진입하면 초기화하지 않음
        if self.state == new_state:
            return

        self.handover_command_sent = False
        self.handover_failed = False
        self.pending_command = None
        self.pending_command_sent_time = None
        self.last_command_ack = None
        self.last_command_ack_time = None

        self.state = new_state

    def _check_wind_safety(self) -> bool:
        if self.state not in (
            FlightState.SEARCH,
            FlightState.TRACK,
            FlightState.INTERCEPT,
        ):
            return True

        # 신뢰할 수 있는 풍속 정보가 없는 경우
        if self.wind_speed_est is None:
            return True

        if self.wind_speed_est >= self.wind_critical:
            self.get_logger().error(
                f'[WIND CRITICAL] {self.wind_speed_est:.1f}m/s '
                f'≥ {self.wind_critical}m/s → RTL'
            )
            self._enter_handover_state(FlightState.RTL)
            return False

        if self.wind_speed_est >= self.wind_warn:
            self.get_logger().warn(
                f'[WIND WARNING] {self.wind_speed_est:.1f}m/s'
            )

        return True

    def _check_attitude_safety(self):
        if self.state in (FlightState.IDLE, FlightState.PREFLIGHT, FlightState.DONE):
            return
        if self.tilt_deg > self.tilt_threshold:
            now = self.get_clock().now()
            if self.attitude_disturbed_since is None:
                self.attitude_disturbed_since = now
                self.get_logger().warn(
                    f'[ATTITUDE] 기울기 {self.tilt_deg:.1f}° > {self.tilt_threshold}° — 복원 대기')
            else:
                elapsed = (now - self.attitude_disturbed_since).nanoseconds / 1e9
                if elapsed > self.att_timeout:
                    self.get_logger().error(
                        f'[ATTITUDE CRITICAL] {elapsed:.1f}초 내 복원 실패 → 비상 착륙')
                    self._enter_handover_state(FlightState.LANDING)
        else:
            if self.attitude_disturbed_since is not None:
                elapsed = (self.get_clock().now() - self.attitude_disturbed_since).nanoseconds / 1e9
                self.get_logger().info(f'[ATTITUDE] 자세 복원 완료 ({elapsed:.2f}초)')
                self.attitude_disturbed_since = None

    # =========================================================================
    # PX4 명령 헬퍼
    # =========================================================================

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_offboard_mode(self, *, position: bool):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = not position
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_pub.publish(msg)

    def _publish_position_sp(self, x=0.0, y=0.0, z=-5.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = float(yaw)
        msg.yawspeed = NAN
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def _publish_velocity_sp(self, vx=0.0, vy=0.0, vz=0.0, yawspeed=0.0):
        msg = TrajectorySetpoint()
        msg.position = [NAN, NAN, NAN]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = NAN
        msg.yawspeed = float(yawspeed)
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)


    def _send_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        # RTL/LAND 요청만 종료 명령 대기 상태로 등록
        if command in (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        ):
            self.pending_command = command
            self.pending_command_sent_time = self.get_clock().now()

            # 새 종료 명령에 이전 ACK를 사용하지 않도록 초기화
            self.last_command_ack = None
            self.last_command_ack_time = None

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.command_pub.publish(msg)

    def arm(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('[CMD] ARM')

    def engage_offboard(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('[CMD] OFFBOARD')

    def land(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('[CMD] LAND')

    def return_to_launch(self):
        self._send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('[CMD] RTL')

    # =========================================================================
    # 비주얼 서보잉 (IBVS)
    # =========================================================================

    def _compute_velocity_cmd(self, v_forward: float):
        """
        이미지 오차 → NED 속도 명령 변환

        ex: 정규화 가로 오차 [-1, 1] (양수 = 풍선이 오른쪽)
        ey: 정규화 세로 오차 [-1, 1] (양수 = 풍선이 아래쪽)

        yaw_rate: 양수 = 시계 방향 (NED 기준)
        vz: 양수 = 하강 (NED 기준)
        vx, vy: 현재 기수(yaw) 방향 전진
        """
        ex = self.bbox_dx / (self.image_w / 2.0)
        ey = self.bbox_dy / (self.image_h / 2.0)

        yaw_rate = float(np.clip(self.Kp_yaw * ex, -1.0, 1.0))
        vz = float(np.clip(self.Kp_vz * ey, -2.0, 2.0))

        vx_ned = v_forward * math.cos(self.yaw_rad)
        vy_ned = v_forward * math.sin(self.yaw_rad)

        return vx_ned, vy_ned, vz, yaw_rate

    # =========================================================================
    # 제어 루프 (10 Hz)
    # =========================================================================

    def _control_loop(self):
        # 종료 상태에서는 새로운 Offboard 명령을 발행하지 않음
        if self.state == FlightState.DONE:
            return

        # PX4 상태 메시지 수신 중단 및 복구 감시
        self._monitor_vehicle_status()

        # 안전 대기 중 PX4가 RTL, LAND 또는 POSCTL 모드라면
        # 새로운 Offboard 제어 신호 및 속도 목표를 발행하지 않음
        if (
            self.state == FlightState.RECOVERY_HOLD
            and self._is_vehicle_status_fresh()
            and self.vehicle_status.nav_state in (
                VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
                VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                VehicleStatus.NAVIGATION_STATE_POSCTL,
                VehicleStatus.NAVIGATION_STATE_ALTCTL,
            )
        ):
            return

        use_position = self.state in (
            FlightState.IDLE, FlightState.PREFLIGHT,
            FlightState.ARMING, FlightState.TAKEOFF,
        )
        self._publish_offboard_mode(position=use_position)

        if self.state == FlightState.IDLE:
            self._publish_position_sp(z=self.target_alt)
            self.setpoint_counter = 0
            self.state = FlightState.PREFLIGHT
            self.get_logger().info('→ PREFLIGHT')

        elif self.state == FlightState.PREFLIGHT:
            self._publish_position_sp(z=self.target_alt)
            self.setpoint_counter += 1
            if self.setpoint_counter >= 50:  # 5초 사전 전송 (EKF2 수렴 대기)
                self.state = FlightState.ARMING
                self.get_logger().info('→ ARMING')

        elif self.state == FlightState.ARMING:
            self._publish_position_sp(z=self.target_alt)
            self.engage_offboard()
            self.arm()
            self.takeoff_xy = [self.local_pos.x, self.local_pos.y]
            self.state = FlightState.TAKEOFF
            self.get_logger().info(f'→ TAKEOFF (목표 {abs(self.target_alt):.1f}m)')

        elif self.state == FlightState.TAKEOFF:
            self._publish_position_sp(z=self.target_alt)
            # arming_state=1이면 ARM 안 된 것 → 재시도
            if self.vehicle_status.arming_state != 2:
                self.setpoint_counter += 1
                if self.setpoint_counter % 50 == 0:  # 5초마다 재시도
                    self.engage_offboard()
                    self.arm()
                    self.get_logger().warn(
                        f'[TAKEOFF] ARM 미확인 (arming_state={self.vehicle_status.arming_state}) — 재시도')
                return
            if abs(self.local_pos.z - self.target_alt) < self.pos_threshold:
                self.miss_count = 0
                self.state = FlightState.SEARCH
                self.get_logger().info('→ SEARCH (풍선 탐색 시작)')

        elif self.state == FlightState.SEARCH:
            if not self._check_wind_safety():
                return

            # PX4 상태 정보가 유효하지 않으면 탐색 회전 명령 차단
            if not self._is_vehicle_status_fresh():
                self._publish_velocity_sp()
                return

            if self._has_valid_detection():
                self.miss_count = 0
                self.state = FlightState.TRACK
                self.get_logger().info(
                    f'→ TRACK (풍선 발견 dx={self.bbox_dx:.0f} dy={self.bbox_dy:.0f}px)')
            else:
                # 제자리에서 천천히 yaw 회전
                self._publish_velocity_sp(yawspeed=self.search_yaw_rate)

        elif self.state == FlightState.TRACK:
            if not self._check_wind_safety():
                return

            # PX4 상태 정보가 유효하지 않으면 새로운 추적 이동 명령 차단
            if not self._is_vehicle_status_fresh():
                self._publish_velocity_sp()
                return

            if not self._has_valid_detection():
                # 검출 데이터가 유효하지 않으면 우선 정지 명령 발행
                self._publish_velocity_sp()

                self.miss_count += 1

                if self.miss_count >= self.lock_miss_limit:
                    self.miss_count = 0
                    self.state = FlightState.SEARCH
                    self.get_logger().info(
                        f'→ SEARCH ({self.lock_miss_limit}제어 주기 동안 유효 검출 없음)')

                return

            self.miss_count = 0

            # lock_on = YOLO 10프레임 확신(7/10) + 중앙 오차 100px 이내
            # → 표적 확인 + 중앙 정렬 동시 달성 시 True

            # 2m 이내 + lock_on → INTERCEPT 돌진
            if self.near_2m and self.lock_on:
                # 상태 전환 시 이전 접근 속도 명령을 갱신
                self._publish_velocity_sp()
                self.state = FlightState.INTERCEPT
                self.get_logger().info(
                    f'→ INTERCEPT (2m + lock_on err=({self.bbox_dx:.0f},{self.bbox_dy:.0f})px)')
                return

            # lock_on이면 전진 접근, 아니면 제자리에서 중앙 정렬만
            if self.lock_on:
                v_fwd = self.v_approach
                phase = "접근"
            else:
                v_fwd = 0.0
                phase = "정렬"

            vx, vy, vz, yaw_rate = self._compute_velocity_cmd(v_fwd)
            self._publish_velocity_sp(vx, vy, vz, yaw_rate)

            self.get_logger().info(
                f'[TRACK/{phase}] lock={self.lock_on} dist={self.distance_m:.2f}m '
                f'err=({self.bbox_dx:.0f},{self.bbox_dy:.0f})px '
                f'vfwd={v_fwd:.1f} yaw={yaw_rate:.2f}')

        elif self.state == FlightState.INTERCEPT:
            # PX4 상태 정보가 유효하지 않으면 요격 이동 명령 차단
            if not self._is_vehicle_status_fresh():
                self._publish_velocity_sp()
                return

            if not self._has_valid_detection():
                self._publish_velocity_sp()

                self.miss_count += 1

                if self.miss_count >= self.lock_miss_limit:
                    self.miss_count = 0
                    self.state = FlightState.SEARCH
                    self.get_logger().warning(
                        '→ SEARCH (INTERCEPT 중 표적 정보 소실 — 결과 미확인)'
                    )

                return

            self.miss_count = 0

            vx, vy, vz, yaw_rate = self._compute_velocity_cmd(self.v_intercept)
            self._publish_velocity_sp(vx, vy, vz, yaw_rate)
            self.get_logger().info(
                f'[INTERCEPT] dist={self.distance_m:.2f}m '
                f'vfwd={self.v_intercept}m/s yaw={yaw_rate:.2f}')

        elif self.state == FlightState.RECOVERY_HOLD:
            # 통신 복구 후에도 자동으로 추적을 재개하지 않음
            # 임시 안전 대기: 이동 및 회전 목표를 0으로 설정
            self._publish_velocity_sp()


        elif self.state == FlightState.RTL:
            # PX4가 아직 OFFBOARD이면 제로 속도 목표 발행
            if (
                self._is_vehicle_status_fresh(
                    self.handover_status_timeout_sec
                )
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self._publish_velocity_sp()

            if not self.handover_command_sent:
                self.return_to_launch()
                self.handover_command_sent = True
                self.get_logger().info(
                    '→ RTL 요청 전송, PX4 응답 대기'
                )

            elif self._is_handover_confirmed():
                self.state = FlightState.DONE
                self.get_logger().info(
                    '→ DONE (PX4 RTL 모드 전환 확인)'
                )

            elif not self.handover_failed:
                if self._is_pending_command_rejected():
                    self.handover_failed = True
                    self.get_logger().error(
                        'RTL 명령 거부/실패: PX4 제어권 이양 미확인'
                    )

                elif self._is_handover_timed_out():
                    self.handover_failed = True
                    self.get_logger().error(
                        'RTL 응답 시간 초과: PX4 제어권 이양 미확인'
                    )

        elif self.state == FlightState.LANDING:
            # PX4가 아직 OFFBOARD라면 제로 속도 목표 발행
            if (
                self._is_vehicle_status_fresh(
                    self.handover_status_timeout_sec
                )
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self._publish_velocity_sp()

            if not self.handover_command_sent:
                # 착륙 명령 최초 한 번 전송
                self.land()
                self.handover_command_sent = True
                self.get_logger().info(
                    '→ LAND 요청 전송, PX4 응답 대기'
                )

            elif self._is_handover_confirmed():
                # ACK 수락 및 AUTO_LAND 모드 확인
                self.state = FlightState.DONE
                self.get_logger().info(
                    '→ DONE (PX4 LAND 모드 전환 확인)'
                )

            elif not self.handover_failed:
                if self._is_pending_command_rejected():
                    self.handover_failed = True
                    self.get_logger().error(
                        'LAND 명령 거부/실패: PX4 제어권 이양 미확인'
                    )

                elif self._is_handover_timed_out():
                    self.handover_failed = True
                    self.get_logger().error(
                        'LAND 응답 시간 초과: PX4 제어권 이양 미확인'
                    )



    # =========================================================================
    # 통계
    # =========================================================================

    def _print_stats(self):
        dx = self.local_pos.x - self.takeoff_xy[0]
        dy = self.local_pos.y - self.takeoff_xy[1]
        err = math.hypot(dx, dy)
        self.get_logger().info('──── 요격 결과 ────')
        self.get_logger().info(
            f'  이륙 지점    : ({self.takeoff_xy[0]:.3f}, {self.takeoff_xy[1]:.3f})')
        self.get_logger().info(
            f'  착륙 지점    : ({self.local_pos.x:.3f}, {self.local_pos.y:.3f})')
        self.get_logger().info(f'  수평 이탈거리 : {err:.3f}m')
        self.get_logger().info(f'  최종 표적 거리: {self.distance_m:.2f}m')



# ---------------- validated yaw-only SITL controller (isolated) ----------------
def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class YawClosedLoopSITL(Node):
    def __init__(self, enable_flight=False):
        super().__init__('yaw_closed_loop_sitl')
        self.enable_flight = enable_flight
        self.state = 'PREFLIGHT' if enable_flight else 'OBSERVE'
        self.state_since = time.monotonic()
        self.last_status_wall = None
        self.last_pos_wall = None
        self.status = None
        self.pos = None
        self.home = None
        self.pairs = OrderedDict()
        self.valid_pair = None
        self.last_pair_wall = None
        self.cmd_yaw = 0.0
        self.last_log_wall = 0.0
        self.goal_z = -1.5
        self.max_yaw_rate = 0.15   # rad/s
        self.gain_yaw = 0.4
        self.data_age_limit_ns = 150_000_000
        self.pair_timeout_wall = 0.30
        self.takeoff_started = None
        self.yaw_started = None
        self.land_requested = False
        self.create_subscription(TargetDetection, '/target/detection', self.on_detection, 10)
        self.create_subscription(TargetDistance, '/target/distance_stamped', self.on_distance, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_pos, PX4_QOS)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.create_timer(0.1, self.tick)
        self.get_logger().warning(f'SITL YAW TEST; flight_enabled={enable_flight}; no forward, vertical or intercept movement')

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def set_state(self, value):
        if self.state != value:
            self.state = value
            self.state_since = time.monotonic()
            self.get_logger().warning(f'STATE={value}')

    def on_status(self, msg):
        self.status = msg
        self.last_status_wall = time.monotonic()

    def on_pos(self, msg):
        self.pos = msg
        self.last_pos_wall = time.monotonic()

    def on_detection(self, msg):
        self.remember('det', msg)

    def on_distance(self, msg):
        self.remember('dist', msg)

    def remember(self, kind, msg):
        key = stamp_ns(msg.header)
        if key <= 0:
            return
        entry = self.pairs.setdefault(key, {})
        entry[kind] = msg
        entry['rx_wall'] = time.monotonic()
        if 'det' in entry and 'dist' in entry:
            self.validate_pair(key, entry['det'], entry['dist'])
            self.pairs.pop(key, None)
        while len(self.pairs) > 30:
            self.pairs.popitem(last=False)

    def validate_pair(self, key, det, dist):
        now = self.get_clock().now().nanoseconds
        age_ns = now - key
        box = (det.x1, det.y1, det.x2, det.y2)
        valid = (
            det.detected and det.class_id == 1 and dist.valid
            and det.header.frame_id == dist.header.frame_id
            and 0 <= age_ns <= self.data_age_limit_ns
            and math.isfinite(dist.distance_m) and 0.3 < dist.distance_m < 20
            and math.isfinite(dist.sync_diff_ms) and abs(dist.sync_diff_ms) <= 20
            and all(math.isfinite(v) for v in box)
            and 0 <= det.x1 < det.x2 <= 1280
            and 0 <= det.y1 < det.y2 <= 960
        )
        if not valid:
            self.valid_pair = None
            return
        # Current simulation RGB image: 1280 x 960. Right of center -> positive NED yaw.
        dx = ((det.x1 + det.x2) * 0.5 - 640.0) / 640.0
        self.valid_pair = (dx, dist.distance_m)
        self.last_pair_wall = time.monotonic()

    def px4_ready(self):
        now = time.monotonic()
        return (
            self.status is not None and self.pos is not None
            and self.last_status_wall is not None and self.last_pos_wall is not None
            and now - self.last_status_wall < 1.0 and now - self.last_pos_wall < 1.0
            and self.pos.xy_valid and self.pos.z_valid
        )

    def pub_mode(self, position):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = not position
        msg.timestamp = self.now_us()
        self.mode_pub.publish(msg)

    def pub_position(self):
        msg = TrajectorySetpoint()
        x, y, z = self.home
        msg.position = [float(x), float(y), float(z + self.goal_z)]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = 0.0
        msg.yawspeed = NAN
        msg.timestamp = self.now_us()
        self.sp_pub.publish(msg)

    def pub_yaw(self, yaw_rate):
        msg = TrajectorySetpoint()
        msg.position = [NAN, NAN, NAN]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [NAN, NAN, NAN]
        msg.yaw = NAN
        msg.yawspeed = float(yaw_rate)
        msg.timestamp = self.now_us()
        self.sp_pub.publish(msg)

    def command(self, command_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command_id
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def request_land(self, reason):
        if self.land_requested:
            return
        self.land_requested = True
        self.get_logger().error(f'LAND REQUEST: {reason}')
        self.pub_yaw(0.0)
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.set_state('LAND_REQUESTED')

    def tick(self):
        now = time.monotonic()
        if self.valid_pair is None or self.last_pair_wall is None or now - self.last_pair_wall > self.pair_timeout_wall:
            self.cmd_yaw = 0.0
            current_pair = None
        else:
            current_pair = self.valid_pair
            self.cmd_yaw = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.gain_yaw * current_pair[0]))

        if now - self.last_log_wall > 1.0:
            d = current_pair[1] if current_pair else float('nan')
            e = current_pair[0] if current_pair else float('nan')
            self.get_logger().info(
                f'state={self.state} ready={self.px4_ready()} ex={e:.3f} dist={d:.2f} yaw_cmd={self.cmd_yaw:.3f} rad/s'
            )
            self.last_log_wall = now

        if self.state in ('OBSERVE', 'LAND_REQUESTED'):
            return
        if not self.px4_ready():
            if self.state in ('TAKEOFF', 'YAW_CONTROL'):
                self.request_land('PX4 status or local position stale/invalid')
            return
        if self.state == 'PREFLIGHT':
            if self.home is None:
                self.home = (self.pos.x, self.pos.y, self.pos.z)
                self.state_since = now
                self.get_logger().info(f'home={self.home}, target relative altitude=1.5m')
            self.pub_mode(position=True)
            self.pub_position()
            if now - self.state_since >= 3.0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.takeoff_started = now
                self.set_state('TAKEOFF')
            return
        if self.state == 'TAKEOFF':
            self.pub_mode(position=True)
            self.pub_position()
            if now - self.takeoff_started > 12.0:
                self.request_land('TAKEOFF timeout')
                return
            if (self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and abs(self.pos.z - (self.home[2] + self.goal_z)) < 0.35):
                self.yaw_started = now
                self.set_state('YAW_CONTROL')
            return
        if self.state == 'YAW_CONTROL':
            if now - self.yaw_started > 15.0:
                self.request_land('15-second yaw test complete')
                return
            if (self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED
                or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.request_land('PX4 left armed Offboard state')
                return
            # Zero velocity axes; only bounded yaw rate can change.
            self.pub_mode(position=False)
            self.pub_yaw(self.cmd_yaw)



def main():
    """SITL yaw-only observe by default. Legacy mission must be explicit."""
    parser = argparse.ArgumentParser(description='Interceptor controller (SITL test gated)')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--yaw-only', action='store_true', help='Use validated SITL yaw controller (default)')
    modes.add_argument('--legacy-mission', action='store_true', help='Use original interception mission; NOT verified for new sensor topics')
    parser.add_argument('--enable-flight', action='store_true', help='EXPLICIT FLIGHT CONSENT: SITL ONLY')
    args, ros_args = parser.parse_known_args()
    if args.legacy_mission and not args.enable_flight:
        parser.error('--legacy-mission requires --enable-flight (old mission auto-arms/takes off)')
    rclpy.init(args=ros_args)
    node = None
    try:
        if args.legacy_mission:
            # Retained unmodified for backward compatibility. NOT integrated with stamped YOLO/Depth.
            node = InterceptNode()
            node.get_logger().warning('LEGACY MISSION: AUTO ARM/TAKEOFF ENABLED; NOT VALIDATED WITH NEW SENSOR TOPICS')
        else:
            node = YawClosedLoopSITL(enable_flight=args.enable_flight)
            if args.enable_flight and not bool(node.get_parameter('use_sim_time').value):
                raise RuntimeError('Yaw flight test requires ROS use_sim_time:=true (Gazebo SITL only)')
        rclpy.spin(node)
    except KeyboardInterrupt:
        if isinstance(node, YawClosedLoopSITL):
            if node.enable_flight and node.state in ('TAKEOFF', 'YAW_CONTROL'):
                node.request_land('Ctrl+C')
                time.sleep(0.25)
        elif isinstance(node, InterceptNode):
            node.get_logger().warning('Ctrl+C: legacy mission LAND requested')
            node.land()
            time.sleep(0.25)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
