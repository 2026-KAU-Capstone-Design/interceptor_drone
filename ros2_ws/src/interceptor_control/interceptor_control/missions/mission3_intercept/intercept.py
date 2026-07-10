#!/usr/bin/env python3
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
  INTERCEPT: near_2m 진입 시 v_intercept 속도로 돌진
             lock_on 소실(풍선 파괴/소실) → LANDING

제어 방식: velocity setpoint (이미지 기반 비주얼 서보잉, IBVS)
  - yaw_rate ∝ dx (픽셀 → 좌우 yaw 보정)
  - vz       ∝ dy (픽셀 → 상하 고도 보정, NED 기준)
  - vx, vy   = v_forward × (cos/sin yaw) (현재 기수 방향 전진)

구독 토픽 (/target/*):
  balloon_bbox (Float32MultiArray, 15개 필드)
    [0]  detected   (0|1)
    [1]  conf
    [2-5] x1 y1 x2 y2 (smoothed bbox, pixels)
    [6-7] cx cy       (bbox 중심, pixels)
    [8-9] dx dy       (cx/cy - 이미지 중심, pixels)
    [10-11] image_w image_h
    [12] red_ratio
    [13] distance_m   (-1 = 추정 불가)
    [14] near_2m      (0|1)
  lock_on   (Bool)
  distance_m (Float32)
  near_2m   (Bool)

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

사용법:
  ros2 run interceptor_control intercept
  ros2 run interceptor_control intercept --ros-args \\
      -p target_altitude:=-8.0 -p v_approach:=2.0
"""

import math
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
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

        # ── 인식 Subscribers ──
        self.create_subscription(
            Float32MultiArray, '/target/balloon_bbox', self._cb_bbox, 10)
        self.create_subscription(
            Bool, '/target/lock_on', self._cb_lock_on, 10)
        self.create_subscription(
            Float32, '/target/distance_m', self._cb_distance, 10)
        self.create_subscription(
            Bool, '/target/near_2m', self._cb_near, 10)

        # ── 비행 상태 ──
        self.state = FlightState.IDLE
        self.local_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.setpoint_counter = 0
        self.takeoff_xy = [0.0, 0.0]

        # ── 인식 상태 ──
        self.detected = False
        self.bbox_dx = 0.0       # 이미지 중심 대비 픽셀 오차 (양수 = 오른쪽/아래)
        self.bbox_dy = 0.0
        self.image_w = 1280.0
        self.image_h = 960.0
        self.lock_on = False
        self.distance_m = -1.0
        self.near_2m = False
        self.miss_count = 0      # TRACK/INTERCEPT 연속 미감지 횟수

        # ── 자세/풍속 ──
        self.yaw_rad = 0.0
        self.tilt_deg = 0.0
        self.wind_speed_est = 0.0
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
        if len(d) < 15:
            return
        self.detected = d[0] > 0.5
        self.bbox_dx = float(d[8])
        self.bbox_dy = float(d[9])
        if d[10] > 0:
            self.image_w = float(d[10])
        if d[11] > 0:
            self.image_h = float(d[11])

    def _cb_lock_on(self, msg: Bool):
        self.lock_on = msg.data

    def _cb_distance(self, msg: Float32):
        self.distance_m = msg.data

    def _cb_near(self, msg: Bool):
        self.near_2m = msg.data

    # =========================================================================
    # PX4 콜백
    # =========================================================================

    def _cb_local_pos(self, msg):
        self.local_pos = msg
        if self.state in (FlightState.TRACK, FlightState.INTERCEPT) and msg.v_xy_valid:
            self.wind_speed_est = float(math.hypot(msg.vx, msg.vy))

    def _cb_status(self, msg):
        self.vehicle_status = msg

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

    def _check_wind_safety(self) -> bool:
        if self.state not in (FlightState.SEARCH, FlightState.TRACK, FlightState.INTERCEPT):
            return True
        if self.wind_speed_est >= self.wind_critical:
            self.get_logger().error(
                f'[WIND CRITICAL] {self.wind_speed_est:.1f}m/s ≥ {self.wind_critical}m/s → RTL')
            self.state = FlightState.RTL
            return False
        if self.wind_speed_est >= self.wind_warn:
            self.get_logger().warn(f'[WIND WARNING] {self.wind_speed_est:.1f}m/s')
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
                    self.state = FlightState.LANDING
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
            if self.detected:
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

            if not self.detected:
                self.miss_count += 1
                if self.miss_count >= self.lock_miss_limit:
                    self.miss_count = 0
                    self.state = FlightState.SEARCH
                    self.get_logger().info(
                        f'→ SEARCH ({self.lock_miss_limit}프레임 연속 미감지)')
                else:
                    self._publish_velocity_sp()  # 제자리 정지 유지
                return

            self.miss_count = 0

            # lock_on = YOLO 10프레임 확신(7/10) + 중앙 오차 100px 이내
            # → 표적 확인 + 중앙 정렬 동시 달성 시 True

            # 2m 이내 + lock_on → INTERCEPT 돌진
            if self.near_2m and self.lock_on:
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
            if not self.detected:
                self.miss_count += 1
                if self.miss_count >= self.lock_miss_limit:
                    self.miss_count = 0
                    self.state = FlightState.LANDING
                    self.get_logger().info('→ LANDING (표적 소실 — 요격 완료 또는 표적 소멸)')
                return
            self.miss_count = 0

            vx, vy, vz, yaw_rate = self._compute_velocity_cmd(self.v_intercept)
            self._publish_velocity_sp(vx, vy, vz, yaw_rate)
            self.get_logger().info(
                f'[INTERCEPT] dist={self.distance_m:.2f}m '
                f'vfwd={self.v_intercept}m/s yaw={yaw_rate:.2f}')

        elif self.state == FlightState.RTL:
            self.return_to_launch()
            self.state = FlightState.DONE
            self.get_logger().info('→ DONE (RTL)')

        elif self.state == FlightState.LANDING:
            self.land()
            self._print_stats()
            self.state = FlightState.DONE
            self.get_logger().info('→ DONE (착륙 명령)')

        elif self.state == FlightState.DONE:
            self._publish_velocity_sp()  # 0 유지

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


def main(args=None):
    rclpy.init(args=args)
    node = InterceptNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C → 착륙')
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
