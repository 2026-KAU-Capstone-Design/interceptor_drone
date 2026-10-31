#!/usr/bin/env python3
"""
Interceptor Drone — Offboard Hover Node (팀원 미션 템플릿)
==========================================================

Offboard 모드로 이륙 → 호버 → 착륙하는 기본 노드.
팀원들은 이 코드를 복사해서 미션별 코드를 작성합니다.

포함 기능:
  - 3-메시지 패턴 (OffboardControlMode + TrajectorySetpoint + VehicleCommand)
  - 비행 상태 머신 (IDLE → PREFLIGHT → ARMING → TAKEOFF → HOVER → LANDING → DONE)
  - 풍속 추정 + 강풍 시 자동 RTL/비상착륙
  - 자세 교란 감지 (기울기 > 30° 이후 2초 내 미복원 → 비상착륙)
  - 호버/착륙 오차 통계 출력

사용법:
  ros2 run interceptor_control offboard_hover
  ros2 run interceptor_control offboard_hover --ros-args -p target_altitude:=-10.0

파라미터 (config/mission_params.yaml 로 일괄 설정 가능):
  - target_altitude: 목표 고도 (NED, 음수=위) [기본: -5.0]
  - hover_duration:  호버 유지 시간(초)        [기본: 30.0]
  - wind_speed_warn: 풍속 경고 기준 (m/s)      [기본: 8.0]
  - wind_speed_critical: 강풍 RTL 기준 (m/s)   [기본: 15.0]
  - attitude_recovery_timeout: 자세 복원 타임아웃(초) [기본: 2.0]
  - position_threshold: 위치 도달 판정 거리 (m) [기본: 0.5]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from enum import Enum, auto

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleOdometry,
)


# =============================================================================
# 비행 상태 머신
# =============================================================================
class FlightState(Enum):
    IDLE       = auto()   # 시작 전
    PREFLIGHT  = auto()   # setpoint 사전 전송 (1초, PX4 offboard 진입 조건)
    ARMING     = auto()   # ARM + OFFBOARD 모드 전환
    TAKEOFF    = auto()   # 목표 고도까지 상승
    HOVER      = auto()   # 호버 유지
    MOVE_TO_B  = auto()   # B 지점으로 이동
    HOLD_B     = auto()   # B 지점에서 5초 정지
    RETURN_TO_A = auto()  # A 지점으로 복귀
    NAVIGATE   = auto()   # 이동 (팀원 미션 확장용)
    RTL        = auto()   # 자동 귀환 (강풍 등)
    LANDING    = auto()   # 착륙
    DONE       = auto()   # 완료


# =============================================================================
# QoS Profile (PX4 uXRCE-DDS 호환)
# =============================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OffboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')

        # ── ROS2 Parameters ──
        self.declare_parameter('target_altitude', -5.0)
        self.declare_parameter('hover_duration', 30.0)
        self.declare_parameter('wind_speed_warn', 8.0)
        self.declare_parameter('wind_speed_critical', 15.0)
        self.declare_parameter('attitude_recovery_timeout', 2.0)
        self.declare_parameter('position_threshold', 0.5)
        self.declare_parameter('tilt_threshold_deg', 30.0)

        self.target_alt       = self.get_parameter('target_altitude').value
        self.hover_duration   = self.get_parameter('hover_duration').value
        self.wind_warn        = self.get_parameter('wind_speed_warn').value
        self.wind_critical    = self.get_parameter('wind_speed_critical').value
        self.att_timeout      = self.get_parameter('attitude_recovery_timeout').value
        self.pos_threshold    = self.get_parameter('position_threshold').value
        self.tilt_threshold   = self.get_parameter('tilt_threshold_deg').value

        # ── Publishers (명령 → PX4) ──
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub  = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── Subscribers (PX4 → 텔레메트리) ──
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._cb_local_pos, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._cb_status, PX4_QOS)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._cb_odometry, PX4_QOS)

        # ── 내부 상태 ──
        self.state = FlightState.IDLE
        self.local_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.setpoint_counter = 0
        self.hover_start_time = None
        self.hold_b_start_time = None
        self.takeoff_xy = [0.0, 0.0]

        # Mission 2 목표 지점
        self.point_a = [0.0, 0.0]
        self.point_b = [20.0, 0.0]
        self.hold_b_duration = 5.0

        # 풍속/자세 안전 모니터링
        self.wind_speed_est = 0.0
        self.tilt_deg = 0.0
        self.attitude_disturbed_since = None

        # ── 10Hz 제어 루프 ──
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('=== Offboard Hover Node ===')
        self.get_logger().info(f'  고도     : {abs(self.target_alt):.1f}m')
        self.get_logger().info(f'  호버     : {self.hover_duration:.0f}s')
        self.get_logger().info(f'  풍속 경고: {self.wind_warn}m/s, 위험: {self.wind_critical}m/s')
        self.get_logger().info(f'  기울기 한계: {self.tilt_threshold}°, 복원 제한: {self.att_timeout}s')

    # =========================================================================
    # Subscriber Callbacks
    # =========================================================================

    def _cb_local_pos(self, msg):
        self.local_pos = msg
        if self.state == FlightState.HOVER and msg.v_xy_valid:
            self.wind_speed_est = float(np.sqrt(msg.vx**2 + msg.vy**2))

    def _cb_status(self, msg):
        self.vehicle_status = msg

    def _cb_odometry(self, msg):
        """VehicleOdometry의 쿼터니언에서 기울기(tilt) 추출"""
        q = msg.q  # [w, x, y, z]
        if len(q) < 4:
            return
        # 쿼터니언 → roll, pitch (라디안)
        sinr = 2.0 * (q[0] * q[1] + q[2] * q[3])
        cosr = 1.0 - 2.0 * (q[1]**2 + q[2]**2)
        roll = np.arctan2(sinr, cosr)

        sinp = np.clip(2.0 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0)
        pitch = np.arcsin(sinp)

        self.tilt_deg = float(np.degrees(np.sqrt(roll**2 + pitch**2)))
        self._check_attitude_safety()

    # =========================================================================
    # Safety Monitors
    # =========================================================================

    def _check_wind_safety(self):
        """풍속 기반 안전 판단 — 호버 중에만 유의미"""
        if self.state not in (FlightState.HOVER, FlightState.NAVIGATE):
            return True
        if self.wind_speed_est >= self.wind_critical:
            self.get_logger().error(
                f'[WIND CRITICAL] {self.wind_speed_est:.1f}m/s ≥ {self.wind_critical}m/s → RTL')
            self.state = FlightState.RTL
            return False
        if self.wind_speed_est >= self.wind_warn:
            self.get_logger().warn(
                f'[WIND WARNING] {self.wind_speed_est:.1f}m/s ≥ {self.wind_warn}m/s')
        return True

    def _check_attitude_safety(self):
        """자세 교란 감지 — 기울기 > threshold 이후 timeout 내 미복원 시 비상착륙"""
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
    # PX4 Command Helpers
    # =========================================================================

    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self, position=True):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x=0.0, y=0.0, z=-5.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0, param7=0.0):
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
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('[CMD] ARM')

    def engage_offboard(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('[CMD] OFFBOARD 모드 전환')

    def land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('[CMD] LAND')

    def return_to_launch(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('[CMD] RTL (자동 귀환)')

    # =========================================================================
    # Main Control Loop (10Hz)
    # =========================================================================

    def _control_loop(self):
        # ── 항상 OffboardControlMode 발행 (끊기면 PX4 fail-safe) ──
        self.publish_offboard_mode()

        # 현재 위치
        cur_x = float(self.local_pos.x)
        cur_y = float(self.local_pos.y)

        # 목표점까지 수평 거리 계산 함수
        def horizontal_distance_to(target_x, target_y):
            return float(np.sqrt((cur_x - target_x)**2 + (cur_y - target_y)**2))

        # ── 상태 머신 ──
        if self.state == FlightState.IDLE:
            # A 지점에서 5m 고도 이륙 준비
            self.publish_setpoint(x=0.0, y=0.0, z=self.target_alt)
            self.state = FlightState.PREFLIGHT
            self.setpoint_counter = 0
            self.get_logger().info('→ PREFLIGHT')

        elif self.state == FlightState.PREFLIGHT:
            self.publish_setpoint(x=0.0, y=0.0, z=self.target_alt)
            self.setpoint_counter += 1

            if self.setpoint_counter >= 10:  # 1초
                self.state = FlightState.ARMING
                self.get_logger().info('→ ARMING')

        elif self.state == FlightState.ARMING:
            self.publish_setpoint(x=0.0, y=0.0, z=self.target_alt)
            self.engage_offboard()
            self.arm()

            self.takeoff_xy = [self.local_pos.x, self.local_pos.y]
            self.point_a = [self.local_pos.x, self.local_pos.y]
            self.point_b = [self.point_a[0] + 20.0, self.point_a[1]]

            self.state = FlightState.TAKEOFF
            self.get_logger().info(f'→ TAKEOFF (목표 {abs(self.target_alt):.1f}m)')

        elif self.state == FlightState.TAKEOFF:
            # A 지점에서 목표 고도까지 상승
            self.publish_setpoint(x=self.point_a[0], y=self.point_a[1], z=self.target_alt)
            alt_err = abs(self.local_pos.z - self.target_alt)

            if alt_err < self.pos_threshold:
                self.state = FlightState.MOVE_TO_B
                self.get_logger().info(
                    f'→ MOVE_TO_B: B=({self.point_b[0]:.1f}, {self.point_b[1]:.1f}, {self.target_alt:.1f})'
                )

        elif self.state == FlightState.MOVE_TO_B:
            # B 지점으로 이동
            self.publish_setpoint(x=self.point_b[0], y=self.point_b[1], z=self.target_alt)
            self._check_wind_safety()

            dist_b = horizontal_distance_to(self.point_b[0], self.point_b[1])

            if dist_b < self.pos_threshold:
                self.hold_b_start_time = self.get_clock().now()
                self.state = FlightState.HOLD_B
                self.get_logger().info(
                    f'→ HOLD_B: B 도착, 오차={dist_b:.3f}m, 5초 정지'
                )

        elif self.state == FlightState.HOLD_B:
            # B 지점에서 5초 정지
            self.publish_setpoint(x=self.point_b[0], y=self.point_b[1], z=self.target_alt)
            self._check_wind_safety()

            elapsed = (self.get_clock().now() - self.hold_b_start_time).nanoseconds / 1e9

            if elapsed >= self.hold_b_duration:
                self.state = FlightState.RETURN_TO_A
                self.get_logger().info('→ RETURN_TO_A: A 지점으로 복귀')

        elif self.state == FlightState.RETURN_TO_A:
            # A 지점으로 복귀
            self.publish_setpoint(x=self.point_a[0], y=self.point_a[1], z=self.target_alt)
            self._check_wind_safety()

            dist_a = horizontal_distance_to(self.point_a[0], self.point_a[1])

            if dist_a < self.pos_threshold:
                self.state = FlightState.LANDING
                self.get_logger().info(
                    f'→ LANDING: A 복귀 완료, 오차={dist_a:.3f}m'
                )

        elif self.state == FlightState.RTL:
            self.return_to_launch()
            self.state = FlightState.DONE
            self.get_logger().info('→ DONE (RTL 실행됨)')

        elif self.state == FlightState.LANDING:
            self.land()
            self._print_landing_stats()
            self.state = FlightState.DONE
            self.get_logger().info('→ DONE (착륙 명령 전송됨)')

        elif self.state == FlightState.DONE:
            self.publish_setpoint(x=self.point_a[0], y=self.point_a[1], z=0.0)

    # =========================================================================
    # Statistics
    # =========================================================================

    def _print_hover_stats(self):
        dx = self.local_pos.x - self.takeoff_xy[0]
        dy = self.local_pos.y - self.takeoff_xy[1]
        drift = np.sqrt(dx**2 + dy**2)
        self.get_logger().info('──── 호버 통계 ────')
        self.get_logger().info(f'  수평 드리프트 : {drift:.3f}m (Δx={dx:.3f}, Δy={dy:.3f})')
        self.get_logger().info(f'  현재 고도     : {self.local_pos.z:.3f}m (목표: {self.target_alt})')
        self.get_logger().info(f'  추정 풍속     : {self.wind_speed_est:.2f}m/s')

    def _print_landing_stats(self):
        dx = self.local_pos.x - self.takeoff_xy[0]
        dy = self.local_pos.y - self.takeoff_xy[1]
        err = np.sqrt(dx**2 + dy**2)
        self.get_logger().info('──── 착륙 오차 ────')
        self.get_logger().info(f'  이륙 지점 : ({self.takeoff_xy[0]:.3f}, {self.takeoff_xy[1]:.3f})')
        self.get_logger().info(f'  현재 위치 : ({self.local_pos.x:.3f}, {self.local_pos.y:.3f})')
        self.get_logger().info(f'  수평 오차 : {err:.3f}m')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C → 착륙 명령')
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
