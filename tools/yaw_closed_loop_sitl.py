#!/usr/bin/env python3
"""PX4/Gazebo SITL only: small, bounded yaw-only visual closed-loop test.

No approach, translation, intercept, or autonomous search.
Never use on physical hardware. Requires --enable-flight to arm.
"""
import argparse
import math
import time
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from interceptor_interfaces.msg import TargetDetection, TargetDistance
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
NAN = float('nan')


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
    parser = argparse.ArgumentParser()
    parser.add_argument('--enable-flight', action='store_true', help='SIM ONLY: automatically ARM/take off after preflight')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = YawClosedLoopSITL(enable_flight=args.enable_flight)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.enable_flight and node.state in ('TAKEOFF', 'YAW_CONTROL'):
            node.request_land('Ctrl+C')
            # Give the outbound DDS writer time to transmit the request.
            time.sleep(0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
