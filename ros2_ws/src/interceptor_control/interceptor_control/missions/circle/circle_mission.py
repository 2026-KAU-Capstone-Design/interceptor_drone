#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

from interceptor_control.missions.circle.circle_logger import CircleLogger

class CircleMission(Node):
    def __init__(self):
        super().__init__('circle_mission')

        self.waypoints = [
            [0.0, 0.0, -5.0],  # Takeoff point
            [5.0, 0.0, -5.0],  # Circle start point
        ]

        self.current_wp_index = 0
        self.acceptance_radius = 0.5
        self.hold_time = 2.0

        # Circle Mission Parameters
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.circle_radius = 5.0

        self.circle_altitude = -5.0

        self.angular_speed = 0.35      # rad/s
        self.theta = 0.0
        self.start_theta = None

        self.circle_started = False
        self.circle_completed = False

        self.offboard_setpoint_counter = 0
        self.hold_start_time = None
        self.mission_start_time = self.get_clock().now()
        self.finished = False
        self.disarm_sent = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.state = "INIT"

        workspace_path = (
            Path.home() / "Documents" / "interceptor_drone" / "ros2_ws"
        )

        self.logger = CircleLogger(workspace_path)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Circle Mission Node Started")

    def vehicle_local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz

    def vehicle_status_callback(self, msg):
        pass

    def timer_callback(self):
        now = self.get_clock().now()
        mission_elapsed = (now - self.mission_start_time).nanoseconds / 1e9

        target_x, target_y, target_z = self.get_current_target()

        self.logger.log(
            mission_elapsed,
            self.state,
            self.theta,
            self.current_x,
            self.current_y,
            self.current_z,
            target_x,
            target_y,
            target_z,
            self.circle_center_x,
            self.circle_center_y,
            self.circle_radius,
            self.current_vx,
            self.current_vy,
            self.current_vz,
            0.0,   # roll
            0.0,   # pitch
            0.0,   # yaw
        )

        if self.state != "END":
            self.publish_offboard_control_mode()
            self.publish_position_setpoint(target_x, target_y, target_z)

        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 21:
            self.offboard_setpoint_counter += 1
            return

        if self.state == "INIT":
            self.get_logger().info("State: TAKEOFF")
            self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            dist = self.distance_to_target(target_x, target_y, target_z)

            if dist < self.acceptance_radius:
                self.get_logger().info("Takeoff point reached. Start waypoint mission.")
                self.state = "MOVE_TO_START"
                self.current_wp_index = 1

        elif self.state == "MOVE_TO_START":
            target_x, target_y, target_z = self.get_current_target()
            dist = self.distance_to_target(target_x, target_y, target_z)

            self.get_logger().info(
                f"Moving to WP{self.current_wp_index}: "
                f"target=({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) | "
                f"pos=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) | "
                f"dist={dist:.2f} m"
            )

            if dist < self.acceptance_radius:
                self.get_logger().info(
                   "Circle start point reached. Starting circle flight..."
                )

                self.circle_start_time = self.get_clock().now()
                self.theta = 0.0
                self.circle_started = True
                self.state = "CIRCLE"
        elif self.state == "CIRCLE":
            elapsed = (
                self.get_clock().now() - self.circle_start_time
            ).nanoseconds / 1e9

            self.theta = self.angular_speed * elapsed

            if self.theta >= 2.0 * math.pi:
                self.theta = 2.0 * math.pi

                self.logger.record_lap_complete(elapsed)

                self.get_logger().info(
                    f"Circle completed. Lap time: {elapsed:.2f} s"
                )

                self.state = "RETURN_HOME"

        elif self.state == "RETURN_HOME":
            dist = self.distance_to_target(
                0.0,
                0.0,
                self.circle_altitude,
            )

            self.get_logger().info(
                f"Returning Home... distance={dist:.2f} m"
            )

            if dist < self.acceptance_radius:
                self.get_logger().info(
                    "Home reached. Descending..."
                )
                self.state = "DESCEND"

        elif self.state == "DESCEND":
            self.get_logger().info(
                f"Descending... current_z={self.current_z:.2f}"
            )

            if self.current_z > -0.25 and not self.disarm_sent:
                self.get_logger().info("Ground reached. Disarming...")
                self.disarm()
                self.disarm_sent = True
                self.state = "END"

        elif self.state == "END":
            if not self.finished:
                csv_path, summary_path = self.logger.finish()
                self.get_logger().info("Circle mission finished.")
                self.get_logger().info(f"CSV saved: {csv_path}")
                self.get_logger().info(f"Summary saved: {summary_path}")
                self.finished = True

            self.timer.cancel()

    def get_current_target(self):
        if self.state == "DESCEND":
            return 0.0, 0.0, -0.1

        if self.state == "END":
            return self.current_x, self.current_y, self.current_z

        if self.state == "CIRCLE":
            target_x = (
                self.circle_center_x
                + self.circle_radius * math.cos(self.theta)
            )

            target_y = (
                self.circle_center_y
                + self.circle_radius * math.sin(self.theta)
            )

            return target_x, target_y, self.circle_altitude

        if self.state == "RETURN_HOME":
            return 0.0, 0.0, self.circle_altitude

        if self.current_wp_index >= len(self.waypoints):
            return 0.0, 0.0, -0.1

        return self.waypoints[self.current_wp_index]

    def distance_to_target(self, x, y, z):
        dx = self.current_x - x
        dy = self.current_y - y
        dz = self.current_z - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)

        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.get_logger().info("Disarm command sent")


def main(args=None):
    rclpy.init(args=args)
    node = CircleMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
