#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

from interceptor_control.missions.hover.hover_logger import HoverLogger


class HoverMission(Node):
    def __init__(self):
        super().__init__('hover_mission')

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0
        self.hover_time = 30.0

        self.offboard_setpoint_counter = 0
        self.hover_start_time = None
        self.mission_start_time = self.get_clock().now()
        self.finished = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.state = "INIT"

        package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../..")
        )
        result_dir = os.path.join(package_root, "results")
        self.logger = HoverLogger(result_dir)

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

        self.get_logger().info("Hover Mission Node Started")

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

        if self.state in ["INIT", "TAKEOFF", "HOVER"]:
            target_z = self.target_z
        elif self.state == "DESCEND":
            target_z = -0.1
        else:
            target_z = self.current_z

        self.logger.log(
            mission_elapsed,
            self.current_x,
            self.current_y,
            self.current_z,
            self.target_x,
            self.target_y,
            target_z,
            self.current_vx,
            self.current_vy,
            self.current_vz,
            self.state
        )

        self.publish_offboard_control_mode()

        self.publish_position_setpoint(
            self.target_x,
            self.target_y,
            target_z
        )

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
            altitude_error = abs(self.current_z - self.target_z)

            if altitude_error < 0.3:
                self.get_logger().info("Target altitude reached. Start Hover.")
                self.hover_start_time = self.get_clock().now()
                self.state = "HOVER"

        elif self.state == "HOVER":
            elapsed = (
                self.get_clock().now() - self.hover_start_time
            ).nanoseconds / 1e9

            self.get_logger().info(
                f"Hovering... {elapsed:.1f}s / {self.hover_time:.1f}s | "
                f"pos=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})"
            )

            if elapsed >= self.hover_time:
                self.get_logger().info("Hover complete. Descending...")
                self.state = "DESCEND"

        elif self.state == "DESCEND":
            self.get_logger().info(
                f"Descending... current_z={self.current_z:.2f}"
            )

            if self.current_z > -0.25:
                self.get_logger().info("Ground reached. Disarming...")
                self.disarm()
                self.state = "END"

        elif self.state == "END":
            if not self.finished:
                csv_path, summary_path = self.logger.finish()
                self.get_logger().info("Hover mission finished.")
                self.get_logger().info(f"CSV saved: {csv_path}")
                self.get_logger().info(f"Summary saved: {summary_path}")
                self.finished = True

            self.timer.cancel()

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
        msg.position = [x, y, z]
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
    node = HoverMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
