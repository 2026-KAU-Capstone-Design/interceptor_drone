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
from px4_msgs.msg import VehicleAttitude

from interceptor_control.missions.high_speed.high_speed_logger import HighSpeedLogger

class HighSpeedVelocityMission(Node):
    def __init__(self):
        super().__init__('high_speed_velocity_mission')

        self.waypoints = [
            [0.0, 0.0, -5.0],  # Takeoff point
            [0.0, 0.0, -5.0],  # Circle start point
        ]

        self.current_wp_index = 0
        self.acceptance_radius = 0.5
        self.hold_time = 2.0

        # High-Speed Mission Parameters
        self.flight_altitude = -5.0

        self.target_speed = 20.0
        self.flight_distance = 30.0

        self.accel_distance = 5.0
        self.decel_distance = 5.0

        self.start_x = 0.0
        self.start_y = 0.0

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
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        self.state = "INIT"

        workspace_path = (
            Path.home() / "Documents" / "interceptor_drone" / "ros2_ws"
        )

        self.logger = HighSpeedLogger(workspace_path)

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


        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("High-Speed Velocity Mission Node Started")

    def vehicle_local_position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz

    def vehicle_attitude_callback(self, msg):
        # Quaternion (w, x, y, z)
        q = msg.q

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.current_roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        # Pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.current_pitch = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            self.current_pitch = math.degrees(math.asin(sinp))

        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def vehicle_status_callback(self, msg):
        pass

    def timer_callback(self):
        now = self.get_clock().now()
        mission_elapsed = (now - self.mission_start_time).nanoseconds / 1e9

        target_x, target_y, target_z = self.get_current_target()

        self.logger.log(
            mission_elapsed,
            self.state,
            self.current_x,
            self.current_y,
            self.current_z,
            target_x,
            target_y,
            target_z,
            self.current_vx,
            self.current_vy,
            self.current_vz,
            self.current_roll,
            self.current_pitch,
            self.current_yaw,
        )

        if self.state != "END":
            self.publish_offboard_control_mode()

            if self.state in [
                "ACCELERATE",
                "CRUISE",
                "DECELERATE",
                "RETURN_HOME",
            ]:
                commanded_vx = 0.0
                commanded_vy = 0.0
                commanded_vz = 0.0

                if self.state == "ACCELERATE":
                    elapsed = (
                        self.get_clock().now() - self.high_speed_start_time
                    ).nanoseconds / 1e9

                    commanded_vx = min(
                        self.target_speed,
                        self.target_speed * elapsed / 4.0,
                    )

                elif self.state == "CRUISE":
                    commanded_vx = self.target_speed

                elif self.state == "DECELERATE":
                    elapsed = (
                        self.get_clock().now() - self.decel_start_time
                    ).nanoseconds / 1e9

                    commanded_vx = max(
                        0.0,
                        self.target_speed * (1.0 - elapsed / 4.0),
                    )

                else:  # RETURN_HOME
                    error_x = -self.current_x
                    error_y = -self.current_y

                    distance_xy = math.sqrt(
                        error_x ** 2 + error_y ** 2
                    )

                    if distance_xy > 0.01:
                        return_speed = min(
                            3.0,
                            0.8 * distance_xy,
                        )

                        commanded_vx = (
                            return_speed * error_x / distance_xy
                        )
                        commanded_vy = (
                            return_speed * error_y / distance_xy
                        )

                    altitude_error = (
                        self.flight_altitude - self.current_z
                    )

                    commanded_vz = max(
                        -1.0,
                        min(1.0, 0.8 * altitude_error),
                    )

                if self.state in [
                    "ACCELERATE",
                    "CRUISE",
                    "DECELERATE",
                ]:
                    self.publish_high_speed_setpoint(
                        commanded_vx,
                        commanded_vy,
                        self.flight_altitude,
                        yaw=0.0,
                    )

                else:  # RETURN_HOME
                    self.publish_velocity_setpoint(
                        commanded_vx,
                        commanded_vy,
                        commanded_vz,
                        yaw=0.0,
                    )
            else:
                self.publish_position_setpoint(
                    target_x,
                    target_y,
                    target_z,
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
            dist = self.distance_to_target(target_x, target_y, target_z)

            if dist < self.acceptance_radius:
                self.get_logger().info("Takeoff point reached. Start waypoint mission.")
                self.state = "MOVE_TO_START"
                self.current_wp_index = 1
        elif self.state == "MOVE_TO_START":
            target_x, target_y, target_z = self.get_current_target()
            dist = self.distance_to_target(target_x, target_y, target_z)

            self.get_logger().info(
                f"Moving to start point: "
                f"target=({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) | "
                f"pos=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) | "
                f"dist={dist:.2f} m"
            )

            if dist < self.acceptance_radius:
                self.get_logger().info(
                    "High-speed start point reached. Accelerating..."
                )

                self.high_speed_start_time = self.get_clock().now()
                self.state = "ACCELERATE"

        elif self.state == "ACCELERATE":
            elapsed = (
                self.get_clock().now() - self.high_speed_start_time
            ).nanoseconds / 1e9

            if elapsed >= 4.0:
                self.get_logger().info("Acceleration complete. Cruising...")
                self.cruise_start_time = self.get_clock().now()
                self.state = "CRUISE"

        elif self.state == "CRUISE":
            elapsed = (
                self.get_clock().now() - self.cruise_start_time
            ).nanoseconds / 1e9

            if elapsed >= 8.0:
                self.get_logger().info("Cruise complete. Decelerating...")
                self.decel_start_time = self.get_clock().now()
                self.state = "DECELERATE"

        elif self.state == "DECELERATE":
            elapsed = (
                self.get_clock().now() - self.decel_start_time
            ).nanoseconds / 1e9

            if elapsed >= 4.0:
                self.get_logger().info("Deceleration complete. Returning home...")
                self.state = "RETURN_HOME"

        elif self.state == "RETURN_HOME":
            dist = self.distance_to_target(
                0.0,
                0.0,
                self.flight_altitude,
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

        if self.state == "ACCELERATE":
            return 10.0, 0.0, self.flight_altitude

        if self.state == "CRUISE":
            return self.flight_distance, 0.0, self.flight_altitude

        if self.state == "DECELERATE":
            return self.flight_distance, 0.0, self.flight_altitude
        if self.state == "RETURN_HOME":
            return 0.0, 0.0, self.flight_altitude

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

        if self.state in [
            "ACCELERATE",
            "CRUISE",
            "DECELERATE",
        ]:
            msg.position = True
            msg.velocity = False

        elif self.state == "RETURN_HOME":
            msg.position = False
            msg.velocity = True

        else:
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

    def publish_high_speed_setpoint(self, vx, vy, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = [
            float("nan"),
            float("nan"),
            float(z),
        ]

        msg.velocity = [
            float(vx),
            float(vy),
            float("nan"),
        ]

        msg.yaw = float(yaw)

        self.trajectory_setpoint_pub.publish(msg)


    def publish_velocity_setpoint(self, vx, vy, vz, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = float(yaw)

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
    node = HighSpeedVelocityMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
