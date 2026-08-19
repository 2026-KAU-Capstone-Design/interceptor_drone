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
from px4_msgs.msg import VehicleAttitudeSetpoint
from interceptor_control.missions.transition.transition_2.transition_2_logger import Transition2Logger

class Transition2Mission(Node):

    def __init__(self):
        super().__init__('transition_2_mission')

        # Flight parameters
        self.flight_altitude = -5.0
        self.acceptance_radius = 0.5

        # Pitch-step parameters [deg]
        self.pitch_steps = [
            0.0,
            -30.0,
        ]

        self.pitch_step_index = 0
        self.step_hold_time = 2.0

        # Current vehicle state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Command state
        self.commanded_pitch = 0.0

        # PX4 hover thrust parameter (MPC_THR_HOVER)
        self.hover_thrust = 0.60

        # Thrust compensation limit
        self.max_transition_thrust = 0.70

        self.offboard_setpoint_counter = 0
        self.state = "INIT"

        self.step_start_time = None
        self.hover_start_time = None

        self.finished = False
        self.disarm_sent = False

        self.mission_start_time = self.get_clock().now()

        # Transition 2 Logger
        workspace_path = Path(__file__).resolve().parents[6]

        self.logger = Transition2Logger(
            workspace_path=workspace_path
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )

        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile,
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile,
        )

        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile,
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.get_logger().info(
            "Transition 2 Pitch-Step Mission Node Started"
        )
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

        self.current_roll = math.degrees(
            math.atan2(sinr_cosp, cosr_cosp)
        )

        # Pitch
        sinp = 2.0 * (w * y - z * x)

        if abs(sinp) >= 1:
            self.current_pitch = math.degrees(
                math.copysign(math.pi / 2, sinp)
            )
        else:
            self.current_pitch = math.degrees(
                math.asin(sinp)
            )

        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        self.current_yaw = math.degrees(
            math.atan2(siny_cosp, cosy_cosp)
        )

    def vehicle_status_callback(self, msg):
        pass

    def timer_callback(self):
        now = self.get_clock().now()

        self.log_current_state()

        # Offboard setpoint를 먼저 일정 횟수 전송한 뒤 Arm / Offboard 진입
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 21:
            self.offboard_setpoint_counter += 1

            # 초기에는 지상 위치 유지
            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
            )
            return

        # -------------------------------------------------
        # INIT
        # -------------------------------------------------
        if self.state == "INIT":
            self.get_logger().info("State: TAKEOFF")
            self.state = "TAKEOFF"

        # -------------------------------------------------
        # TAKEOFF
        # -------------------------------------------------
        elif self.state == "TAKEOFF":

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
            )

            altitude_error = abs(
                self.current_z - self.flight_altitude
            )

            if altitude_error < self.acceptance_radius:
                self.get_logger().info(
                    "Takeoff complete. Hovering..."
                )

                self.hover_start_time = now
                self.state = "HOVER"

        # -------------------------------------------------
        # HOVER
        # -------------------------------------------------
        elif self.state == "HOVER":

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
            )

            hover_elapsed = (
                now - self.hover_start_time
            ).nanoseconds / 1e9

            if hover_elapsed >= 2.0:
                self.pitch_step_index = 1

                self.commanded_pitch = (
                    self.pitch_steps[self.pitch_step_index]
                )

                self.step_start_time = now
                self.state = "PITCH_STEP"

                self.get_logger().info(
                    f"Starting Pitch Step: "
                    f"{self.commanded_pitch:.1f} deg"
                )

        # -------------------------------------------------
        # PITCH STEP
        # -------------------------------------------------
        elif self.state == "PITCH_STEP":

            self.commanded_pitch = (
                self.pitch_steps[self.pitch_step_index]
            )

            self.publish_attitude_setpoint(
                self.commanded_pitch
            )

            step_elapsed = (
                now - self.step_start_time
            ).nanoseconds / 1e9

            self.get_logger().info(
                f"Pitch Step "
                f"{self.commanded_pitch:.1f} deg | "
                f"Actual {self.current_pitch:.1f} deg | "
                f"{step_elapsed:.1f}/{self.step_hold_time:.1f}s"
            )

            if step_elapsed >= self.step_hold_time:

                self.pitch_step_index += 1

                if self.pitch_step_index >= len(self.pitch_steps):

                    self.get_logger().info(
                        "Pitch steps complete. Recovering..."
                    )

                    self.commanded_pitch = 0.0
                    self.step_start_time = now
                    self.state = "RECOVERY"

                else:
                    self.commanded_pitch = (
                        self.pitch_steps[
                            self.pitch_step_index
                        ]
                    )

                    self.step_start_time = now

                    self.get_logger().info(
                        f"Next Pitch Step: "
                        f"{self.commanded_pitch:.1f} deg"
                    )

        # -------------------------------------------------
        # RECOVERY
        # -------------------------------------------------
        elif self.state == "RECOVERY":

            self.commanded_pitch = 0.0

            self.publish_attitude_setpoint(
                self.commanded_pitch
            )

            recovery_elapsed = (
                now - self.step_start_time
            ).nanoseconds / 1e9

            if recovery_elapsed >= 2.0:

                self.get_logger().info(
                    "Pitch recovery complete. Landing..."
                )

                self.state = "DESCEND"

        # -------------------------------------------------
        # DESCEND
        # -------------------------------------------------
        elif self.state == "DESCEND":

            self.publish_position_setpoint(
                0.0,
                0.0,
                -0.1,
            )

            if (
                self.current_z > -0.25
                and not self.disarm_sent
            ):
                self.get_logger().info(
                    "Ground reached. Disarming..."
                )

                self.disarm()
                self.disarm_sent = True
                self.state = "END"

        # -------------------------------------------------
        # END
        # -------------------------------------------------
        elif self.state == "END":

            if not self.finished:
                csv_path, summary_path = self.logger.finish()

                self.get_logger().info(
                    "Transition 2 Pitch-Step Mission Finished"
                )
                self.get_logger().info(
                    f"CSV saved: {csv_path}"
                )
                self.get_logger().info(
                    f"Summary saved: {summary_path}"
                )

                self.finished = True

            self.timer.cancel()

    def log_current_state(self):
        mission_elapsed = (
            self.get_clock().now() - self.mission_start_time
        ).nanoseconds / 1e9

        self.logger.log(
            mission_elapsed,
            self.state,

            self.current_x,
            self.current_y,
            self.current_z,

            self.current_vx,
            self.current_vy,
            self.current_vz,

            self.commanded_pitch,

            self.current_roll,
            self.current_pitch,
            self.current_yaw,
        )

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.state in ["PITCH_STEP", "RECOVERY"]:
            msg.position = False
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = True
            msg.body_rate = False
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

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)

    def publish_attitude_setpoint(self, pitch_deg):
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        pitch_rad = math.radians(pitch_deg)

        # Pitch 증가에 따른 수직 추력 감소 보상
        cos_pitch = math.cos(pitch_rad)

        if abs(cos_pitch) > 0.1:
            compensated_thrust = (
                self.hover_thrust / abs(cos_pitch)
            )
        else:
            compensated_thrust = self.max_transition_thrust

        compensated_thrust = min(
            compensated_thrust,
            self.max_transition_thrust,
        )

        # Roll = 0, Yaw = 0, Pitch only
        half_pitch = pitch_rad / 2.0

        msg.q_d = [
            math.cos(half_pitch),
            0.0,
            math.sin(half_pitch),
            0.0,
        ]

        # Multicopter thrust direction is -Z body axis
        msg.thrust_body = [
            0.0,
            0.0,
            -compensated_thrust,
        ]

        self.attitude_setpoint_pub.publish(msg)

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
            param2=6.0,
        )
        self.get_logger().info("Offboard mode command sent")


    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info("Arm command sent")


    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info("Disarm command sent")

def main(args=None):
    rclpy.init(args=args)

    node = Transition2Mission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()