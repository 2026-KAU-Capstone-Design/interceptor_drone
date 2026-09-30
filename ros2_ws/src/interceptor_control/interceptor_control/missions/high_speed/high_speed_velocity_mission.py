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
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleRatesSetpoint
from px4_msgs.msg import VehicleAngularVelocity

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

        self.target_speed = 50.0

        # Cruise entry condition
        # CRUISE statistics begin only after the vehicle
        # is actually close to the 50 m/s target.
        self.accel_ramp_time = 6.0
        self.cruise_entry_speed = 49.8
        self.cruise_entry_hold_time = 2.0
        self.accel_timeout = 25.0
        self.cruise_ready_start_time = None

        # Transition Flight Parameters
        self.transition_pitch_deg = -50.0
        self.hover_thrust = 0.60
        self.transition_base_thrust = 0.93
        self.transition_duration = 4.5

        self.transition_recover_duration = 1.0
        self.pre_transition_pitch_deg = 0.0
        self.transition_recover_start_time = None

        self.decel_start_vy = 0.0
        self.decel_vy_recover_duration = 1.5

        # Pre-flight stabilization parameters
        self.stabilize_duration = 2.0

        self.stabilize_roll_limit = 5.0
        self.stabilize_pitch_limit = 5.0
        self.stabilize_yaw_limit = 5.0
        self.stabilize_xy_speed_limit = 0.3

        self.stabilize_ok_start_time = None
        self.stabilize_log_counter = 0

        # High-speed test mode
        # True  : 고속 시험 후 현 위치 착륙
        # False : 기존처럼 원점 복귀 후 착륙
        self.test_mode = True

        self.land_x = 0.0
        self.land_y = 0.0

        self.brake_hold_duration = 2.0
        self.brake_hold_start_time = None

        self.transition_kp_z = 0.10
        self.transition_kd_z = 0.10

        self.transition_min_thrust = 0.50
        self.transition_max_thrust = 1.00
        # Transition test
        self.transition_accel_ff = 2.5

        self.flight_distance = 30.0

        self.accel_distance = 5.0
        self.decel_distance = 5.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.offboard_setpoint_counter = 0
        self.hold_start_time = None
        self.transition_start_time = None
        self.decel_start_time = None
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
        self.initial_yaw_rad = None

        # PX4 controller thrust setpoint logging
        self.thrust_sp_x = 0.0
        self.thrust_sp_y = 0.0
        self.thrust_sp_z = 0.0
        self.thrust_sp_norm = 0.0

        # Controller tracking diagnostics
        self.pitch_sp_deg = 0.0
        self.pitch_rate_sp = 0.0
        self.pitch_rate_actual = 0.0

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

        self.vehicle_attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
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

        self.vehicle_thrust_setpoint_subscriber = self.create_subscription(
            VehicleThrustSetpoint,
            "/fmu/out/vehicle_thrust_setpoint",
            self.vehicle_thrust_setpoint_callback,
            qos_profile,
        )

        self.vehicle_attitude_setpoint_subscriber = self.create_subscription(
            VehicleAttitudeSetpoint,
            "/fmu/out/vehicle_attitude_setpoint",
            self.vehicle_attitude_setpoint_callback,
            qos_profile,
        )

        self.vehicle_rates_setpoint_subscriber = self.create_subscription(
            VehicleRatesSetpoint,
            "/fmu/out/vehicle_rates_setpoint",
            self.vehicle_rates_setpoint_callback,
            qos_profile,
        )

        self.vehicle_angular_velocity_subscriber = self.create_subscription(
            VehicleAngularVelocity,
            "/fmu/out/vehicle_angular_velocity",
            self.vehicle_angular_velocity_callback,
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
        
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        self.current_yaw = math.degrees(yaw_rad)

        # Save initial yaw once
        if self.initial_yaw_rad is None:
            self.initial_yaw_rad = yaw_rad

    def vehicle_thrust_setpoint_callback(self, msg):
        self.thrust_sp_x = float(msg.xyz[0])
        self.thrust_sp_y = float(msg.xyz[1])
        self.thrust_sp_z = float(msg.xyz[2])

        self.thrust_sp_norm = math.sqrt(
            self.thrust_sp_x ** 2
            + self.thrust_sp_y ** 2
            + self.thrust_sp_z ** 2
        )

    def vehicle_attitude_setpoint_callback(self, msg):
        # Desired attitude quaternion (w, x, y, z)
        q = msg.q_d

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        sinp = 2.0 * (w * y - z * x)

        if abs(sinp) >= 1.0:
            self.pitch_sp_deg = math.degrees(
                math.copysign(math.pi / 2.0, sinp)
            )
        else:
            self.pitch_sp_deg = math.degrees(math.asin(sinp))

    def vehicle_rates_setpoint_callback(self, msg):
        # PX4 body pitch-rate setpoint [rad/s]
        self.pitch_rate_sp = float(msg.pitch)

    def vehicle_angular_velocity_callback(self, msg):
        # Actual body pitch rate q [rad/s]
        self.pitch_rate_actual = float(msg.xyz[1])

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
            self.thrust_sp_x,
            self.thrust_sp_y,
            self.thrust_sp_z,
            self.thrust_sp_norm,
            self.pitch_sp_deg,
            self.current_pitch,
            self.pitch_rate_sp,
            self.pitch_rate_actual,
        )

        if self.state != "END":
            self.publish_offboard_control_mode()

            if self.state == "TRANSITION":
                self.publish_transition_attitude_setpoint(
                    self.transition_pitch_deg,
                    base_thrust=self.transition_base_thrust,
                )

            elif self.state == "TRANSITION_RECOVER":
                elapsed = (
                    self.get_clock().now()
                    - self.transition_recover_start_time
                ).nanoseconds / 1e9

                progress = min(
                    1.0,
                    elapsed / self.transition_recover_duration,
                )

                # -50 deg에서 천이 진입 직전 CRUISE pitch로
                # 부드럽게 복귀
                recover_pitch_deg = (
                    self.transition_pitch_deg
                    + (
                        self.pre_transition_pitch_deg
                        - self.transition_pitch_deg
                    )
                    * progress
                )

                # 복귀 목표 pitch에서 고도 유지를 위한
                # 기본 thrust 계산
                recover_pitch_rad = math.radians(
                    self.pre_transition_pitch_deg
                )

                recover_end_thrust = (
                    self.hover_thrust
                    / max(
                        0.1,
                        math.cos(recover_pitch_rad),
                    )
                )

                recover_end_thrust = max(
                    self.transition_min_thrust,
                    min(
                        self.transition_max_thrust,
                        recover_end_thrust,
                    ),
                )

                # Transition thrust → Cruise 자세용 thrust로
                # 부드럽게 감소
                recover_base_thrust = (
                    self.transition_base_thrust
                    + (
                        recover_end_thrust
                        - self.transition_base_thrust
                    )
                    * progress
                )

                self.publish_transition_attitude_setpoint(
                    recover_pitch_deg,
                    base_thrust=recover_base_thrust,
                )

            elif self.state in [
                "ACCELERATE",
                "CRUISE",
                "DECELERATE",
                "RETURN_HOME",
            ]:
                commanded_vx = 0.0
                commanded_vy = 0.0
                commanded_vz = 0.0
                commanded_ax = 0.0

                if self.state == "ACCELERATE":
                    elapsed = (
                        self.get_clock().now() - self.high_speed_start_time
                    ).nanoseconds / 1e9

                    commanded_vx = min(
                        self.target_speed,
                        self.target_speed * elapsed / self.accel_ramp_time,
                    )

                elif self.state == "CRUISE":
                    # Velocity Controller only:
                    # PX4 determines required acceleration / pitch / thrust
                    commanded_vx = self.target_speed

                elif self.state == "DECELERATE":
                    elapsed = (
                        self.get_clock().now() - self.decel_start_time
                    ).nanoseconds / 1e9

                    commanded_vx = max(
                        0.0,
                        self.target_speed * (1.0 - elapsed / 8.0),
                    )

                    vy_progress = min(
                        1.0,
                        elapsed / self.decel_vy_recover_duration,
                    )

                    commanded_vy = (
                        self.decel_start_vy
                        * (1.0 - vy_progress)
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
                        ax=commanded_ax,
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
                    "High-speed start point reached. Stabilizing..."
                )

                self.stabilize_ok_start_time = None
                self.stabilize_log_counter = 0

                self.state = "STABILIZE"

        elif self.state == "STABILIZE":
            xy_speed = math.sqrt(
                self.current_vx ** 2
                + self.current_vy ** 2
            )

            # Yaw error relative to world-frame 0 deg
            yaw_error = (
                (self.current_yaw + 180.0) % 360.0
                - 180.0
            )

            is_stable = (
                abs(self.current_roll)
                <= self.stabilize_roll_limit
                and abs(self.current_pitch)
                <= self.stabilize_pitch_limit
                and abs(yaw_error)
                <= self.stabilize_yaw_limit
                and xy_speed
                <= self.stabilize_xy_speed_limit
            )

            # About once per second
            self.stabilize_log_counter += 1

            if self.stabilize_log_counter >= 20:
                self.get_logger().info(
                    f"Stabilizing | "
                    f"Roll={self.current_roll:.2f} deg | "
                    f"Pitch={self.current_pitch:.2f} deg | "
                    f"Yaw={self.current_yaw:.2f} deg | "
                    f"YawErr={yaw_error:.2f} deg | "
                    f"XY Speed={xy_speed:.2f} m/s"
                )

                self.stabilize_log_counter = 0

            if is_stable:
                if self.stabilize_ok_start_time is None:
                    self.stabilize_ok_start_time = (
                        self.get_clock().now()
                    )

                    self.get_logger().info(
                        "Stable condition detected. "
                        "Holding for 2.0 seconds..."
                    )

                stable_elapsed = (
                    self.get_clock().now()
                    - self.stabilize_ok_start_time
                ).nanoseconds / 1e9

                if stable_elapsed >= self.stabilize_duration:
                    self.get_logger().info(
                        "Stabilization complete. Accelerating..."
                    )

                    self.high_speed_start_time = (
                        self.get_clock().now()
                    )

                    self.state = "ACCELERATE"

            else:
                # 조건을 하나라도 벗어나면
                # 2초 안정화 타이머 다시 시작
                self.stabilize_ok_start_time = None

        elif self.state == "ACCELERATE":
            now = self.get_clock().now()

            elapsed = (
                now - self.high_speed_start_time
            ).nanoseconds / 1e9

            speed_xy = math.hypot(
                self.current_vx,
                self.current_vy,
            )

            # First 6 s:
            # velocity setpoint ramps from 0 -> 50 m/s.
            #
            # After 6 s:
            # continue commanding 50 m/s until the actual
            # speed reaches the cruise-entry threshold.
            if (
                elapsed >= self.accel_ramp_time
                and speed_xy >= self.cruise_entry_speed
            ):
                if self.cruise_ready_start_time is None:
                    self.cruise_ready_start_time = now

                    self.get_logger().info(
                        f"Near target speed: {speed_xy:.2f} m/s. "
                        f"Holding >= {self.cruise_entry_speed:.1f} m/s "
                        f"for {self.cruise_entry_hold_time:.1f} s..."
                    )

                ready_elapsed = (
                    now - self.cruise_ready_start_time
                ).nanoseconds / 1e9

                if ready_elapsed >= self.cruise_entry_hold_time:
                    self.get_logger().info(
                        f"Cruise-entry speed stabilized: "
                        f"{speed_xy:.2f} m/s. Starting CRUISE..."
                    )

                    self.cruise_start_time = now
                    self.state = "CRUISE"

            else:
                # Continuous 2 s condition:
                # reset timer whenever speed drops below threshold.
                self.cruise_ready_start_time = None

            # If 50 m/s-class flight cannot be reached,
            # do not include the acceleration period in CRUISE statistics.
            if (
                self.state == "ACCELERATE"
                and elapsed >= self.accel_timeout
            ):
                self.get_logger().warning(
                    f"Cruise-entry timeout: "
                    f"required >= {self.cruise_entry_speed:.1f} m/s, "
                    f"actual={speed_xy:.2f} m/s. "
                    "Skipping CRUISE and decelerating."
                )

                self.decel_start_vy = self.current_vy
                self.decel_start_time = now
                self.state = "DECELERATE"

        elif self.state == "CRUISE":
            elapsed = (
                self.get_clock().now() - self.cruise_start_time
            ).nanoseconds / 1e9

            if elapsed >= 20.0:
                self.get_logger().info(
                    "Velocity-only high-speed hold complete. Decelerating..."
                )

                self.decel_start_vy = self.current_vy
                self.decel_start_time = self.get_clock().now()
                self.state = "DECELERATE"

        elif self.state == "TRANSITION":
            elapsed = (
                self.get_clock().now() - self.transition_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.transition_duration:
                self.get_logger().info(
                    "Transition complete. Recovering attitude..."
                )

                self.transition_recover_start_time = (
                    self.get_clock().now()
                )

                self.state = "TRANSITION_RECOVER"

        elif self.state == "TRANSITION_RECOVER":
            elapsed = (
                self.get_clock().now()
                - self.transition_recover_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.transition_recover_duration:
                self.get_logger().info(
                    "Attitude recovery complete. Decelerating..."
                )

                self.decel_start_vy = self.current_vy
                self.decel_start_time = self.get_clock().now()
                self.state = "DECELERATE"

        elif self.state == "DECELERATE":
            elapsed = (
                self.get_clock().now()
                - self.decel_start_time
            ).nanoseconds / 1e9

            if elapsed >= 8.0:
                if self.test_mode:
                    self.land_x = self.current_x
                    self.land_y = self.current_y

                    self.get_logger().info(
                        f"Deceleration complete. "
                        f"Test mode: holding at current position "
                        f"({self.land_x:.2f}, {self.land_y:.2f})"
                    )

                    self.brake_hold_start_time = self.get_clock().now()
                    self.state = "BRAKE_HOLD"

                else:
                    self.get_logger().info(
                        "Deceleration complete. Returning home..."
                    )
                    self.state = "RETURN_HOME"

        elif self.state == "BRAKE_HOLD":
            elapsed = (
                self.get_clock().now()
                - self.brake_hold_start_time
            ).nanoseconds / 1e9

            self.get_logger().info(
                f"Brake hold... {elapsed:.1f} / "
                f"{self.brake_hold_duration:.1f} s"
            )

            if elapsed >= self.brake_hold_duration:
                self.get_logger().info(
                    "Brake hold complete. Descending..."
                )
                self.state = "DESCEND"

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
                self.get_logger().info("High-speed mission finished.")
                self.get_logger().info(f"CSV saved: {csv_path}")
                self.get_logger().info(f"Summary saved: {summary_path}")
                self.finished = True

            self.timer.cancel()

    def get_current_target(self):
        if self.state == "BRAKE_HOLD":
            return self.land_x, self.land_y, self.flight_altitude

        if self.state == "DESCEND":
            if self.test_mode:
                return self.land_x, self.land_y, -0.1

            return 0.0, 0.0, -0.1

        if self.state == "END":
            return self.current_x, self.current_y, self.current_z

        if self.state == "STABILIZE":
            return 0.0, 0.0, self.flight_altitude

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

        # Default: all control modes disabled
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        # Attitude-based transition flight
        if self.state in [
            "TRANSITION",
            "TRANSITION_RECOVER",
        ]:
            msg.attitude = True
        # High-speed velocity control + altitude position control
        elif self.state in [
            "ACCELERATE",
            "CRUISE",
            "DECELERATE",
        ]:
            msg.position = True

        # Return home using velocity control
        elif self.state == "RETURN_HOME":
            msg.velocity = True

        # TAKEOFF / HOLD / DESCEND etc.
        else:
            msg.position = True

        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        if self.state == "STABILIZE":
            msg.yaw = 0.0

        elif self.initial_yaw_rad is not None:
            msg.yaw = float(self.initial_yaw_rad)

        else:
            msg.yaw = float("nan")

        self.trajectory_setpoint_pub.publish(msg)



    def publish_high_speed_setpoint(
        self,
        vx,
        vy,
        z,
        ax=0.0,
        yaw=0.0,
    ):
        msg = TrajectorySetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

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

        msg.acceleration = [
            float(ax),
            0.0,
            float("nan"),
        ]

        msg.yaw = float(yaw)

        self.trajectory_setpoint_pub.publish(msg)
    def publish_transition_attitude_setpoint(
        self,
        pitch_deg,
        base_thrust=None,
    ):
        msg = VehicleAttitudeSetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        pitch_rad = math.radians(pitch_deg)

        # roll = 0, yaw = 0, pitch only
        msg.q_d = [
            math.cos(pitch_rad / 2.0),
            0.0,
            math.sin(pitch_rad / 2.0),
            0.0,
        ]

        # Tilt compensation
        if base_thrust is None:
            thrust_mag = self.transition_base_thrust
        else:
            thrust_mag = base_thrust

        # Altitude compensation (NED)
        altitude_error = self.current_z - self.flight_altitude

        thrust_mag += (
            self.transition_kp_z * altitude_error
            + self.transition_kd_z * self.current_vz
        )

        thrust_mag = max(
            self.transition_min_thrust,
            min(self.transition_max_thrust, thrust_mag),
        )

        msg.thrust_body = [
            0.0,
            0.0,
            -float(thrust_mag),
        ]

        msg.yaw_sp_move_rate = 0.0
        msg.reset_integral = False
        msg.fw_control_yaw_wheel = False

        self.vehicle_attitude_setpoint_pub.publish(msg)

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
