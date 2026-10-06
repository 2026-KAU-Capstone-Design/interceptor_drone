#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Float32MultiArray, Bool
from interceptor_control.missions.tracking.rocket_track_logger import RocketTrackLogger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
)


NAN = float("nan")


class MovingTargetFollowMission(Node):

    def __init__(self):
        super().__init__("moving_target_follow_mission")

        # ============================================================
        # Mission parameters
        # ============================================================

        self.flight_altitude = -5.0
        self.acceptance_radius = 0.5

        # YOLO / tracking parameters
        self.image_w = 1280.0
        self.image_h = 960.0

        # dx normalized error -> yaw rate
        self.kp_yaw = 2.0

        # Maximum yaw rate [rad/s]
        self.max_yaw_rate = 2.0

        # Center deadband [pixel]
        self.yaw_deadband_px = 30.0

        # Detection loss tolerance
        # timer = 20 Hz, 20 counts = approx 1 sec
        self.miss_limit = 20

        # ============================================================
        # Vehicle state
        # ============================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # ============================================================
        # YOLO state
        # ============================================================

        self.detected = False
        self.bbox_dx = 0.0
        self.bbox_dy = 0.0
        self.miss_count = 0
        self.new_bbox = False
        # ============================================================
        # Mission state
        # ============================================================

        self.state = "INIT"
        self.near_2m = False
        self.lock_on = False
        self.offboard_setpoint_counter = 0
        self.hover_start_time = None

        # Current / target yaw [rad]
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_initialized = False

        # Moving-target tracking
        self.last_yaw_rate_cmd = 0.0

        # 3 consecutive YOLO frames -> FOLLOW
        self.acquire_count = 0
        self.acquire_required = 3

        # FOLLOW speeds [m/s]
        self.follow_speed = 3.0
        self.follow_mid_speed = 2.0
        self.follow_min_speed = 2.0

        # CHASE control
        self.chase_speed = 12.0
        self.chase_align_px = 120.0
        self.chase_abort_px = 300.0
        self.chase_align_count = 0
        self.chase_align_required = 8

        # Allow short YOLO detection gaps during CHASE
        # timer = 20 Hz -> 5 cycles ~= 0.25 s
        self.chase_miss_count = 0
        self.chase_miss_limit = 5
         # Vertical tracking
        self.target_z = self.flight_altitude
        # Vertical tracking
        self.kp_z_track = 0.0
        self.max_z_rate_track = 0.35

        self.kp_z_approach = 0.6
        self.max_z_rate_approach = 0.25

        self.z_deadband_px = 40.0

        # PX4 NED altitude limits
        self.min_target_z = -10.0
        self.max_target_z = -2.5
        # Approach control
        self.approach_speed = 4.0
        self.approach_lock_px = 40.0
       
        # Control period = 20 Hz
        self.control_dt = 0.05
        # ============================================================
        # Mission Logger
        # ============================================================

        workspace_path = (
            Path.home()
            / "Documents"
            / "interceptor_drone"
            / "ros2_ws"
        )

        self.mission_logger = RocketTrackLogger(
            str(workspace_path)
        )

        # ============================================================
        # QoS
        # ============================================================
        # ============================================================
        # QoS
        # ============================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # PX4 Publishers
        # ============================================================

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        # ============================================================
        # PX4 Subscribers
        # ============================================================

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            px4_qos,
        )
        # ============================================================
        # YOLO Subscriber
        # ============================================================

        self.create_subscription(
            Float32MultiArray,
            "/target/balloon_bbox",
            self.bbox_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/target/near_2m",
            self.near_2m_callback,
            10,
        )
 
        self.create_subscription(
            Bool,
            "/target/lock_on",
            self.lock_on_callback,
            10,
        )
        # 20 Hz control loop
        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.get_logger().info(
            "=== Rocket Track Mission V1 : YAW ONLY ==="
        )

        self.get_logger().info(
            f"Takeoff altitude : {abs(self.flight_altitude):.1f} m"
        )

        self.get_logger().info(
            f"Kp_yaw={self.kp_yaw}, "
            f"deadband={self.yaw_deadband_px:.0f}px, "
            f"max_yaw_rate={self.max_yaw_rate:.2f}rad/s"
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def vehicle_local_position_callback(self, msg):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
    def vehicle_attitude_callback(self, msg):

        q = msg.q

        if len(q) < 4:
            return

        # PX4 quaternion = [w, x, y, z]
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)

        self.current_roll = math.atan2(
            sinr_cosp,
            cosr_cosp,
        )

        # Pitch
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))

        self.current_pitch = math.asin(sinp)

        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        self.current_yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        # Capture initial vehicle heading once
        if not self.yaw_initialized:
            self.target_yaw = self.current_yaw
            self.yaw_initialized = True

            self.get_logger().info(
                f"Initial yaw captured: "
                f"{math.degrees(self.target_yaw):+.1f} deg"
            )
    def near_2m_callback(self, msg):
        self.near_2m = bool(msg.data)

    def lock_on_callback(self, msg):
        self.lock_on = bool(msg.data)

    def bbox_callback(self, msg):

        data = msg.data

        if len(data) < 15:
            return

        self.detected = data[0] > 0.5

        self.bbox_dx = float(data[8])
        self.bbox_dy = float(data[9])

        if data[10] > 0:
            self.image_w = float(data[10])

        if data[11] > 0:
            self.image_h = float(data[11])
        self.new_bbox = True
    # ================================================================
    # Main control loop
    # ================================================================

    def timer_callback(self):

        if not self.yaw_initialized:
            return

        log_time = self.get_clock().now().nanoseconds / 1e9
        
        self.publish_offboard_control_mode(
            position=True,
            velocity=True,
        )

        # ------------------------------------------------------------
        # Send setpoints before Offboard activation
        # ------------------------------------------------------------

        if self.offboard_setpoint_counter < 20:

            # Offboard 진입 전에는 현재 기체 방향을 계속 따라감
            self.target_yaw = self.current_yaw

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
                yaw=self.target_yaw,
            )

            self.offboard_setpoint_counter += 1

            return

        if self.offboard_setpoint_counter == 20:

            # Target UAV is located in Gazebo +X direction
            self.target_yaw = math.radians(90.0)

            self.get_logger().info(
                f"Takeoff yaw locked: "
                f"{math.degrees(self.target_yaw):+.1f} deg"
            )

            self.engage_offboard_mode()
            self.arm()

            self.offboard_setpoint_counter += 1

            self.get_logger().info(
                "Offboard + ARM command sent"
            )

            return

        # ============================================================
        # State machine
        # ============================================================

        if self.state == "INIT":

            self.state = "TAKEOFF"

            self.get_logger().info(
                "State: TAKEOFF"
            )


        # ------------------------------------------------------------
        # TAKEOFF
        # ------------------------------------------------------------

        elif self.state == "TAKEOFF":

            self.get_logger().info(
                f"[TAKEOFF] current_yaw="
                f"{math.degrees(self.current_yaw):+.1f}deg | "
                f"target_yaw="
                f"{math.degrees(self.target_yaw):+.1f}deg"
            )

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
                yaw=self.target_yaw,
            )

            altitude_error = abs(
                self.current_z - self.flight_altitude
            )

            if altitude_error < self.acceptance_radius:

                self.hover_start_time = (
                    self.get_clock().now()
                )

                self.state = "HOVER"

                self.get_logger().info(
                    "Takeoff complete -> HOVER"
                )

        # ------------------------------------------------------------
        # HOVER
        # ------------------------------------------------------------

        elif self.state == "HOVER":

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
                yaw=self.target_yaw,
            )

            elapsed = (
                self.get_clock().now()
                - self.hover_start_time
            ).nanoseconds / 1e9

            if elapsed >= 2.0:

                self.state = "SEARCH"

                self.get_logger().info(
                    "Hover complete -> WAIT_TARGET"
                )

        # ------------------------------------------------------------
        # WAIT TARGET
        # ------------------------------------------------------------

        # ------------------------------------------------------------
        # SEARCH
        # ------------------------------------------------------------
        elif self.state == "SEARCH":

            self.last_yaw_rate_cmd = 0.0

            self.publish_approach_setpoint(
                0.0,
                0.0,
                self.target_z,
                self.current_yaw,
            )

            if self.detected and self.new_bbox:

                self.new_bbox = False
                self.acquire_count = 1
                self.state = "ACQUIRE"

                self.get_logger().info(
                    "[SEARCH] Object detected "
                    f"-> ACQUIRE "
                    f"(1/{self.acquire_required})"
                )

        # ------------------------------------------------------------
        # ACQUIRE
        # 3 consecutive detections before FOLLOW
        # ------------------------------------------------------------
        elif self.state == "ACQUIRE":

            if not self.detected:

                self.acquire_count = 0
                self.last_yaw_rate_cmd = 0.0
                self.state = "SEARCH"

                self.get_logger().info(
                    "[ACQUIRE] Detection lost -> SEARCH"
                )
                return

            if not self.new_bbox:

                self.publish_approach_setpoint(
                    0.0,
                    0.0,
                    self.target_z,
                    self.current_yaw,
                )
                return

            self.new_bbox = False

            dx = self.bbox_dx
            dy = self.bbox_dy

            # Horizontal image error -> direct yaw-rate
            if abs(dx) <= self.yaw_deadband_px:
                yaw_rate = 0.0

            else:
                normalized_dx = (
                    dx / (self.image_w / 2.0)
                )

                yaw_rate = (
                    self.kp_yaw * normalized_dx
                )

                yaw_rate = max(
                    -self.max_yaw_rate,
                    min(
                        self.max_yaw_rate,
                        yaw_rate,
                    ),
                )

            self.last_yaw_rate_cmd = yaw_rate

            # --------------------------------------------------------
            # Moving-target vertical tracking
            # image dy -> NED altitude setpoint
            # --------------------------------------------------------
            if abs(dy) <= self.z_deadband_px:
                z_rate = 0.0

            else:
                normalized_dy = (
                    dy / (self.image_h / 2.0)
                )

                z_rate = (
                    self.kp_z_track
                    * normalized_dy
                )

                z_rate = max(
                    -self.max_z_rate_track,
                    min(
                        self.max_z_rate_track,
                        z_rate,
                    ),
                )

            self.target_z += (
                z_rate * self.control_dt
            )

            self.target_z = max(
                self.min_target_z,
                min(
                    self.max_target_z,
                    self.target_z,
                ),
            )

            # ACQUIRE에서는 전진하지 않음
            self.publish_approach_setpoint(
                0.0,
                0.0,
                self.target_z,
                self.current_yaw,
            )

            self.acquire_count += 1

            self.get_logger().info(
                f"[ACQUIRE] "
                f"{self.acquire_count}/"
                f"{self.acquire_required} | "
                f"dx={dx:+.1f}px | "
                f"dy={dy:+.1f}px | "
                f"yaw_rate={yaw_rate:+.3f}rad/s"
            )

            if self.acquire_count >= self.acquire_required:

                self.miss_count = 0
                self.state = "FOLLOW"

                self.get_logger().info(
                    "[ACQUIRE] Confirmed -> FOLLOW"
                )

        # ------------------------------------------------------------
        # FOLLOW
        # Keep moving while tracking the object
        # ------------------------------------------------------------
        elif self.state == "FOLLOW":

            if not self.detected:

                self.miss_count += 1
                self.last_yaw_rate_cmd = 0.0
                self.chase_align_count = 0

                self.publish_approach_setpoint(
                    0.0,
                    0.0,
                    self.target_z,
                    self.current_yaw,
                )

                if self.miss_count >= self.miss_limit:

                    self.miss_count = 0
                    self.acquire_count = 0
                    self.state = "SEARCH"

                    self.get_logger().warn(
                        "[FOLLOW] Target lost -> SEARCH"
                    )

                    self.mission_logger.mark_target_lost()

                return

            self.miss_count = 0

            # New YOLO frame
            if self.new_bbox:

                self.new_bbox = False

                dx = self.bbox_dx
                dy = self.bbox_dy

                if abs(dx) <= self.yaw_deadband_px:
                    yaw_rate = 0.0

                else:
                    normalized_dx = (
                        dx / (self.image_w / 2.0)
                    )

                    yaw_rate = (
                        self.kp_yaw * normalized_dx
                    )

                    yaw_rate = max(
                        -self.max_yaw_rate,
                        min(
                            self.max_yaw_rate,
                            yaw_rate,
                        ),
                    )

                self.last_yaw_rate_cmd = yaw_rate

                # --------------------------------------------------------
                # FOLLOW -> CHASE qualification
                # Count only NEW YOLO frames.
                # --------------------------------------------------------
                if abs(dx) <= self.chase_align_px:
                    self.chase_align_count += 1
                else:
                    self.chase_align_count = 0

                # --------------------------------------------------------
                # Moving-target vertical tracking
                # image dy -> NED altitude setpoint
                # --------------------------------------------------------
                if abs(dy) <= self.z_deadband_px:
                    z_rate = 0.0

                else:
                    normalized_dy = (
                        dy / (self.image_h / 2.0)
                    )

                    z_rate = (
                        self.kp_z_track
                        * normalized_dy
                    )

                    z_rate = max(
                        -self.max_z_rate_track,
                        min(
                            self.max_z_rate_track,
                            z_rate,
                        ),
                    )

                self.target_z += (
                    z_rate * self.control_dt
                )

                self.target_z = max(
                    self.min_target_z,
                    min(
                        self.max_target_z,
                        self.target_z,
                    ),
                )

            else:

                dx = self.bbox_dx
                dy = self.bbox_dy
                yaw_rate = self.last_yaw_rate_cmd

            # --------------------------------------------------------
            # FOLLOW speed
            #
            # 이전처럼 오차가 커졌다고 0 m/s로 멈추지 않는다.
            # --------------------------------------------------------
            abs_dx = abs(dx)

            if abs_dx <= 120.0:
                forward_speed = self.follow_speed

            elif abs_dx <= 250.0:
                forward_speed = self.follow_mid_speed

            else:
                forward_speed = self.follow_min_speed

            # 현재 기수방향으로 계속 이동
            vx = (
                forward_speed
                * math.cos(self.current_yaw)
            )

            vy = (
                forward_speed
                * math.sin(self.current_yaw)
            )

            self.publish_approach_setpoint(
                vx,
                vy,
                self.target_z,
                self.current_yaw,
            )

            self.get_logger().info(
                f"[FOLLOW] "
                f"dx={dx:+.1f}px | "
                f"dy={dy:+.1f}px | "
                f"yaw_rate={yaw_rate:+.3f}rad/s | "
                f"speed={forward_speed:.1f}m/s | "
                f"z_sp={self.target_z:.2f}m"
            )

            self.mission_logger.log(
                log_time,
                self.state,
                self.detected,
                self.lock_on,
                self.near_2m,
                self.current_x,
                self.current_y,
                self.current_z,
                dx,
                dy,
                self.current_yaw,
                self.current_yaw,
                self.target_z,
                yaw_rate=yaw_rate,
                forward_speed=forward_speed,
                vx=vx,
                vy=vy,
                roll_actual=self.current_roll,
                pitch_actual=self.current_pitch,
                yaw_actual=self.current_yaw,
            )

            if (
                self.chase_align_count
                >= self.chase_align_required
            ):
                self.chase_align_count = 0
                self.state = "CHASE"

                self.get_logger().info(
                    "[FOLLOW] Stable target -> CHASE "
                    f"({self.chase_speed:.1f} m/s)"
                )

        # ------------------------------------------------------------
        # CHASE
        # Target has been stable in FOLLOW -> accelerate to 5 m/s.
        # If horizontal tracking degrades, fall back to FOLLOW.
        # ------------------------------------------------------------
        elif self.state == "CHASE":

            if not self.detected:

                self.chase_miss_count += 1

                # ----------------------------------------------------
                # Short YOLO gap:
                # keep the previous yaw-rate and keep chasing.
                # ----------------------------------------------------
                if (
                    self.chase_miss_count
                    <= self.chase_miss_limit
                ):

                    forward_speed = self.chase_speed

                    vx = (
                        forward_speed
                        * math.cos(self.current_yaw)
                    )

                    vy = (
                        forward_speed
                        * math.sin(self.current_yaw)
                    )

                    self.publish_approach_setpoint(
                        vx,
                        vy,
                        self.target_z,
                        self.current_yaw,
                    )

                    self.get_logger().warn(
                        f"[CHASE] Detection gap "
                        f"{self.chase_miss_count}/"
                        f"{self.chase_miss_limit} | "
                        f"holding {forward_speed:.1f} m/s"
                    )

                    return

                # ----------------------------------------------------
                # Detection missing too long -> FOLLOW
                # ----------------------------------------------------
                self.chase_miss_count = 0
                self.chase_align_count = 0
                self.state = "FOLLOW"
                self.last_yaw_rate_cmd = 0.0

                forward_speed = self.follow_min_speed

                vx = (
                    forward_speed
                    * math.cos(self.current_yaw)
                )

                vy = (
                    forward_speed
                    * math.sin(self.current_yaw)
                )

                self.publish_approach_setpoint(
                    vx,
                    vy,
                    self.target_z,
                    self.current_yaw,
                )

                self.get_logger().warn(
                    "[CHASE] Detection lost too long "
                    "-> FOLLOW"
                )

                return

            # Detection recovered / available
            self.chase_miss_count = 0

            if self.new_bbox:

                self.new_bbox = False

                dx = self.bbox_dx
                dy = self.bbox_dy

                if abs(dx) <= self.yaw_deadband_px:
                    yaw_rate = 0.0

                else:
                    normalized_dx = (
                        dx / (self.image_w / 2.0)
                    )

                    yaw_rate = (
                        self.kp_yaw * normalized_dx
                    )

                    yaw_rate = max(
                        -self.max_yaw_rate,
                        min(
                            self.max_yaw_rate,
                            yaw_rate,
                        ),
                    )

                self.last_yaw_rate_cmd = yaw_rate

            else:

                dx = self.bbox_dx
                dy = self.bbox_dy
                yaw_rate = self.last_yaw_rate_cmd

            # Tracking degraded -> slow down and return to FOLLOW
            if abs(dx) > self.chase_abort_px:

                self.chase_align_count = 0
                self.state = "FOLLOW"

                forward_speed = self.follow_min_speed

                vx = (
                    forward_speed
                    * math.cos(self.current_yaw)
                )

                vy = (
                    forward_speed
                    * math.sin(self.current_yaw)
                )

                self.publish_approach_setpoint(
                    vx,
                    vy,
                    self.target_z,
                    self.current_yaw,
                )

                self.get_logger().warn(
                    f"[CHASE] dx={dx:+.1f}px "
                    "-> FOLLOW"
                )

                return

            # CHASE speed = 5 m/s
            forward_speed = self.chase_speed

            vx = (
                forward_speed
                * math.cos(self.current_yaw)
            )

            vy = (
                forward_speed
                * math.sin(self.current_yaw)
            )

            self.publish_approach_setpoint(
                vx,
                vy,
                self.target_z,
                self.current_yaw,
            )

            self.get_logger().info(
                f"[CHASE] "
                f"dx={dx:+.1f}px | "
                f"dy={dy:+.1f}px | "
                f"yaw_rate={yaw_rate:+.3f}rad/s | "
                f"speed={forward_speed:.1f}m/s"
            )

            self.mission_logger.log(
                log_time,
                self.state,
                self.detected,
                self.lock_on,
                self.near_2m,
                self.current_x,
                self.current_y,
                self.current_z,
                dx,
                dy,
                self.current_yaw,
                self.current_yaw,
                self.target_z,
                yaw_rate=yaw_rate,
                forward_speed=forward_speed,
                vx=vx,
                vy=vy,
                roll_actual=self.current_roll,
                pitch_actual=self.current_pitch,
                yaw_actual=self.current_yaw,
            )

        elif self.state == "INTERCEPT":

            self.publish_approach_setpoint(
                0.0,
                0.0,
                self.target_z,
                self.target_yaw,
            )
    # ================================================================
    # PX4 setpoint helpers
    # ================================================================

    def publish_offboard_control_mode(
        self,
        position,
        velocity,
    ):

        msg = OffboardControlMode()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(
        self,
        x,
        y,
        z,
        yaw=0.0,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        msg.velocity = [
            NAN,
            NAN,
            NAN,
        ]

        msg.yaw = float(yaw)
        msg.yawspeed = NAN

        self.trajectory_setpoint_pub.publish(msg)

    def publish_approach_setpoint(
        self,
        vx,
        vy,
        z,
        yaw,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        # X/Y = velocity control
        # Z   = position control
        msg.position = [
            NAN,
            NAN,
            float(z),
        ]

        msg.velocity = [
            float(vx),
            float(vy),
            NAN,
        ]

        msg.acceleration = [
            NAN,
            NAN,
            NAN,
        ]

        if self.state in ("ACQUIRE", "FOLLOW", "CHASE"):
            msg.yaw = NAN
            msg.yawspeed = float(
                self.last_yaw_rate_cmd
            )
        else:
            msg.yaw = float(yaw)
            msg.yawspeed = NAN

        self.trajectory_setpoint_pub.publish(msg)

    def publish_velocity_setpoint(
        self,
        vx,
        vy,
        vz,
        yawspeed,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.position = [
            NAN,
            NAN,
            NAN,
        ]

        msg.velocity = [
            float(vx),
            float(vy),
            float(vz),
        ]

        msg.acceleration = [
            NAN,
            NAN,
            NAN,
        ]

        msg.yaw = NAN
        msg.yawspeed = float(yawspeed)

        self.trajectory_setpoint_pub.publish(msg)

    

    # ================================================================
    # PX4 vehicle commands
    # ================================================================

    def publish_vehicle_command(
        self,
        command,
        **params,
    ):

        msg = VehicleCommand()

        msg.timestamp = int(
            self.get_clock().now().nanoseconds / 1000
        )

        msg.param1 = params.get(
            "param1",
            0.0,
        )

        msg.param2 = params.get(
            "param2",
            0.0,
        )

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

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )


def main(args=None):

    rclpy.init(args=args)

    node = MovingTargetFollowMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Rocket Track Mission stopped"
        )

    finally:

        node.mission_logger.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()