#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Float32MultiArray, Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
)


NAN = float("nan")


class RocketTrackMission(Node):

    def __init__(self):
        super().__init__("rocket_track_mission")

        # ============================================================
        # Mission parameters
        # ============================================================

        self.flight_altitude = -5.0
        self.acceptance_radius = 0.5

        # YOLO / tracking parameters
        self.image_w = 1280.0
        self.image_h = 960.0

        # dx normalized error -> yaw rate
        self.kp_yaw = 0.8

        # Maximum yaw rate [rad/s]
        self.max_yaw_rate = 0.6

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
        self.offboard_setpoint_counter = 0
        self.hover_start_time = None

        # Current / target yaw [rad]
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_initialized = False
         # Vertical tracking
        self.target_z = self.flight_altitude
        self.kp_z = 0.25
        self.max_z_step = 0.05
        self.z_deadband_px = 30.0

        # PX4 NED altitude limits
        self.min_target_z = -10.0
        self.max_target_z = -2.5
        # Approach control
        self.approach_speed = 1.0
        self.approach_lock_px = 40.0
       
        # Control period = 20 Hz
        self.control_dt = 0.05
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

        self.publish_offboard_control_mode(
            position=True,
            velocity=True,
        )

        # ------------------------------------------------------------
        # Send setpoints before Offboard activation
        # ------------------------------------------------------------

        if self.offboard_setpoint_counter < 20:

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
                yaw=self.target_yaw,
            )

            self.offboard_setpoint_counter += 1

            return

        if self.offboard_setpoint_counter == 20:

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

                self.state = "WAIT_TARGET"

                self.get_logger().info(
                    "Hover complete -> WAIT_TARGET"
                )

        # ------------------------------------------------------------
        # WAIT TARGET
        # ------------------------------------------------------------

        elif self.state == "WAIT_TARGET":

            self.publish_position_setpoint(
                0.0,
                0.0,
                self.flight_altitude,
                yaw=self.target_yaw,
            )

            if self.detected:

                self.miss_count = 0
                self.state = "YAW_TRACK"

                self.get_logger().info(
                    f"Target detected -> YAW_TRACK "
                    f"dx={self.bbox_dx:.1f}px"
                )

        # ------------------------------------------------------------
        # YAW TRACKING
        # ------------------------------------------------------------

        elif self.state == "YAW_TRACK":

            # Detection lost
            if not self.detected:

                self.miss_count += 1

                self.publish_position_setpoint(
                    0.0,
                    0.0,
                    self.flight_altitude,
                    yaw=self.target_yaw,
                )


                if self.miss_count >= self.miss_limit:

                    self.miss_count = 0
                    self.state = "WAIT_TARGET"

                    self.get_logger().warn(
                        "Target lost -> WAIT_TARGET"
                    )

                return

            self.miss_count = 0

            if not self.new_bbox:
                self.publish_position_setpoint(
                    0.0,
                    0.0,
                    self.target_z,
                    yaw=self.target_yaw,
                )
                return

            self.new_bbox = False
            # --------------------------------------------------------
            # Horizontal image error
            # --------------------------------------------------------

            dx = self.bbox_dx
            dy = self.bbox_dy

            # Target close enough to image center
            if abs(dx) <= self.yaw_deadband_px:

                yaw_rate = 0.0

                phase = "LOCK"

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

                phase = "TRACK"
            # --------------------------------------------------------
            # Vertical image error -> altitude target correction
            # --------------------------------------------------------

            if abs(dy) <= self.z_deadband_px:
                z_step = 0.0
            else:
                normalized_dy = (
                    dy / (self.image_h / 2.0)
                )

                z_step = (
                    self.kp_z * normalized_dy
                )

                z_step = max(
                    -self.max_z_step,
                    min(
                        self.max_z_step,
                        z_step,
                    ),
                )

            # PX4 NED:
            # target_z 증가 = 하강
            # target_z 감소 = 상승
            self.target_z += z_step

            self.target_z = max(
                self.min_target_z,
                min(
                    self.max_target_z,
                    self.target_z,
                ),
            )


            # Integrate yaw rate into yaw angle command
            self.target_yaw += yaw_rate * self.control_dt

            # Wrap yaw to [-pi, pi]
            self.target_yaw = math.atan2(
                math.sin(self.target_yaw),
                math.cos(self.target_yaw),
            )

            # Hold XYZ position, change Yaw only
            self.publish_position_setpoint(
                0.0,
                0.0,
                self.target_z,
                yaw=self.target_yaw,
            )

            self.get_logger().info(
                f"[{phase}] "
                f"dx={dx:+.1f}px | "
                f"dy={dy:+.1f}px | "
                f"yaw_rate={yaw_rate:+.3f}rad/s | "
                f"yaw_sp={math.degrees(self.target_yaw):+.1f}deg | "
                f"z_sp={self.target_z:+.2f}m"
            )

            # --------------------------------------------------------
            # Yaw locked -> start approach
            # --------------------------------------------------------

            if abs(dx) <= self.approach_lock_px:

                self.state = "APPROACH"

                self.get_logger().info(
                    "Yaw aligned -> APPROACH"
                )
        # ------------------------------------------------------------
        # APPROACH
        # ------------------------------------------------------------
        elif self.state == "APPROACH":
            # --------------------------------------------------------
            # Intercept distance reached
            # --------------------------------------------------------
            if self.near_2m:
                self.state = "INTERCEPT"
                self.publish_approach_setpoint(
                    0.0,
                    0.0,
                    self.target_z,
                    self.target_yaw,
                )

                self.get_logger().info(
                    "[INTERCEPT] Target within 2 m -> STOP"
                )

                return
            # --------------------------------------------------------
            # Detection lost
            # --------------------------------------------------------
            if not self.detected:

                self.miss_count += 1

                self.publish_approach_setpoint(
                    0.0,
                    0.0,
                    self.target_z,
                    self.target_yaw,
                )

                if self.miss_count >= self.miss_limit:

                    self.miss_count = 0
                    self.state = "WAIT_TARGET"

                    self.get_logger().warn(
                        "Target lost -> WAIT_TARGET"
                    )

                return

            self.miss_count = 0

            # --------------------------------------------------------
            # No new YOLO frame -> keep previous command
            # --------------------------------------------------------
            if not self.new_bbox:

                vx = (
                    self.approach_speed
                    * math.cos(self.target_yaw)
                )

                vy = (
                    self.approach_speed
                    * math.sin(self.target_yaw)
                )

                self.publish_approach_setpoint(
                    vx,
                    vy,
                    self.target_z,
                    self.target_yaw,
                )

                return

            # New YOLO frame consumed
            self.new_bbox = False

            dx = self.bbox_dx
            dy = self.bbox_dy

            # --------------------------------------------------------
            # Yaw correction
            # --------------------------------------------------------
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

            self.target_yaw += (
                yaw_rate * self.control_dt
            )

            self.target_yaw = math.atan2(
                math.sin(self.target_yaw),
                math.cos(self.target_yaw),
            )

            # --------------------------------------------------------
            # Vertical correction
            # --------------------------------------------------------
            if abs(dy) <= self.z_deadband_px:
                z_step = 0.0
            else:
                normalized_dy = (
                    dy / (self.image_h / 2.0)
                )

                z_step = (
                    self.kp_z * normalized_dy
                )

                z_step = max(
                    -self.max_z_step,
                    min(
                        self.max_z_step,
                        z_step,
                    ),
                )

            self.target_z += z_step

            self.target_z = max(
                self.min_target_z,
                min(
                    self.max_target_z,
                    self.target_z,
                ),
            )

            # --------------------------------------------------------
            # Forward velocity
            # --------------------------------------------------------
            # Horizontal error based forward-speed control
            abs_dx = abs(dx)

            if abs_dx <= 60.0:
                # Target near center -> normal approach
                forward_speed = self.approach_speed

            elif abs_dx <= 150.0:
                # Moderate error -> slow approach
                forward_speed = 0.4

            else:
                # Large error -> stop forward motion and yaw-align first
                forward_speed = 0.0


            vx = (
                forward_speed
                * math.cos(self.target_yaw)
            )

            vy = (
                forward_speed
                * math.sin(self.target_yaw)
            )

            self.publish_approach_setpoint(
                vx,
                vy,
                self.target_z,
                self.target_yaw,
            )

            self.get_logger().info(
                f"[APPROACH] "
                f"dx={dx:+.1f}px | "
                f"dy={dy:+.1f}px | "
                f"yaw_rate={yaw_rate:+.3f}rad/s | "
                f"vx={vx:+.2f} | "
                f"vy={vy:+.2f} | "
                f"speed={forward_speed:.2f} | "
                f"yaw_sp={math.degrees(self.target_yaw):+.1f}deg | "
                f"z_sp={self.target_z:+.2f}m"
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

    node = RocketTrackMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Rocket Track Mission stopped"
        )

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()