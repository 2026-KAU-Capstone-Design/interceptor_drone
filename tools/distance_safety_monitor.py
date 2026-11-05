#!/usr/bin/env python3
"""Read-only safety monitor for RGB-Depth distance measurements."""

import math

import rclpy
from rclpy.node import Node

from interceptor_interfaces.msg import TargetDistance


class DistanceSafetyMonitor(Node):
    def __init__(self):
        super().__init__("distance_safety_monitor")

        self.declare_parameter("max_age_ms", 150.0)
        self.declare_parameter("timeout_ms", 300.0)
        self.declare_parameter("max_sync_diff_ms", 20.0)

        self.max_age_ms = float(self.get_parameter("max_age_ms").value)
        self.timeout_ms = float(self.get_parameter("timeout_ms").value)
        self.max_sync_diff_ms = float(
            self.get_parameter("max_sync_diff_ms").value
        )

        self.last_rx_ns = None
        self.last_status = None
        self.last_report_ns = None

        self.create_subscription(
            TargetDistance,
            "/target/distance_stamped",
            self.on_distance,
            10,
        )
        self.create_timer(0.05, self.check_timeout)

        self.get_logger().info(
            "Distance safety monitor started (READ ONLY): "
            f"max_age={self.max_age_ms:.0f}ms, "
            f"timeout={self.timeout_ms:.0f}ms, "
            f"max_sync_diff={self.max_sync_diff_ms:.0f}ms"
        )

    def report(self, status, reason, age_ms=None, distance_m=None):
        now_ns = self.get_clock().now().nanoseconds

        changed = status != self.last_status
        periodic = (
            self.last_report_ns is None
            or now_ns - self.last_report_ns >= 1_000_000_000
        )

        if changed or periodic:
            age_text = (
                f"{age_ms:.1f}ms" if age_ms is not None else "N/A"
            )
            distance_text = (
                f"{distance_m:.3f}m"
                if distance_m is not None
                else "N/A"
            )

            self.get_logger().info(
                f"STATUS={status} REASON={reason} "
                f"AGE={age_text} DISTANCE={distance_text}"
            )
            self.last_report_ns = now_ns

        self.last_status = status

    def on_distance(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.last_rx_ns = now_ns

        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        if not msg.valid:
            self.report("REJECT", "INVALID_FLAG")
            return

        if stamp_ns <= 0:
            self.report("REJECT", "INVALID_TIMESTAMP")
            return

        distance = float(msg.distance_m)
        if not math.isfinite(distance) or distance <= 0.0:
            self.report("REJECT", "INVALID_DISTANCE")
            return

        sync_diff = float(msg.sync_diff_ms)
        if (
            not math.isfinite(sync_diff)
            or abs(sync_diff) > self.max_sync_diff_ms
        ):
            self.report("REJECT", "SYNC_ERROR")
            return

        age_ms = (now_ns - stamp_ns) / 1_000_000.0

        if age_ms < 0.0:
            self.report("REJECT", "FUTURE_TIMESTAMP", age_ms, distance)
        elif age_ms > self.max_age_ms:
            self.report("REJECT", "STALE_DATA", age_ms, distance)
        else:
            self.report("ACCEPT", "VALID", age_ms, distance)

    def check_timeout(self):
        now_ns = self.get_clock().now().nanoseconds

        if self.last_rx_ns is None:
            self.report("WAIT", "NO_MESSAGE")
            return

        elapsed_ms = (now_ns - self.last_rx_ns) / 1_000_000.0

        if elapsed_ms > self.timeout_ms:
            self.report("REJECT", "MESSAGE_TIMEOUT", elapsed_ms)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSafetyMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
