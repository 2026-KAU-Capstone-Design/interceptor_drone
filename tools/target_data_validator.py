#!/usr/bin/env python3
"""Read-only validator for synchronized YOLO detection and Depth distance."""

import math

import rclpy
from rclpy.node import Node
from interceptor_interfaces.msg import TargetDetection, TargetDistance


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class TargetDataValidator(Node):
    def __init__(self):
        super().__init__("target_data_validator")

        self.max_age_ms = 150.0
        self.max_sync_ms = 20.0
        self.match_timeout_ms = 100.0
        self.expected_class = 1

        self.detections = {}
        self.distances = {}

        self.accept_count = 0
        self.reject_count = 0
        self.timeout_count = 0
        self.last_log_ns = None

        self.create_subscription(
            TargetDetection, "/target/detection",
            self.on_detection, 10
        )
        self.create_subscription(
            TargetDistance, "/target/distance_stamped",
            self.on_distance, 10
        )
        self.create_timer(0.05, self.check_pending)

        self.get_logger().info(
            "Target data validator started (READ ONLY): "
            "class=1, max_age=150ms, sync=20ms, match_timeout=100ms"
        )

    def on_detection(self, msg):
        key = stamp_ns(msg.header)
        if key <= 0:
            self.reject_count += 1
            return
        self.detections[key] = msg
        self.try_match(key)

    def on_distance(self, msg):
        key = stamp_ns(msg.header)
        if key <= 0:
            self.reject_count += 1
            return
        self.distances[key] = msg
        self.try_match(key)

    def try_match(self, key):
        if key not in self.detections or key not in self.distances:
            return

        detection = self.detections.pop(key)
        distance = self.distances.pop(key)

        now_ns = self.get_clock().now().nanoseconds
        age_ms = (now_ns - key) / 1_000_000.0

        reason = self.validate(detection, distance, age_ms)

        if reason == "VALID":
            self.accept_count += 1
            self.report("MATCH_ACCEPT", reason, age_ms, distance.distance_m)
        else:
            self.reject_count += 1
            self.report("MATCH_REJECT", reason, age_ms, distance.distance_m)

    def validate(self, detection, distance, age_ms):
        if not detection.detected:
            return "NO_DETECTION"

        if detection.class_id != self.expected_class:
            return "WRONG_CLASS"

        if not distance.valid:
            return "INVALID_DISTANCE_FLAG"

        if detection.header.frame_id != distance.header.frame_id:
            return "FRAME_MISMATCH"

        if not math.isfinite(age_ms) or age_ms < 0:
            return "INVALID_AGE"

        if age_ms > self.max_age_ms:
            return "STALE_DATA"

        if not math.isfinite(distance.distance_m) or distance.distance_m <= 0:
            return "INVALID_DISTANCE"

        if (
            not math.isfinite(distance.sync_diff_ms)
            or abs(distance.sync_diff_ms) > self.max_sync_ms
        ):
            return "SYNC_ERROR"

        bbox = (detection.x1, detection.y1, detection.x2, detection.y2)
        if not all(math.isfinite(v) for v in bbox):
            return "INVALID_BBOX"

        if detection.x2 <= detection.x1 or detection.y2 <= detection.y1:
            return "INVALID_BBOX"

        return "VALID"

    def check_pending(self):
        now_ns = self.get_clock().now().nanoseconds

        for store in (self.detections, self.distances):
            expired = [
                key for key in store
                if (now_ns - key) / 1_000_000.0 > self.match_timeout_ms
            ]
            for key in expired:
                del store[key]
                self.timeout_count += 1

        if self.last_log_ns is None or now_ns - self.last_log_ns >= 1_000_000_000:
            self.get_logger().info(
                f"STATS accept={self.accept_count} "
                f"reject={self.reject_count} "
                f"unmatched_drop={self.timeout_count} "
                f"pending_det={len(self.detections)} "
                f"pending_dist={len(self.distances)}"
            )
            self.last_log_ns = now_ns

    def report(self, status, reason, age_ms, distance_m):
        now_ns = self.get_clock().now().nanoseconds

        if status == "MATCH_REJECT":
            self.get_logger().warn(
                f"{status} REASON={reason} AGE={age_ms:.1f}ms"
            )
        elif (
            self.last_log_ns is None
            or now_ns - self.last_log_ns >= 1_000_000_000
        ):
            self.get_logger().info(
                f"{status} REASON={reason} "
                f"AGE={age_ms:.1f}ms DISTANCE={distance_m:.3f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetDataValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
