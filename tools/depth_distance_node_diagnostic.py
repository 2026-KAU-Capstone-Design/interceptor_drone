#!/usr/bin/env python3

from collections import deque

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from interceptor_interfaces.msg import TargetDetection


class DepthDistanceNode(Node):
    def __init__(self):
        super().__init__("depth_distance_node")

        # 앞서 ROS 2 CameraInfo에서 확인한 값
        self.fx_rgb = 1397.22359
        self.fy_rgb = 1397.22353
        self.cx_rgb = 960.0
        self.cy_rgb = 540.0

        self.fx_depth = 432.49604204
        self.fy_depth = 432.49604204
        self.cx_depth = 320.0
        self.cy_depth = 240.0

        # 최근 Depth 프레임 저장 (30Hz 기준 약 2초)
        self.depth_buffer = deque(maxlen=60)
        self.last_depth_stamp_ns = None

        # Depth 프레임과 아직 연결되지 않은 YOLO 검출 결과
        self.pending_detections = deque(maxlen=30)

        # 매칭 대기 시간: 초기 시험값 0.1초
        self.match_wait_ns = 100_000_000

        # 거리 결과를 이미 처리한 가장 최신 RGB 촬영 시각
        self.last_processed_rgb_stamp_ns = None

        # 동기화 실패 후 Depth 지연 도착 여부 진단
        self.expired_rgb_stamps = deque(maxlen=100)

        # 추후 RGB-Depth 촬영 시각 비교에 사용할 기준
        self.sync_tolerance_ns = 20_000_000  # 20ms

        # YOLO 메시지 수신 중단 감지
        self.last_detection_receive_ns = None
        self.detection_timeout_ns = 500_000_000  # 0.5초
        self.detection_timeout_reported = False

        # 0.1초마다 수신 중단 여부 확인
        self.create_timer(0.1, self.check_detection_timeout)

        self.create_subscription(
            Image,
            "/depth_camera",
            self.depth_callback,
            qos_profile_sensor_data,
        )

        # RGB 촬영 시각이 포함된 YOLO 검출 결과 구독
        self.create_subscription(
            TargetDetection,
            "/target/detection",
            self.stamped_detection_callback,
            10,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/target/depth_distance_m",
            10,
        )

        self.get_logger().info(
            "Depth distance node started (measurement only)"
        )


    def find_matching_depth(self, rgb_stamp_ns):
        """RGB 촬영 시각과 가장 가까운 Depth 프레임 검색."""
        if rgb_stamp_ns <= 0 or not self.depth_buffer:
            return None

        depth_stamp_ns, depth_msg = min(
            self.depth_buffer,
            key=lambda item: abs(item[0] - rgb_stamp_ns),
        )

        diff_ns = abs(depth_stamp_ns - rgb_stamp_ns)

        if diff_ns > self.sync_tolerance_ns:
            return None

        return depth_msg, diff_ns

    def process_pending_detections(self):
        """대기 중인 검출 결과와 Depth 프레임 매칭."""
        if not self.pending_detections:
            return

        now_ns = self.get_clock().now().nanoseconds
        remaining = deque()

        while self.pending_detections:
            receive_ns, detection_msg = self.pending_detections.popleft()

            rgb_stamp_ns = (
                detection_msg.header.stamp.sec * 1_000_000_000
                + detection_msg.header.stamp.nanosec
            )

            # 이미 처리한 프레임보다 오래된 결과는 폐기
            if (
                self.last_processed_rgb_stamp_ns is not None
                and rgb_stamp_ns <= self.last_processed_rgb_stamp_ns
            ):
                continue

            matched = self.find_matching_depth(rgb_stamp_ns)

            if matched is not None:
                depth_msg, diff_ns = matched
                bbox = (
                    detection_msg.x1,
                    detection_msg.y1,
                    detection_msg.x2,
                    detection_msg.y2,
                )

                distance = self.calculate_distance(depth_msg, bbox)
                self.last_processed_rgb_stamp_ns = rgb_stamp_ns
                self.publish_distance(distance)

                self.get_logger().info(
                    f"SYNC_DISTANCE={distance:.3f}m "
                    f"diff={diff_ns / 1e6:.2f}ms",
                    throttle_duration_sec=1.0,
                )

            elif now_ns - receive_ns >= self.match_wait_ns:
                # 매칭 실패한 RGB 촬영 시각을 진단용으로 보관
                self.expired_rgb_stamps.append(rgb_stamp_ns)
                self.last_processed_rgb_stamp_ns = rgb_stamp_ns
                self.publish_distance(-1.0)
                if self.depth_buffer:
                    nearest_stamp_ns, _ = min(
                        self.depth_buffer,
                        key=lambda item: abs(item[0] - rgb_stamp_ns),
                    )
                    diff_ms = (
                        nearest_stamp_ns - rgb_stamp_ns
                    ) / 1_000_000.0
                    oldest_ns = self.depth_buffer[0][0]
                    newest_ns = self.depth_buffer[-1][0]

                    self.get_logger().warn(
                        f"SYNC_TIMEOUT "
                        f"nearest_diff={diff_ms:+.2f}ms "
                        f"buffer_size={len(self.depth_buffer)} "
                        f"rgb_stamp={rgb_stamp_ns} "
                        f"depth_range=[{oldest_ns},{newest_ns}]"
                    )
                else:
                    self.get_logger().warn(
                        "SYNC_TIMEOUT depth_buffer=EMPTY"
                    )

            else:
                remaining.append((receive_ns, detection_msg))

        self.pending_detections = remaining

    def check_detection_timeout(self):
        # Depth 수신이 중단되어도 대기 중인 검출의 만료를 처리
        self.process_pending_detections()

        # 아직 YOLO 메시지를 받은 적이 없는 경우
        if self.last_detection_receive_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed_ns = now_ns - self.last_detection_receive_ns

        if elapsed_ns > self.detection_timeout_ns:
            if not self.detection_timeout_reported:
                self.publish_distance(-1.0)
                self.get_logger().warn(
                    "YOLO_TIMEOUT: distance invalidated"
                )
                self.detection_timeout_reported = True


    def stamped_detection_callback(self, msg):
        # YOLO 메시지 수신 시각 갱신
        self.last_detection_receive_ns = (
            self.get_clock().now().nanoseconds
        )
        self.detection_timeout_reported = False

        rgb_stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if rgb_stamp_ns <= 0:
            self.pending_detections.clear()
            self.publish_distance(-1.0)
            return

        # 이미 처리한 촬영 시각 이하의 메시지는 폐기
        if (
            self.last_processed_rgb_stamp_ns is not None
            and rgb_stamp_ns <= self.last_processed_rgb_stamp_ns
        ):
            return

        # 검출 실패 또는 다른 클래스: 이전 대기 결과 폐기
        if not msg.detected or msg.class_id != 1:
            self.pending_detections.clear()
            self.last_processed_rgb_stamp_ns = rgb_stamp_ns
            self.publish_distance(-1.0)
            return

        receive_ns = self.get_clock().now().nanoseconds

        # 검출 결과를 보관하고 현재 Depth 버퍼와 매칭 시도
        self.pending_detections.append((receive_ns, msg))
        self.process_pending_detections()

    def depth_callback(self, msg):
        # Depth 영상 촬영 시각
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        # 유효한 Depth 프레임만 저장
        if stamp_ns > 0 and msg.encoding == "32FC1":
            if self.last_depth_stamp_ns is not None:
                gap_ms = (
                    stamp_ns - self.last_depth_stamp_ns
                ) / 1_000_000.0

                if gap_ms > 50.0:
                    self.get_logger().warn(
                        f"DEPTH_GAP={gap_ms:.2f}ms"
                    )

            self.last_depth_stamp_ns = stamp_ns
            self.depth_buffer.append((stamp_ns, msg))

            # 과거에 만료된 RGB 프레임의 Depth가 뒤늦게 도착했는지 확인
            remaining_expired = deque(maxlen=100)

            for expired_stamp_ns in self.expired_rgb_stamps:
                diff_ns = abs(stamp_ns - expired_stamp_ns)

                if diff_ns <= self.sync_tolerance_ns:
                    self.get_logger().warn(
                        f"LATE_DEPTH_ARRIVAL "
                        f"rgb_stamp={expired_stamp_ns} "
                        f"depth_stamp={stamp_ns} "
                        f"diff={diff_ns / 1e6:.2f}ms"
                    )
                elif stamp_ns - expired_stamp_ns > 2_000_000_000:
                    # 2초 이상 지난 진단 기록은 폐기
                    continue
                else:
                    remaining_expired.append(expired_stamp_ns)

            self.expired_rgb_stamps = remaining_expired
            self.process_pending_detections()

    def publish_distance(self, distance):
        output = Float32()
        output.data = float(distance)
        self.distance_pub.publish(output)

    def calculate_distance(self, depth_msg, bbox):
        """동기화된 Depth 프레임과 RGB bbox로 거리 계산."""

        if depth_msg.encoding != "32FC1":
            return -1.0

        try:
            depth = np.frombuffer(
                depth_msg.data, dtype=np.float32
            ).reshape(
                depth_msg.height,
                depth_msg.step // 4
            )[:, :depth_msg.width]
        except (ValueError, TypeError):
            return -1.0

        x1, y1, x2, y2 = bbox

        # 잘못된 Bounding Box 좌표 차단
        if not np.all(np.isfinite([x1, y1, x2, y2])):
            return -1.0

        if x1 >= x2 or y1 >= y2:
            return -1.0

        def to_depth(u, v):
            ud = (
                self.fx_depth * (u - self.cx_rgb)
                / self.fx_rgb + self.cx_depth
            )
            vd = (
                self.fy_depth * (v - self.cy_rgb)
                / self.fy_rgb + self.cy_depth
            )
            return ud, vd

        ux1, vy1 = to_depth(x1, y1)
        ux2, vy2 = to_depth(x2, y2)

        left = max(0, int(np.floor(min(ux1, ux2))))
        right = min(depth_msg.width, int(np.ceil(max(ux1, ux2))))
        top = max(0, int(np.floor(min(vy1, vy2))))
        bottom = min(depth_msg.height, int(np.ceil(max(vy1, vy2))))

        if right <= left or bottom <= top:
            return -1.0

        roi = depth[top:bottom, left:right]

        valid = roi[
            np.isfinite(roi)
            & (roi > 0.2)
            & (roi < 19.1)
        ]

        if valid.size < 20:
            return -1.0

        return float(np.median(valid))


def main():
    rclpy.init()
    node = DepthDistanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
