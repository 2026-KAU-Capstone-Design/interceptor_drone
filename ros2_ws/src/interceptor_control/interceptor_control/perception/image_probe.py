#!/usr/bin/env python3
"""
Image Probe — 카메라 토픽이 실제로 들어오는지 검증하는 진단 노드

/camera/image 를 구독해서:
  - 메시지 수신율(Hz)
  - 해상도, encoding
  - 픽셀 통계 (평균/최대/최소)
  - "검붉은색" 비율 (HSV 기반 간단 마스크) — 풍선이 보이는지 빠르게 확인
을 1초마다 출력한다.

사용법:
  ros2 run interceptor_control image_probe

  # 또는 다른 토픽으로 실행:
  ros2 run interceptor_control image_probe --ros-args -r __ns:=/cam2 -p topic:=/cam2/image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class ImageProbe(Node):
    def __init__(self):
        super().__init__('image_probe')
        self.declare_parameter('topic', '/camera/image')
        topic = self.get_parameter('topic').value

        self.sub = self.create_subscription(Image, topic, self.cb_image, 10)
        self.timer = self.create_timer(1.0, self.print_stats)

        self.frame_count = 0
        self.total_frames = 0
        self.last_msg = None
        self.last_red_ratio = 0.0

        self.get_logger().info(f'=== Image Probe ===')
        self.get_logger().info(f'  topic: {topic}')
        self.get_logger().info(f'  매 1초마다 통계 출력')

    def cb_image(self, msg: Image):
        self.frame_count += 1
        self.total_frames += 1
        self.last_msg = msg

        # 빠른 검붉은색 비율 추정 (RGB 기준 간단 룰)
        try:
            ch = 3 if 'rgb' in msg.encoding.lower() or 'bgr' in msg.encoding.lower() else 1
            if ch == 3:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                # 가운데 320x240 영역만 (속도)
                h, w, _ = arr.shape
                cx, cy = w // 2, h // 2
                roi = arr[max(0, cy-120):cy+120, max(0, cx-160):cx+160]
                # encoding이 rgb8 또는 bgr8 일 수 있음
                if 'bgr' in msg.encoding.lower():
                    b, g, r = roi[..., 0], roi[..., 1], roi[..., 2]
                else:
                    r, g, b = roi[..., 0], roi[..., 1], roi[..., 2]
                # 검붉은색 마스크: R 우세 + G/B 낮음 + 너무 밝지 않음
                mask = (r.astype(int) > g.astype(int) + 30) & \
                       (r.astype(int) > b.astype(int) + 30) & \
                       (r < 200)
                self.last_red_ratio = float(mask.mean())
        except Exception:
            self.last_red_ratio = -1.0

    def print_stats(self):
        if self.last_msg is None:
            self.get_logger().warn(
                '[NO IMAGE] 이미지 토픽 수신 안됨 — bridge 와 PX4 SITL 확인 필요')
            return

        msg = self.last_msg
        size_kb = len(msg.data) / 1024
        self.get_logger().info(
            f'fps={self.frame_count:>3d}  '
            f'{msg.width}x{msg.height} {msg.encoding}  '
            f'{size_kb:>6.1f}KB/frame  '
            f'red_ratio={self.last_red_ratio:.3f}  '
            f'(total {self.total_frames})'
        )
        self.frame_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImageProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
