#!/usr/bin/env python3
"""
IBVS Monitor — 실시간 제어 시각화
====================================
드론 카메라 피드에 bbox, 조준점, 오차 벡터, 제어 명령을 오버레이해서 보여줍니다.

실행:
  python3 tools/ibvs_monitor.py
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool, Float32
import cv2
import numpy as np


def ros_img_to_cv2(msg: Image) -> np.ndarray:
    """cv_bridge 없이 ROS Image → OpenCV BGR 변환 (NumPy 2.x 호환)."""
    dtype_map = {
        'rgb8':   (np.uint8,  3),
        'bgr8':   (np.uint8,  3),
        'rgba8':  (np.uint8,  4),
        'bgra8':  (np.uint8,  4),
        'mono8':  (np.uint8,  1),
        'mono16': (np.uint16, 1),
    }
    enc = msg.encoding.lower()
    if enc not in dtype_map:
        raise ValueError(f'지원하지 않는 인코딩: {enc}')
    dtype, ch = dtype_map[enc]
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    if ch == 1:
        img = arr.reshape(msg.height, msg.width)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        img = arr.reshape(msg.height, msg.width, ch)
        if enc in ('rgb8', 'rgba8'):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


# YOLO 토픽 QoS
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class IBVSMonitor(Node):
    def __init__(self):
        super().__init__('ibvs_monitor')
        self.frame = None
        self.bbox = None          # [cx, cy, w, h, ...]
        self.lock_on = False
        self.distance_m = -1.0
        self.near_2m = False
        self.image_w = 1280
        self.image_h = 960

        # Kp 파라미터 (intercept.py 와 동일)
        self.Kp_yaw = 0.8
        self.Kp_vz  = 1.0

        self.create_subscription(Image, '/camera/image',
                                 self._cb_image, SENSOR_QOS)
        self.create_subscription(Float32MultiArray, '/target/balloon_bbox',
                                 self._cb_bbox, SENSOR_QOS)
        self.create_subscription(Bool, '/target/lock_on',
                                 self._cb_lock, SENSOR_QOS)
        self.create_subscription(Float32, '/target/distance_m',
                                 self._cb_dist, SENSOR_QOS)
        self.create_subscription(Bool, '/target/near_2m',
                                 self._cb_near, SENSOR_QOS)

        self.create_timer(0.05, self._render)  # 20Hz display
        self.get_logger().info('IBVS Monitor 시작 — 창에서 q 또는 Esc 로 종료')

    # ──────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────
    def _cb_image(self, msg):
        try:
            self.frame = ros_img_to_cv2(msg)
            self.image_h, self.image_w = self.frame.shape[:2]
        except Exception:
            pass

    def _cb_bbox(self, msg):
        if len(msg.data) >= 4:
            self.bbox = msg.data  # [cx, cy, w, h, ...]

    def _cb_lock(self, msg):
        self.lock_on = msg.data

    def _cb_dist(self, msg):
        self.distance_m = msg.data

    def _cb_near(self, msg):
        self.near_2m = msg.data

    # ──────────────────────────────────────────────
    # 렌더링
    # ──────────────────────────────────────────────
    def _render(self):
        if self.frame is None:
            return

        img = self.frame.copy()
        cx_img = self.image_w // 2
        cy_img = self.image_h // 2

        # ── 중앙 조준선 ──
        self._draw_crosshair(img, cx_img, cy_img, size=30, color=(0, 255, 0), thickness=2)

        # ── 풍선 bbox + 제어 벡터 ──
        yaw_rate = 0.0
        vz = 0.0
        if self.bbox is not None and len(self.bbox) >= 4:
            bx  = int(self.bbox[0])   # bbox 중심 x
            by  = int(self.bbox[1])   # bbox 중심 y
            bw  = int(self.bbox[2])   # bbox 너비
            bh  = int(self.bbox[3])   # bbox 높이

            x1 = bx - bw // 2
            y1 = by - bh // 2
            x2 = bx + bw // 2
            y2 = by + bh // 2

            # 탐지만으로 바로 TRACK 표시 (lock_on 불필요)
            box_color = (0, 200, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(img, (bx, by), 5, box_color, -1)

            # 오차 계산 (IBVS 제어와 동일)
            dx = bx - cx_img
            dy = by - cy_img
            ex = dx / (self.image_w / 2.0)   # [-1, 1]
            ey = dy / (self.image_h / 2.0)   # [-1, 1]
            yaw_rate = float(np.clip(self.Kp_yaw * ex, -1.0,  1.0))
            vz       = float(np.clip(self.Kp_vz  * ey, -2.0,  2.0))

            # 오차 벡터 항상 표시 (중심 → bbox 중심)
            cv2.arrowedLine(img, (cx_img, cy_img), (bx, by),
                            (0, 50, 255), 2, tipLength=0.15)

            # bbox 레이블
            cv2.putText(img, "TRACK", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        # ── 픽셀 오차 바 ──
        self._draw_error_bar(img, cx_img, cy_img)

        # ── HUD 텍스트 ──
        self._draw_hud(img, yaw_rate, vz)

        cv2.imshow('IBVS Monitor', img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            sys.exit(0)

    def _draw_crosshair(self, img, cx, cy, size=30, color=(0, 255, 0), thickness=2):
        cv2.line(img, (cx - size, cy), (cx + size, cy), color, thickness)
        cv2.line(img, (cx, cy - size), (cx, cy + size), color, thickness)
        cv2.circle(img, (cx, cy), size // 2, color, 1)

    def _draw_error_bar(self, img, cx_img, cy_img):
        if self.bbox is None or len(self.bbox) < 4:
            return
        bx = int(self.bbox[0])
        by = int(self.bbox[1])
        dx = bx - cx_img
        dy = by - cy_img

        # 수평 오차 바 (하단)
        bar_y = self.image_h - 30
        bar_len = 200
        bar_x = cx_img
        cv2.line(img, (bar_x - bar_len, bar_y), (bar_x + bar_len, bar_y), (80, 80, 80), 2)
        ex_pix = int(np.clip(dx / (self.image_w / 2.0) * bar_len, -bar_len, bar_len))
        bar_color_x = (0, 200, 0) if abs(ex_pix) < bar_len * 0.1 else (0, 100, 255)
        cv2.line(img, (bar_x, bar_y), (bar_x + ex_pix, bar_y), bar_color_x, 4)
        cv2.putText(img, f"yaw_err: {dx:+.0f}px", (bar_x - bar_len, bar_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # 수직 오차 바 (우측)
        bar_x2 = self.image_w - 30
        bar_cy = cy_img
        cv2.line(img, (bar_x2, bar_cy - bar_len), (bar_x2, bar_cy + bar_len), (80, 80, 80), 2)
        ey_pix = int(np.clip(dy / (self.image_h / 2.0) * bar_len, -bar_len, bar_len))
        bar_color_y = (0, 200, 0) if abs(ey_pix) < bar_len * 0.1 else (0, 100, 255)
        cv2.line(img, (bar_x2, bar_cy), (bar_x2, bar_cy + ey_pix), bar_color_y, 4)
        cv2.putText(img, f"vz_err: {dy:+.0f}px", (bar_x2 - 100, bar_cy - bar_len - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    def _draw_hud(self, img, yaw_rate, vz):
        lines = []

        # 상태
        if self.near_2m:
            state_str = "INTERCEPT"
            state_color = (0, 0, 255)
        elif self.bbox is not None:
            state_str = "TRACK"
            state_color = (0, 200, 255)
        else:
            state_str = "SEARCH"
            state_color = (180, 180, 180)

        cv2.putText(img, f"State: {state_str}", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)

        if self.distance_m > 0:
            cv2.putText(img, f"Dist : {self.distance_m:.1f} m", (15, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1)

        if self.bbox is not None:
            cv2.putText(img, f"yaw_rate: {yaw_rate:+.3f} rad/s", (15, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)
            cv2.putText(img, f"vz      : {vz:+.3f} m/s  (NED)", (15, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1)

        # 조준 정렬 표시
        if self.bbox is not None and len(self.bbox) >= 4:
            bx = int(self.bbox[0])
            by = int(self.bbox[1])
            cx_img = self.image_w // 2
            cy_img = self.image_h // 2
            err = math.sqrt((bx - cx_img)**2 + (by - cy_img)**2)
            aligned = err < 30
            mark = "■ ALIGNED" if aligned else f"  err={err:.0f}px"
            c = (0, 255, 100) if aligned else (0, 160, 255)
            cv2.putText(img, mark, (15, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)


def main():
    rclpy.init()
    node = IBVSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()
