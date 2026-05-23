#!/usr/bin/env python3

import argparse
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray
from ultralytics import YOLO


class YoloLiveDetector(Node):
    def __init__(self, args):
        super().__init__("yolo_live_detector")

        self.topic = args.topic
        self.model_path = args.model
        self.conf = args.conf
        self.show = args.show
        self.frame_skip = args.frame_skip

        # Filtering parameters
        self.min_red_ratio = args.min_red_ratio
        self.min_box_area = args.min_box_area
        self.max_box_area_ratio = args.max_box_area_ratio
        self.min_aspect_ratio = args.min_aspect_ratio
        self.max_aspect_ratio = args.max_aspect_ratio
        self.max_jump_px = args.max_jump_px

        # Smoothing
        self.smooth_alpha = args.smooth_alpha
        self.smoothed_cx = None
        self.smoothed_cy = None
        self.smoothed_w = None
        self.smoothed_h = None

        # Lock-on parameters
        self.lock_window = args.lock_window
        self.lock_required_on = args.lock_required_on
        self.lock_required_off = args.lock_required_off
        self.center_tolerance_on = args.center_tolerance_on
        self.center_tolerance_off = args.center_tolerance_off
        self.lock_on_state = False

        # Distance estimation parameters
        self.balloon_diameter_m = args.balloon_diameter_m
        self.camera_hfov_rad = args.camera_hfov_rad
        self.near_distance_m = args.near_distance_m

        self.detect_history = deque(maxlen=self.lock_window)

        self.model = YOLO(self.model_path)

        self.frame_count = 0
        self.last_time = time.time()
        self.fps_count = 0

        self.miss_count = 0
        self.reset_after_misses = args.reset_after_misses

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.bbox_pub = self.create_publisher(Float32MultiArray, "/target/balloon_bbox", 10)
        self.lock_pub = self.create_publisher(Bool, "/target/lock_on", 10)
        self.distance_pub = self.create_publisher(Float32, "/target/distance_m", 10)
        self.near_pub = self.create_publisher(Bool, "/target/near_2m", 10)

        self.get_logger().info("=== YOLO Live Detector with Stabilization ===")
        self.get_logger().info(f"topic: {self.topic}")
        self.get_logger().info(f"model: {self.model_path}")
        self.get_logger().info(f"conf: {self.conf}")
        self.get_logger().info(f"min_red_ratio: {self.min_red_ratio}")
        self.get_logger().info(f"max_jump_px: {self.max_jump_px}")
        self.get_logger().info(f"smooth_alpha: {self.smooth_alpha}")
        self.get_logger().info("publishing: /target/balloon_bbox")
        self.get_logger().info("publishing: /target/lock_on")
        self.get_logger().info("publishing: /target/distance_m")
        self.get_logger().info("publishing: /target/near_2m")

    def ros_image_to_bgr(self, msg: Image):
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()

        if encoding == "rgb8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
            image = image[:, :width * 3].reshape(height, width, 3)
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if encoding == "bgr8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
            image = image[:, :width * 3].reshape(height, width, 3)
            return image.copy()

        if encoding == "rgba8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
            image = image[:, :width * 4].reshape(height, width, 4)
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

        if encoding == "bgra8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
            image = image[:, :width * 4].reshape(height, width, 4)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def compute_red_ratio(self, frame, x1, y1, x2, y2):
        h, w = frame.shape[:2]

        x1 = max(0, min(w - 1, int(x1)))
        x2 = max(0, min(w - 1, int(x2)))
        y1 = max(0, min(h - 1, int(y1)))
        y2 = max(0, min(h - 1, int(y2)))

        if x2 <= x1 or y2 <= y1:
            return 0.0

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return 0.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 25, 10])
        upper_red1 = np.array([20, 255, 255])
        lower_red2 = np.array([160, 25, 10])
        upper_red2 = np.array([179, 255, 255])

        mask_hsv1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_hsv2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_hsv = cv2.bitwise_or(mask_hsv1, mask_hsv2)

        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)

        mask_bgr = ((r > 35) & (r > g + 15) & (r > b + 15)).astype(np.uint8) * 255
        mask = cv2.bitwise_or(mask_hsv, mask_bgr)

        red_pixels = np.count_nonzero(mask)
        total_pixels = roi.shape[0] * roi.shape[1]

        return red_pixels / max(total_pixels, 1)

    def pass_geometry_filter(self, x1, y1, x2, y2, image_w, image_h):
        box_w = max(1, x2 - x1)
        box_h = max(1, y2 - y1)
        area = box_w * box_h
        image_area = image_w * image_h

        if area < self.min_box_area:
            return False, f"area too small: {area}"

        if area / image_area > self.max_box_area_ratio:
            return False, f"area too large: {area / image_area:.3f}"

        aspect = box_w / box_h
        if aspect < self.min_aspect_ratio or aspect > self.max_aspect_ratio:
            return False, f"bad aspect: {aspect:.2f}"

        return True, "ok"

    def pass_jump_filter(self, cx, cy):
        if self.smoothed_cx is None or self.smoothed_cy is None:
            return True, "first detection"

        dist = math.hypot(cx - self.smoothed_cx, cy - self.smoothed_cy)

        if dist > self.max_jump_px:
            return False, f"jump too large: {dist:.1f}px"

        return True, "ok"

    def update_smoothing(self, cx, cy, box_w, box_h):
        alpha = self.smooth_alpha

        if self.smoothed_cx is None:
            self.smoothed_cx = cx
            self.smoothed_cy = cy
            self.smoothed_w = box_w
            self.smoothed_h = box_h
        else:
            self.smoothed_cx = alpha * self.smoothed_cx + (1.0 - alpha) * cx
            self.smoothed_cy = alpha * self.smoothed_cy + (1.0 - alpha) * cy
            self.smoothed_w = alpha * self.smoothed_w + (1.0 - alpha) * box_w
            self.smoothed_h = alpha * self.smoothed_h + (1.0 - alpha) * box_h

        sx1 = self.smoothed_cx - self.smoothed_w / 2.0
        sy1 = self.smoothed_cy - self.smoothed_h / 2.0
        sx2 = self.smoothed_cx + self.smoothed_w / 2.0
        sy2 = self.smoothed_cy + self.smoothed_h / 2.0

        return sx1, sy1, sx2, sy2, self.smoothed_cx, self.smoothed_cy

    def reset_tracking_state(self):
        self.smoothed_cx = None
        self.smoothed_cy = None
        self.smoothed_w = None
        self.smoothed_h = None

    def update_lock_on(self, detected, dx, dy):
        self.detect_history.append(bool(detected))
        detect_count = sum(self.detect_history)

        abs_dx = abs(dx)
        abs_dy = abs(dy)

        if not self.lock_on_state:
            centered = abs_dx < self.center_tolerance_on and abs_dy < self.center_tolerance_on

            if (
                len(self.detect_history) >= self.lock_window
                and detect_count >= self.lock_required_on
                and centered
            ):
                self.lock_on_state = True

        else:
            too_far = abs_dx > self.center_tolerance_off or abs_dy > self.center_tolerance_off

            if detect_count <= self.lock_required_off or too_far:
                self.lock_on_state = False

        return self.lock_on_state

    def estimate_distance_m(self, bbox_width_px, image_width_px):
        if bbox_width_px <= 1:
            return -1.0

        fx = image_width_px / (2.0 * math.tan(self.camera_hfov_rad / 2.0))
        distance_m = self.balloon_diameter_m * fx / bbox_width_px

        return float(distance_m)

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.frame_count % self.frame_skip != 0:
            return

        try:
            frame = self.ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image conversion failed: {e}")
            return

        image_h, image_w = frame.shape[:2]
        image_cx = image_w / 2.0
        image_cy = image_h / 2.0

        results = self.model.predict(
            source=frame,
            conf=self.conf,
            imgsz=640,
            verbose=False,
            device=0,
        )

        result = results[0]
        boxes = result.boxes

        candidates = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                box_w = x2 - x1
                box_h = y2 - y1
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                ok_geom, reason_geom = self.pass_geometry_filter(x1, y1, x2, y2, image_w, image_h)
                if not ok_geom:
                    self.get_logger().debug(f"reject geometry: {reason_geom}")
                    continue

                red_ratio = self.compute_red_ratio(frame, x1, y1, x2, y2)
                if red_ratio < self.min_red_ratio:
                    self.get_logger().info(
                        f"reject red_ratio={red_ratio:.3f} conf={conf:.3f} bbox=({x1},{y1},{x2},{y2})"
                    )
                    continue

                ok_jump, reason_jump = self.pass_jump_filter(cx, cy)
                if not ok_jump:
                    self.get_logger().info(
                        f"reject jump: {reason_jump} conf={conf:.3f} bbox=({x1},{y1},{x2},{y2})"
                    )
                    continue

                candidates.append({
                    "cls_id": cls_id,
                    "conf": conf,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "cx": cx,
                    "cy": cy,
                    "box_w": box_w,
                    "box_h": box_h,
                    "red_ratio": red_ratio,
                })

        if not candidates:
            self.miss_count += 1

            if self.miss_count >= self.reset_after_misses:
                self.reset_tracking_state()

            self.publish_no_detection(image_w, image_h)
            lock_on = self.update_lock_on(False, 9999.0, 9999.0)
            self.publish_lock(lock_on)
            self.publish_distance(-1.0)
            self.publish_near(False)

            if self.show:
                cv2.imshow("YOLOv8 Balloon Detection", frame)
                cv2.waitKey(1)

            return

        # 가장 신뢰도 높은 후보 1개 선택
        best = max(candidates, key=lambda c: c["conf"])
        self.miss_count = 0

        sx1, sy1, sx2, sy2, scx, scy = self.update_smoothing(
            best["cx"],
            best["cy"],
            best["box_w"],
            best["box_h"],
        )

        dx = scx - image_cx
        dy = scy - image_cy

        lock_on = self.update_lock_on(True, dx, dy)

        smoothed_box_w = max(1.0, sx2 - sx1)
        distance_m = self.estimate_distance_m(smoothed_box_w, image_w)
        near_2m = 0.0 < distance_m <= self.near_distance_m

        self.publish_bbox(
            detected=1.0,
            conf=best["conf"],
            x1=sx1,
            y1=sy1,
            x2=sx2,
            y2=sy2,
            cx=scx,
            cy=scy,
            dx=dx,
            dy=dy,
            image_w=image_w,
            image_h=image_h,
            red_ratio=best["red_ratio"],
            distance_m=distance_m,
            near_2m=1.0 if near_2m else 0.0,
        )
        self.publish_lock(lock_on)
        self.publish_distance(distance_m)
        self.publish_near(near_2m)

        self.get_logger().info(
            f"bbox conf={best['conf']:.3f} red={best['red_ratio']:.3f} "
            f"center=({scx:.1f},{scy:.1f}) error=({dx:.1f},{dy:.1f}) lock_on={lock_on}"
            f"distance={distance_m:.2f}m near_2m={near_2m} lock_on={lock_on}"
        )

        annotated = frame.copy()

        # YOLO raw bbox
        cv2.rectangle(
            annotated,
            (int(best["x1"]), int(best["y1"])),
            (int(best["x2"]), int(best["y2"])),
            (255, 0, 0),
            2,
        )

        # Smoothed bbox
        cv2.rectangle(
            annotated,
            (int(sx1), int(sy1)),
            (int(sx2), int(sy2)),
            (0, 255, 0),
            2,
        )

        label = f"balloon {best['conf']:.2f} {distance_m:.2f}m red {best['red_ratio']:.2f}"
        cv2.putText(
            annotated,
            label,
            (int(sx1), max(20, int(sy1) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        # 화면 중심점
        cv2.circle(annotated, (int(image_cx), int(image_cy)), 5, (255, 255, 255), -1)

        # 표적 중심점
        cv2.circle(annotated, (int(scx), int(scy)), 5, (0, 255, 255), -1)

        # 중심 오차선
        cv2.line(
            annotated,
            (int(image_cx), int(image_cy)),
            (int(scx), int(scy)),
            (0, 255, 255),
            2,
        )

        if lock_on:
            cv2.putText(
                annotated,
                "LOCK ON",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
            )

        if near_2m:
            cv2.putText(
                annotated,
                "NEAR 2m",
                (30, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 0, 255),
                3,
            )

        self.fps_count += 1
        now = time.time()
        if now - self.last_time >= 1.0:
            self.get_logger().info(f"inference fps={self.fps_count}")
            self.fps_count = 0
            self.last_time = now

        if self.show:
            cv2.imshow("YOLOv8 Balloon Detection", annotated)
            cv2.waitKey(1)

    def publish_bbox(
        self,
        detected,
        conf,
        x1,
        y1,
        x2,
        y2,
        cx,
        cy,
        dx,
        dy,
        image_w,
        image_h,
        red_ratio,
        distance_m,
        near_2m,
    ):
        msg = Float32MultiArray()
        msg.data = [
            float(detected),
            float(conf),
            float(x1),
            float(y1),
            float(x2),
            float(y2),
            float(cx),
            float(cy),
            float(dx),
            float(dy),
            float(image_w),
            float(image_h),
            float(red_ratio),
            float(distance_m),
            float(near_2m),
        ]
        self.bbox_pub.publish(msg)

    def publish_no_detection(self, image_w, image_h):
        msg = Float32MultiArray()
        msg.data = [
            0.0,  # detected
            0.0,  # confidence
            0.0,  # x1
            0.0,  # y1
            0.0,  # x2
            0.0,  # y2
            0.0,  # cx
            0.0,  # cy
            0.0,  # dx
            0.0,  # dy
            float(image_w),
            float(image_h),
            0.0,  # red_ratio
            -1.0,  # distance_m, unavailable
            0.0,   # near_2m
        ]
        self.bbox_pub.publish(msg)

    def publish_distance(self, distance_m: float):
        msg = Float32()
        msg.data = float(distance_m)
        self.distance_pub.publish(msg)

    def publish_near(self, near: bool):
        msg = Bool()
        msg.data = bool(near)
        self.near_pub.publish(msg)

    def publish_lock(self, lock_on: bool):
        msg = Bool()
        msg.data = bool(lock_on)
        self.lock_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image")
    parser.add_argument("--model", required=True)
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--frame-skip", type=int, default=1)

    # Filtering options
    parser.add_argument("--min-red-ratio", type=float, default=0.08)
    parser.add_argument("--min-box-area", type=float, default=100.0)
    parser.add_argument("--max-box-area-ratio", type=float, default=0.85)
    parser.add_argument("--min-aspect-ratio", type=float, default=0.2)
    parser.add_argument("--max-aspect-ratio", type=float, default=5.0)
    parser.add_argument("--max-jump-px", type=float, default=600.0)
    parser.add_argument("--reset-after-misses", type=int, default=5)

    # Smoothing option
    parser.add_argument("--smooth-alpha", type=float, default=0.7)

    # Lock-on options
    parser.add_argument("--lock-window", type=int, default=10)
    parser.add_argument("--lock-required-on", type=int, default=7)
    parser.add_argument("--lock-required-off", type=int, default=4)
    parser.add_argument("--center-tolerance-on", type=float, default=100.0)
    parser.add_argument("--center-tolerance-off", type=float, default=180.0)

    # Distance estimation options
    parser.add_argument("--balloon-diameter-m", type=float, default=1.0)
    parser.add_argument("--camera-hfov-rad", type=float, default=1.74)
    parser.add_argument("--near-distance-m", type=float, default=2.0)

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YoloLiveDetector(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
