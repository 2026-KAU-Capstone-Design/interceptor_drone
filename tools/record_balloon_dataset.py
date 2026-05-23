#!/usr/bin/env python3

import argparse
import os
import random
from pathlib import Path

import cv2
import numpy as np
import rclpy
# from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class BalloonDatasetRecorder(Node):
    def __init__(self, args):
        super().__init__("balloon_dataset_recorder")

        self.topic = args.topic
        self.out_dir = Path(args.out).expanduser()
        self.max_images = args.max_images
        self.save_every = args.save_every
        self.val_ratio = args.val_ratio
        self.min_area = args.min_area

        # self.bridge = CvBridge()
        self.frame_count = 0
        self.saved_count = 0

        for split in ["train", "val"]:
            (self.out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (self.out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("=== Balloon Dataset Recorder ===")
        self.get_logger().info(f"topic: {self.topic}")
        self.get_logger().info(f"output: {self.out_dir}")
        self.get_logger().info(f"max_images: {self.max_images}")
        self.get_logger().info(f"save_every: {self.save_every}")

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.frame_count % self.save_every != 0:
            return

        try:
            frame = self.ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image conversion failed: {e}")
            return

        bbox = self.find_red_balloon_bbox(frame)

        if bbox is None:
            if self.frame_count % 30 == 0:
                self.get_logger().warn("No red balloon detected in this frame")
            return

        self.get_logger().info(f"debug: bbox returned = {bbox}")

        x1, y1, x2, y2 = bbox
        h, w = frame.shape[:2]

        # YOLO format: class x_center y_center width height, normalized 0~1
        x_center = ((x1 + x2) / 2.0) / w
        y_center = ((y1 + y2) / 2.0) / h
        box_w = (x2 - x1) / w
        box_h = (y2 - y1) / h

        split = "val" if random.random() < self.val_ratio else "train"

        filename = f"balloon_{self.saved_count:06d}"
        image_path = self.out_dir / "images" / split / f"{filename}.jpg"
        label_path = self.out_dir / "labels" / split / f"{filename}.txt"

        cv2.imwrite(str(image_path), frame)

        with open(label_path, "w", encoding="utf-8") as f:
            f.write(f"0 {x_center:.6f} {y_center:.6f} {box_w:.6f} {box_h:.6f}\n")

        self.saved_count += 1

        self.get_logger().info(
            f"saved {self.saved_count}/{self.max_images}: {image_path} "
            f"bbox=({x1},{y1},{x2},{y2}) split={split}"
        )

        if self.saved_count >= self.max_images:
            self.get_logger().info("Finished recording dataset.")
            rclpy.shutdown()

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

        if encoding == "mono8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
            image = image[:, :width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def find_red_balloon_bbox(self, frame):
        """
        Detect dark-red balloon using a combined HSV + BGR rule.
        This is more tolerant for Gazebo lighting than HSV-only thresholding.
        """

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV red range, widened for dark red / maroon Gazebo material.
        lower_red1 = np.array([0, 25, 10])
        upper_red1 = np.array([20, 255, 255])

        lower_red2 = np.array([160, 25, 10])
        upper_red2 = np.array([179, 255, 255])

        mask_hsv1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_hsv2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_hsv = cv2.bitwise_or(mask_hsv1, mask_hsv2)

        # BGR dominance rule.
        # frame[:, :, 2] = R, frame[:, :, 1] = G, frame[:, :, 0] = B
        b = frame[:, :, 0].astype(np.int16)
        g = frame[:, :, 1].astype(np.int16)
        r = frame[:, :, 2].astype(np.int16)

        mask_bgr = (
            (r > 35) &
            (r > g + 15) &
            (r > b + 15)
        ).astype(np.uint8) * 255

        # Combine both masks.
        mask = cv2.bitwise_or(mask_hsv, mask_bgr)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if self.frame_count % 30 == 0:
            self.get_logger().info(f"debug: largest red area = {area:.1f}")

        if area < self.min_area:
            return None

        x, y, w, h = cv2.boundingRect(largest)

        # Add small padding around detected object.
        pad = 5
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(frame.shape[1] - 1, x + w + pad)
        y2 = min(frame.shape[0] - 1, y + h + pad)

        return x1, y1, x2, y2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image")
    parser.add_argument("--out", default="~/datasets/balloon_yolo")
    parser.add_argument("--max-images", type=int, default=500)
    parser.add_argument("--save-every", type=int, default=3)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--min-area", type=int, default=80)

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BalloonDatasetRecorder(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

