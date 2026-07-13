#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageOnlyRecorder(Node):
    def __init__(self, args):
        super().__init__("image_only_recorder")

        self.topic = args.topic
        self.out_dir = Path(args.out).expanduser()
        self.max_images = args.max_images
        self.save_every = args.save_every

        self.frame_count = 0
        self.saved_count = 0

        (self.out_dir / "images").mkdir(parents=True, exist_ok=True)

        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("=== Image Only Recorder ===")
        self.get_logger().info(f"topic: {self.topic}")
        self.get_logger().info(f"output: {self.out_dir}")
        self.get_logger().info(f"max_images: {self.max_images}")
        self.get_logger().info(f"save_every: {self.save_every}")

    def ros_image_to_bgr(self, msg: Image):
        h = msg.height
        w = msg.width
        enc = msg.encoding.lower()

        if enc == "rgb8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
            image = image[:, :w * 3].reshape(h, w, 3)
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if enc == "bgr8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
            image = image[:, :w * 3].reshape(h, w, 3)
            return image.copy()

        if enc == "rgba8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
            image = image[:, :w * 4].reshape(h, w, 4)
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

        if enc == "bgra8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
            image = image[:, :w * 4].reshape(h, w, 4)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.frame_count % self.save_every != 0:
            return

        try:
            frame = self.ros_image_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"image conversion failed: {e}")
            return

        filename = f"fixedwing_{self.saved_count:06d}.jpg"
        image_path = self.out_dir / "images" / filename

        cv2.imwrite(str(image_path), frame)

        self.saved_count += 1
        self.get_logger().info(f"saved {self.saved_count}/{self.max_images}: {image_path}")

        if self.saved_count >= self.max_images:
            self.get_logger().info("Finished recording images.")
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image")
    parser.add_argument("--out", default="~/datasets/fixedwing_raw")
    parser.add_argument("--max-images", type=int, default=200)
    parser.add_argument("--save-every", type=int, default=3)

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ImageOnlyRecorder(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
