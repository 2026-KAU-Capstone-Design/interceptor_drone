#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image

CAMERA = (
    "/dev/v4l/by-id/"
    "usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0"
)

class UsbCamera(Node):
    def __init__(self):
        super().__init__("usb_camera_ros")
        self.publisher = self.create_publisher(Image, "/camera/image", 1)
        self.cap = cv2.VideoCapture(CAMERA, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError("USB 카메라 열기 실패: " + CAMERA)

        self.cap.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.failures = 0
        self.first_frame = True
        self.timer = self.create_timer(1.0 / 30.0, self.capture)

    def capture(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.failures += 1
            if self.failures >= 5:
                raise RuntimeError("카메라 프레임 수신 실패")
            return

        self.failures = 0
        # ROS_CAMERA_ROTATE_180
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        height, width = frame.shape[:2]

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = frame.tobytes()
        self.publisher.publish(msg)

        if self.first_frame:
            self.get_logger().info(
                f"CAMERA_READY {width}x{height} -> /camera/image")
            self.first_frame = False

def main():
    rclpy.init()
    node = None
    try:
        node = UsbCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cap.release()
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
