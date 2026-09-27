#!/usr/bin/env python3

import argparse
import math
import time
import sys


from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# ============================================================
# Gazebo pose control
# ============================================================

def yaw_to_quaternion(yaw):
    """
    Z축 yaw(rad)를 quaternion으로 변환.
    roll = pitch = 0
    """
    half = yaw / 2.0

    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)

    return qx, qy, qz, qw


def set_pose(node, world, model, x, y, z, yaw):
    """
    Gazebo Transport Python API를 이용하여
    target model의 위치와 방향을 직접 변경한다.

    subprocess로 'gz service'를 매번 실행하지 않기 때문에
    훨씬 높은 update rate를 기대할 수 있다.
    """

    qx, qy, qz, qw = yaw_to_quaternion(yaw)

    pose = Pose()

    pose.name = model

    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    service = f"/world/{world}/set_pose"

    result, response = node.request(
        service,
        pose,
        Pose,
        Boolean,
        100
    )

    if not result:
        return False

    return response.data

# ============================================================
# Circle
# ============================================================

def circle_pose(distance, cx, cy, z, radius):
    """
    distance = 지금까지 이동해야 하는 경로 거리 [m]

    원에서는
        arc length = radius * theta
    따라서
        theta = distance / radius
    """

    theta = distance / radius

    x = cx + radius * math.cos(theta)
    y = cy + radius * math.sin(theta)

    # 원의 접선 방향
    dx = -radius * math.sin(theta)
    dy = radius * math.cos(theta)

    yaw = math.atan2(dy, dx)

    return x, y, z, yaw


# ============================================================
# Ellipse
# ============================================================

class EllipsePath:
    """
    타원에서 단순히 theta를 일정하게 증가시키면
    실제 이동 속도가 일정하지 않는다.

    따라서 타원을 작은 구간으로 미리 나누고,
    각 구간의 실제 길이를 계산하여
    누적 거리 기반으로 위치를 찾는다.
    """

    def __init__(self, cx, cy, z, a, b, samples=5000):

        self.cx = cx
        self.cy = cy
        self.z = z
        self.a = a
        self.b = b

        self.samples = samples

        self.theta_table = []
        self.distance_table = []

        total_distance = 0.0

        prev_x = cx + a
        prev_y = cy

        self.theta_table.append(0.0)
        self.distance_table.append(0.0)

        for i in range(1, samples + 1):

            theta = 2.0 * math.pi * i / samples

            x = cx + a * math.cos(theta)
            y = cy + b * math.sin(theta)

            segment = math.hypot(
                x - prev_x,
                y - prev_y
            )

            total_distance += segment

            self.theta_table.append(theta)
            self.distance_table.append(total_distance)

            prev_x = x
            prev_y = y

        self.length = total_distance

    def pose(self, distance):

        # 한 바퀴 이후 다시 처음부터
        s = distance % self.length

        # 누적 거리 table에서 현재 위치 검색
        low = 0
        high = len(self.distance_table) - 1

        while low < high:
            mid = (low + high) // 2

            if self.distance_table[mid] < s:
                low = mid + 1
            else:
                high = mid

        i = max(1, low)

        s0 = self.distance_table[i - 1]
        s1 = self.distance_table[i]

        theta0 = self.theta_table[i - 1]
        theta1 = self.theta_table[i]

        if s1 > s0:
            ratio = (s - s0) / (s1 - s0)
        else:
            ratio = 0.0

        theta = theta0 + ratio * (theta1 - theta0)

        x = self.cx + self.a * math.cos(theta)
        y = self.cy + self.b * math.sin(theta)

        # 타원 접선
        dx = -self.a * math.sin(theta)
        dy = self.b * math.cos(theta)

        yaw = math.atan2(dy, dx)

        return x, y, self.z, yaw


# ============================================================
# Rectangle
# ============================================================

def rectangle_pose(distance, cx, cy, z, width, height):

    perimeter = 2.0 * (width + height)

    s = distance % perimeter

    xmin = cx - width / 2.0
    xmax = cx + width / 2.0

    ymin = cy - height / 2.0
    ymax = cy + height / 2.0

    # 1. 아래쪽: 왼쪽 -> 오른쪽
    if s < width:

        x = xmin + s
        y = ymin

        yaw = 0.0

    # 2. 오른쪽: 아래 -> 위
    elif s < width + height:

        s2 = s - width

        x = xmax
        y = ymin + s2

        yaw = math.pi / 2.0

    # 3. 위쪽: 오른쪽 -> 왼쪽
    elif s < 2.0 * width + height:

        s2 = s - (width + height)

        x = xmax - s2
        y = ymax

        yaw = math.pi

    # 4. 왼쪽: 위 -> 아래
    else:

        s2 = s - (2.0 * width + height)

        x = xmin
        y = ymax - s2

        yaw = -math.pi / 2.0

    return x, y, z, yaw


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description="Gazebo moving standard VTOL target"
    )

    # Gazebo
    parser.add_argument(
        "--world",
        default="simple_moving_vtol_target",
        help="Gazebo world name"
    )

    parser.add_argument(
        "--model",
        default="target_standard_vtol",
        help="Target model name"
    )

    # Path
    parser.add_argument(
        "--path",
        choices=["circle", "ellipse", "rectangle"],
        default="circle",
        help="Target path"
    )

    # Motion
    parser.add_argument(
        "--speed",
        type=float,
        default=10.0,
        help="Target speed [m/s]"
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Pose update rate [Hz]"
    )

    # Path center
    parser.add_argument(
        "--center-x",
        type=float,
        default=60.0
    )

    parser.add_argument(
        "--center-y",
        type=float,
        default=0.0
    )

    parser.add_argument(
        "--z",
        type=float,
        default=10.0,
        help="Target altitude [m]"
    )

    # Circle
    parser.add_argument(
        "--radius",
        type=float,
        default=100.0,
        help="Circle radius [m]"
    )

    # Ellipse
    parser.add_argument(
        "--a",
        type=float,
        default=150.0,
        help="Ellipse semi-major axis [m]"
    )

    parser.add_argument(
        "--b",
        type=float,
        default=75.0,
        help="Ellipse semi-minor axis [m]"
    )

    # Rectangle
    parser.add_argument(
        "--width",
        type=float,
        default=200.0,
        help="Rectangle width [m]"
    )

    parser.add_argument(
        "--height",
        type=float,
        default=100.0,
        help="Rectangle height [m]"
    )

    # Model orientation correction
    parser.add_argument(
        "--yaw-offset",
        type=float,
        default=0.0,
        help="Additional model yaw offset [degrees]"
    )

    args = parser.parse_args()

    # Gazebo Transport Node
    # 프로그램 시작 시 한 번만 생성하고 계속 재사용한다.
    node = Node()

    # --------------------------------------------------------
    # Validation
    # --------------------------------------------------------

    if args.speed <= 0:
        print("[ERROR] --speed must be greater than 0.")
        sys.exit(1)

    if args.rate <= 0:
        print("[ERROR] --rate must be greater than 0.")
        sys.exit(1)

    if args.radius <= 0:
        print("[ERROR] --radius must be greater than 0.")
        sys.exit(1)

    if args.a <= 0 or args.b <= 0:
        print("[ERROR] ellipse a/b must be greater than 0.")
        sys.exit(1)

    if args.width <= 0 or args.height <= 0:
        print("[ERROR] rectangle width/height must be greater than 0.")
        sys.exit(1)

    yaw_offset = math.radians(args.yaw_offset)

    # --------------------------------------------------------
    # Ellipse pre-calculation
    # --------------------------------------------------------

    ellipse = None

    if args.path == "ellipse":

        ellipse = EllipsePath(
            args.center_x,
            args.center_y,
            args.z,
            args.a,
            args.b
        )

    # --------------------------------------------------------
    # Information
    # --------------------------------------------------------

    print()
    print("==============================================")
    print(" Moving Standard VTOL Target")
    print("==============================================")
    print(f"World       : {args.world}")
    print(f"Model       : {args.model}")
    print(f"Path        : {args.path}")
    print(f"Speed       : {args.speed:.2f} m/s")
    print(f"             {args.speed * 3.6:.1f} km/h")
    print(f"Altitude    : {args.z:.2f} m")
    print(f"Center      : ({args.center_x:.1f}, {args.center_y:.1f})")
    print(f"Update rate : {args.rate:.1f} Hz")

    if args.path == "circle":
        print(f"Radius      : {args.radius:.1f} m")

    elif args.path == "ellipse":
        print(f"a / b       : {args.a:.1f} / {args.b:.1f} m")
        print(f"Path length : {ellipse.length:.1f} m")

    elif args.path == "rectangle":
        print(f"Width       : {args.width:.1f} m")
        print(f"Height      : {args.height:.1f} m")

    print(f"Yaw offset  : {args.yaw_offset:.1f} deg")
    print("----------------------------------------------")
    print("Press Ctrl+C to stop.")
    print("==============================================")
    print()

    # --------------------------------------------------------
    # Main loop
    # --------------------------------------------------------

    period = 1.0 / args.rate

    start_time = time.monotonic()
    next_update = start_time

    update_count = 0
    last_report = start_time

    try:

        while True:

            now = time.monotonic()

            # 실제 경과 시간으로 이동 거리 계산
            elapsed = now - start_time
            distance = args.speed * elapsed

            # Path calculation
            if args.path == "circle":

                x, y, z, yaw = circle_pose(
                    distance,
                    args.center_x,
                    args.center_y,
                    args.z,
                    args.radius
                )

            elif args.path == "ellipse":

                x, y, z, yaw = ellipse.pose(distance)

            else:

                x, y, z, yaw = rectangle_pose(
                    distance,
                    args.center_x,
                    args.center_y,
                    args.z,
                    args.width,
                    args.height
                )

            yaw += yaw_offset

            success = set_pose(
                node,
                args.world,
                args.model,
                x,
                y,
                z,
                yaw
            )

            if not success:
                print(
                    "[WARN] set_pose failed. "
                    "Check Gazebo world/model name."
                )

            update_count += 1

            # 1초마다 현재 상태 표시
            if now - last_report >= 1.0:

                actual_rate = update_count / (now - last_report)

                print(
                    f"[TARGET] "
                    f"x={x:8.2f} "
                    f"y={y:8.2f} "
                    f"z={z:6.2f} "
                    f"speed={args.speed:5.1f} m/s "
                    f"update={actual_rate:4.1f} Hz"
                )

                update_count = 0
                last_report = now

            # 일정 update rate 유지
            next_update += period

            sleep_time = next_update - time.monotonic()

            if sleep_time > 0:
                time.sleep(sleep_time)

            else:
                # 실행이 너무 느려 schedule이 밀린 경우
                next_update = time.monotonic()

    except KeyboardInterrupt:

        print()
        print("[INFO] Moving target stopped.")


if __name__ == "__main__":
    main()
