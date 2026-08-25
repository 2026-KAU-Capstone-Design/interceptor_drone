#!/usr/bin/env python3

import subprocess
import time


WORLD = "rocket_intercept_test"
TARGET = "target_uav"

# Initial target position [Gazebo ENU]
X = 8.0
Y = 0.0
Z = 8.0

# Target velocity
SPEED_Y = 3.0     # m/s

# Update period
DT = 0.1            # 10 Hz


def set_target_pose(x, y, z):
    request = (
        f'name: "{TARGET}", '
        f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
        f'orientation: {{w: 1}}'
    )

    subprocess.run(
        [
            "gz", "service",
            "-s", f"/world/{WORLD}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", request,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main():
    y = Y

    print("=== Moving Target UAV ===")
    print(f"start : x={X:.1f}, y={Y:.1f}, z={Z:.1f}")
    print(f"speed : {SPEED_Y:.1f} m/s (+Y)")
    print("Ctrl+C to stop")

    try:
        while True:

            y += SPEED_Y * DT

            set_target_pose(
                X,
                y,
                Z,
            )

            print(
                f"\rtarget_uav : "
                f"x={X:.2f} "
                f"y={y:.2f} "
                f"z={Z:.2f}",
                end="",
                flush=True,
            )

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nTarget motion stopped.")


if __name__ == "__main__":
    main()
