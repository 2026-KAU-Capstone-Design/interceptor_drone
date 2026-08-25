#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime


class RocketTrackLogger:
    def __init__(self, workspace_path):

        result_root = os.path.join(
            workspace_path,
            "build",
            "results",
        )

        self.csv_dir = os.path.join(
            result_root,
            "csv",
            "rocket_track",
        )

        self.report_dir = os.path.join(
            result_root,
            "reports",
            "rocket_track",
        )

        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.report_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_path = os.path.join(
            self.csv_dir,
            f"rocket_track_{timestamp}.csv",
        )

        self.report_path = os.path.join(
            self.report_dir,
            f"rocket_track_summary_{timestamp}.txt",
        )

        self.start_time = None
        self.end_time = None

        self.rows = []

        self.intercept_success = False
        self.target_lost_count = 0

        self.fieldnames = [
            "time",
            "state",
            "detected",
            "lock_on",
            "near_2m",
            "x",
            "y",
            "z",
            "dx",
            "dy",
            "current_yaw_deg",
            "target_yaw_deg",
            "target_z",
            "yaw_rate",
            "forward_speed",
            "vx",
            "vy",
        ]

        self.csv_file = open(
            self.csv_path,
            "w",
            newline="",
            encoding="utf-8",
        )

        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=self.fieldnames,
        )

        self.writer.writeheader()

    def log(
        self,
        t,
        state,
        detected,
        lock_on,
        near_2m,
        x,
        y,
        z,
        dx,
        dy,
        current_yaw,
        target_yaw,
        target_z,
        yaw_rate=0.0,
        forward_speed=0.0,
        vx=0.0,
        vy=0.0,
    ):

        if self.start_time is None:
            self.start_time = t

        self.end_time = t

        row = {
            "time": t,
            "state": state,
            "detected": int(bool(detected)),
            "lock_on": int(bool(lock_on)),
            "near_2m": int(bool(near_2m)),
            "x": x,
            "y": y,
            "z": z,
            "dx": dx,
            "dy": dy,
            "current_yaw_deg": math.degrees(current_yaw),
            "target_yaw_deg": math.degrees(target_yaw),
            "target_z": target_z,
            "yaw_rate": yaw_rate,
            "forward_speed": forward_speed,
            "vx": vx,
            "vy": vy,
        }

        self.rows.append(row)
        self.writer.writerow(row)
        self.csv_file.flush()

    def mark_target_lost(self):
        self.target_lost_count += 1

    def mark_intercept(self):
        self.intercept_success = True

    def close(self):

        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()

        if not self.rows:
            return

        elapsed = 0.0

        if (
            self.start_time is not None
            and self.end_time is not None
        ):
            elapsed = self.end_time - self.start_time

        abs_dx = [
            abs(row["dx"])
            for row in self.rows
        ]

        abs_dy = [
            abs(row["dy"])
            for row in self.rows
        ]

        speeds = [
            row["forward_speed"]
            for row in self.rows
        ]

        mean_dx = sum(abs_dx) / len(abs_dx)
        max_dx = max(abs_dx)

        mean_dy = sum(abs_dy) / len(abs_dy)
        max_dy = max(abs_dy)

        max_speed = max(speeds)

        with open(
            self.report_path,
            "w",
            encoding="utf-8",
        ) as f:

            f.write("=== Rocket Track Mission Summary ===\n\n")

            f.write(
                f"Intercept        : "
                f"{'SUCCESS' if self.intercept_success else 'FAIL'}\n"
            )

            f.write(
                f"Mission Time     : {elapsed:.3f} s\n"
            )

            f.write(
                f"Mean |dx|        : {mean_dx:.2f} px\n"
            )

            f.write(
                f"Max |dx|         : {max_dx:.2f} px\n"
            )

            f.write(
                f"Mean |dy|        : {mean_dy:.2f} px\n"
            )

            f.write(
                f"Max |dy|         : {max_dy:.2f} px\n"
            )

            f.write(
                f"Target Lost      : "
                f"{self.target_lost_count}\n"
            )

            f.write(
                f"Max Cmd Speed    : {max_speed:.2f} m/s\n"
            )

            f.write(
                f"\nCSV              : {self.csv_path}\n"
            )


