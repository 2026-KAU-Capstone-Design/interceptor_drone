#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime


class HoverLogger:
    def __init__(self, result_dir):
        self.result_dir = result_dir
        self.csv_dir = os.path.join(result_dir, "csv")
        self.report_dir = os.path.join(result_dir, "reports")

        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.report_dir, exist_ok=True)

        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.csv_dir, f"hover_result_{now}.csv")
        self.summary_path = os.path.join(self.report_dir, f"hover_summary_{now}.txt")

        self.rows = []

    def log(self, t, x, y, z, target_x, target_y, target_z, vx, vy, vz, state):
        error_x = x - target_x
        error_y = y - target_y
        error_z = z - target_z

        horizontal_error = math.sqrt(error_x ** 2 + error_y ** 2)
        position_error_3d = math.sqrt(error_x ** 2 + error_y ** 2 + error_z ** 2)
        speed_xy = math.sqrt(vx ** 2 + vy ** 2)
        speed_3d = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        self.rows.append({
            "time": t,
            "state": state,
            "x": x,
            "y": y,
            "z": z,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "error_x": error_x,
            "error_y": error_y,
            "error_z": error_z,
            "horizontal_error": horizontal_error,
            "position_error_3d": position_error_3d,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "speed_xy": speed_xy,
            "speed_3d": speed_3d,
        })

    def save_csv(self):
        if not self.rows:
            return

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)

    def save_summary(self):
        hover_rows = [row for row in self.rows if row["state"] == "HOVER"]

        if not hover_rows:
            summary = "No hover data recorded.\n"
        else:
            horizontal_errors = [row["horizontal_error"] for row in hover_rows]
            altitude_errors = [abs(row["error_z"]) for row in hover_rows]
            speed_xy_values = [row["speed_xy"] for row in hover_rows]

            mean_xy_error = sum(horizontal_errors) / len(horizontal_errors)
            max_xy_error = max(horizontal_errors)
            rmse_xy_error = math.sqrt(
                sum(e ** 2 for e in horizontal_errors) / len(horizontal_errors)
            )

            mean_alt_error = sum(altitude_errors) / len(altitude_errors)
            max_alt_error = max(altitude_errors)
            rmse_alt_error = math.sqrt(
                sum(e ** 2 for e in altitude_errors) / len(altitude_errors)
            )

            mean_speed_xy = sum(speed_xy_values) / len(speed_xy_values)
            max_speed_xy = max(speed_xy_values)

            last_row = self.rows[-1]
            landing_error = math.sqrt(last_row["x"] ** 2 + last_row["y"] ** 2)

            summary = f"""
========== Hover Mission Summary ==========

Hover Samples              : {len(hover_rows)}
Mean Horizontal Error       : {mean_xy_error:.3f} m
Max Horizontal Error        : {max_xy_error:.3f} m
RMSE Horizontal Error       : {rmse_xy_error:.3f} m

Mean Altitude Error         : {mean_alt_error:.3f} m
Max Altitude Error          : {max_alt_error:.3f} m
RMSE Altitude Error         : {rmse_alt_error:.3f} m

Mean XY Speed               : {mean_speed_xy:.3f} m/s
Max XY Speed                : {max_speed_xy:.3f} m/s

Landing XY Error            : {landing_error:.3f} m

CSV File:
{self.csv_path}

===========================================
"""

        with open(self.summary_path, "w") as f:
            f.write(summary)

    def finish(self):
        self.save_csv()
        self.save_summary()
        return self.csv_path, self.summary_path
