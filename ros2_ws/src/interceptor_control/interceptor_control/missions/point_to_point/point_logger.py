#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime


class PointLogger:
    def __init__(self, result_dir):
        self.csv_dir = os.path.join(result_dir, "csv")
        self.report_dir = os.path.join(result_dir, "reports")

        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.report_dir, exist_ok=True)

        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.csv_dir, f"point_result_{now}.csv")
        self.summary_path = os.path.join(self.report_dir, f"point_summary_{now}.txt")

        self.rows = []
        self.wp_events = []

    def log(self, t, state, wp_index, x, y, z, target_x, target_y, target_z, vx, vy, vz):
        error_x = x - target_x
        error_y = y - target_y
        error_z = z - target_z

        distance_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        horizontal_error = math.sqrt(error_x**2 + error_y**2)
        speed_xy = math.sqrt(vx**2 + vy**2)
        speed_3d = math.sqrt(vx**2 + vy**2 + vz**2)

        self.rows.append({
            "time": t,
            "state": state,
            "wp_index": wp_index,
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
            "distance_error": distance_error,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "speed_xy": speed_xy,
            "speed_3d": speed_3d,
        })

    def record_waypoint_reached(self, wp_index, t, distance_error, x, y, z):
        self.wp_events.append({
            "wp_index": wp_index,
            "arrival_time": t,
            "arrival_error": distance_error,
            "arrival_x": x,
            "arrival_y": y,
            "arrival_z": z,
        })

    def save_csv(self):
        if not self.rows:
            return

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)

    def save_summary(self):
        move_rows = [r for r in self.rows if r["state"] in ["MOVE", "HOLD"]]

        if not self.rows:
            summary = "No point-to-point data recorded.\n"
        else:
            if move_rows:
                speed_xy_values = [r["speed_xy"] for r in move_rows]
                distance_errors = [r["distance_error"] for r in move_rows]

                mean_speed_xy = sum(speed_xy_values) / len(speed_xy_values)
                max_speed_xy = max(speed_xy_values)
                mean_distance_error = sum(distance_errors) / len(distance_errors)
                max_distance_error = max(distance_errors)
            else:
                mean_speed_xy = 0.0
                max_speed_xy = 0.0
                mean_distance_error = 0.0
                max_distance_error = 0.0

            total_time = self.rows[-1]["time"]

            last_row = self.rows[-1]
            landing_xy_error = math.sqrt(last_row["x"]**2 + last_row["y"]**2)

            wp_text = ""
            for event in self.wp_events:
                wp_text += (
                    f"WP{event['wp_index']} | "
                    f"Arrival Time: {event['arrival_time']:.2f} sec | "
                    f"Arrival Error: {event['arrival_error']:.3f} m | "
                    f"Position: ({event['arrival_x']:.2f}, "
                    f"{event['arrival_y']:.2f}, "
                    f"{event['arrival_z']:.2f})\n"
                )

            summary = f"""
========== Point-to-Point Mission Summary ==========

Total Mission Time          : {total_time:.2f} sec

Mean XY Speed               : {mean_speed_xy:.3f} m/s
Max XY Speed                : {max_speed_xy:.3f} m/s

Mean Distance Error         : {mean_distance_error:.3f} m
Max Distance Error          : {max_distance_error:.3f} m

Landing XY Error            : {landing_xy_error:.3f} m

---------- Waypoint Arrival Result ----------
{wp_text}
CSV File:
{self.csv_path}

====================================================
"""

        with open(self.summary_path, "w") as f:
            f.write(summary)

    def finish(self):
        self.save_csv()
        self.save_summary()
        return self.csv_path, self.summary_path
