#!/usr/bin/env python3

import math
from typing import Any

from interceptor_control.utils.mission_logger import MissionLogger


class PointLogger:
    """Point-to-Point 미션 전용 로거."""

    FIELDNAMES = [
        "time",
        "state",
        "wp_index",
        "x",
        "y",
        "z",
        "target_x",
        "target_y",
        "target_z",
        "error_x",
        "error_y",
        "error_z",
        "horizontal_error",
        "distance_error",
        "vx",
        "vy",
        "vz",
        "speed_xy",
        "speed_3d",
    ]

    def __init__(self, workspace_path):
        self.common_logger = MissionLogger(
            mission_name="point",
            workspace_path=workspace_path,
            fieldnames=self.FIELDNAMES,
        )

        self.rows: list[dict[str, Any]] = []
        self.wp_events: list[dict[str, Any]] = []

        self.csv_path = self.common_logger.csv_path
        self.summary_path = self.common_logger.report_path

    def log(
        self,
        t,
        state,
        wp_index,
        x,
        y,
        z,
        target_x,
        target_y,
        target_z,
        vx,
        vy,
        vz,
    ):
        error_x = x - target_x
        error_y = y - target_y
        error_z = z - target_z

        horizontal_error = math.sqrt(error_x**2 + error_y**2)
        distance_error = math.sqrt(
            error_x**2 + error_y**2 + error_z**2
        )

        speed_xy = math.sqrt(vx**2 + vy**2)
        speed_3d = math.sqrt(vx**2 + vy**2 + vz**2)

        row = {
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
        }

        self.rows.append(row)
        self.common_logger.log(row)

    def record_waypoint_reached(
        self,
        wp_index,
        t,
        distance_error,
        x,
        y,
        z,
    ):
        self.wp_events.append(
            {
                "wp_index": wp_index,
                "arrival_time": t,
                "arrival_error": distance_error,
                "arrival_x": x,
                "arrival_y": y,
                "arrival_z": z,
            }
        )

    def create_summary(self):
        if not self.rows:
            return {
                "Result": "No point-to-point data recorded.",
            }

        move_rows = [
            row
            for row in self.rows
            if row["state"] in ["MOVE", "HOLD"]
        ]

        if move_rows:
            speed_xy_values = [
                row["speed_xy"] for row in move_rows
            ]
            distance_errors = [
                row["distance_error"] for row in move_rows
            ]

            mean_speed_xy = (
                sum(speed_xy_values) / len(speed_xy_values)
            )
            max_speed_xy = max(speed_xy_values)

            mean_distance_error = (
                sum(distance_errors) / len(distance_errors)
            )
            max_distance_error = max(distance_errors)
        else:
            mean_speed_xy = 0.0
            max_speed_xy = 0.0
            mean_distance_error = 0.0
            max_distance_error = 0.0

        total_time = self.rows[-1]["time"]

        last_row = self.rows[-1]
        landing_xy_error = math.sqrt(
            last_row["x"] ** 2 + last_row["y"] ** 2
        )

        summary = {
            "Total Mission Time": f"{total_time:.2f} s",
            "Mean XY Speed": f"{mean_speed_xy:.3f} m/s",
            "Max XY Speed": f"{max_speed_xy:.3f} m/s",
            "Mean Distance Error": (
                f"{mean_distance_error:.3f} m"
            ),
            "Max Distance Error": (
                f"{max_distance_error:.3f} m"
            ),
            "Landing XY Error": (
                f"{landing_xy_error:.3f} m"
            ),
            "Waypoint Count": len(self.wp_events),
        }

        for event in self.wp_events:
            wp_index = event["wp_index"]

            summary[f"WP{wp_index} Arrival"] = (
                f"time={event['arrival_time']:.2f} s, "
                f"error={event['arrival_error']:.3f} m, "
                f"position=("
                f"{event['arrival_x']:.2f}, "
                f"{event['arrival_y']:.2f}, "
                f"{event['arrival_z']:.2f})"
            )

        summary["CSV File"] = str(self.csv_path)

        return summary

    def finish(self):
        summary = self.create_summary()

        self.common_logger.write_report(summary)
        self.common_logger.close()

        return str(self.csv_path), str(self.summary_path)
