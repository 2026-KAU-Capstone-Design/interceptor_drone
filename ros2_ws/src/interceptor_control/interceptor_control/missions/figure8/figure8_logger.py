#!/usr/bin/env python3

import math
from typing import Any

from interceptor_control.utils.mission_logger import MissionLogger


class Figure8Logger:
    """Circle 미션 전용 로거."""

    FIELDNAMES = [
        "time",
        "state",
        "theta",
        "x",
        "y",
        "z",
        "target_x",
        "target_y",
        "target_z",
        "center_x",
        "center_y",
        "target_radius",
        "actual_radius",
        "radius_error",
        "altitude_error",
        "vx",
        "vy",
        "vz",
        "speed_xy",
        "speed_3d",
        "roll",
        "pitch",
        "yaw",
    ]

    def __init__(self, workspace_path):
        self.common_logger = MissionLogger(
            mission_name="figure8",
            workspace_path=workspace_path,
            fieldnames=self.FIELDNAMES,
        )

        self.rows: list[dict[str, Any]] = []
        self.lap_completed = False
        self.lap_time = None

        self.csv_path = self.common_logger.csv_path
        self.summary_path = self.common_logger.report_path

    def log(
        self,
        t,
        state,
        theta,
        x,
        y,
        z,
        target_x,
        target_y,
        target_z,
        center_x,
        center_y,
        target_radius,
        vx,
        vy,
        vz,
        roll,
        pitch,
        yaw,
    ):
        actual_radius = math.sqrt(
            (x - center_x) ** 2
            + (y - center_y) ** 2
        )

        radius_error = actual_radius - target_radius
        altitude_error = z - target_z

        speed_xy = math.sqrt(vx**2 + vy**2)
        speed_3d = math.sqrt(vx**2 + vy**2 + vz**2)

        row = {
            "time": t,
            "state": state,
            "theta": theta,
            "x": x,
            "y": y,
            "z": z,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "center_x": center_x,
            "center_y": center_y,
            "target_radius": target_radius,
            "actual_radius": actual_radius,
            "radius_error": radius_error,
            "altitude_error": altitude_error,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "speed_xy": speed_xy,
            "speed_3d": speed_3d,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

        self.rows.append(row)
        self.common_logger.log(row)

    def record_lap_complete(self, lap_time):
        self.lap_completed = True
        self.lap_time = lap_time

    def create_summary(self):
        if not self.rows:
            return {
                "Result": "No circle mission data recorded.",
            }

        circle_rows = [
            row
            for row in self.rows
            if row["state"] == "CIRCLE"
        ]

        if circle_rows:
            radius_errors = [
                abs(row["radius_error"])
                for row in circle_rows
            ]

            altitude_errors = [
                abs(row["altitude_error"])
                for row in circle_rows
            ]

            speed_values = [
                row["speed_xy"]
                for row in circle_rows
            ]

            roll_values = [
                abs(row["roll"])
                for row in circle_rows
            ]

            pitch_values = [
                abs(row["pitch"])
                for row in circle_rows
            ]

            mean_radius_error = (
                sum(radius_errors) / len(radius_errors)
            )
            max_radius_error = max(radius_errors)

            radius_rmse = math.sqrt(
                sum(error**2 for error in radius_errors)
                / len(radius_errors)
            )

            mean_altitude_error = (
                sum(altitude_errors) / len(altitude_errors)
            )
            max_altitude_error = max(altitude_errors)

            mean_speed = (
                sum(speed_values) / len(speed_values)
            )
            max_speed = max(speed_values)

            mean_roll = (
                sum(roll_values) / len(roll_values)
            )
            max_roll = max(roll_values)

            mean_pitch = (
                sum(pitch_values) / len(pitch_values)
            )
            max_pitch = max(pitch_values)

        else:
            mean_radius_error = 0.0
            max_radius_error = 0.0
            radius_rmse = 0.0

            mean_altitude_error = 0.0
            max_altitude_error = 0.0

            mean_speed = 0.0
            max_speed = 0.0

            mean_roll = 0.0
            max_roll = 0.0

            mean_pitch = 0.0
            max_pitch = 0.0

        total_time = self.rows[-1]["time"]

        last_row = self.rows[-1]
        landing_xy_error = math.sqrt(
            last_row["x"] ** 2
            + last_row["y"] ** 2
        )

        lap_time_text = (
            f"{self.lap_time:.2f} s"
            if self.lap_time is not None
            else "Not completed"
        )

        return {
            "Total Mission Time": f"{total_time:.2f} s",
            "Lap Completed": self.lap_completed,
            "Lap Time": lap_time_text,
            "Mean Radius Error": (
                f"{mean_radius_error:.3f} m"
            ),
            "Max Radius Error": (
                f"{max_radius_error:.3f} m"
            ),
            "Radius RMSE": (
                f"{radius_rmse:.3f} m"
            ),
            "Mean Altitude Error": (
                f"{mean_altitude_error:.3f} m"
            ),
            "Max Altitude Error": (
                f"{max_altitude_error:.3f} m"
            ),
            "Mean XY Speed": (
                f"{mean_speed:.3f} m/s"
            ),
            "Max XY Speed": (
                f"{max_speed:.3f} m/s"
            ),
            "Mean Absolute Roll": (
                f"{math.degrees(mean_roll):.2f} deg"
            ),
            "Max Absolute Roll": (
                f"{math.degrees(max_roll):.2f} deg"
            ),
            "Mean Absolute Pitch": (
                f"{math.degrees(mean_pitch):.2f} deg"
            ),
            "Max Absolute Pitch": (
                f"{math.degrees(max_pitch):.2f} deg"
            ),
            "Landing XY Error": (
                f"{landing_xy_error:.3f} m"
            ),
            "CSV File": str(self.csv_path),
        }

    def finish(self):
        summary = self.create_summary()

        self.common_logger.write_report(summary)
        self.common_logger.close()

        return str(self.csv_path), str(self.summary_path)
