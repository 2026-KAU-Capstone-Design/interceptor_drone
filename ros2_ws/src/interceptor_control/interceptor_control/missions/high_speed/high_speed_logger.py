#!/usr/bin/env python3

import math
from typing import Any

from interceptor_control.utils.mission_logger import MissionLogger


class HighSpeedLogger:
    """High-Speed 미션 전용 로거."""

    FIELDNAMES = [
        "time",
        "state",
        "x",
        "y",
        "z",
        "target_x",
        "target_y",
        "target_z",
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
            mission_name="high_speed",
            workspace_path=workspace_path,
            fieldnames=self.FIELDNAMES,
        )

        self.rows: list[dict[str, Any]] = []

        self.csv_path = self.common_logger.csv_path
        self.summary_path = self.common_logger.report_path

    def log(
        self,
        t,
        state,
        x,
        y,
        z,
        target_x,
        target_y,
        target_z,
        vx,
        vy,
        vz,
        roll,
        pitch,
        yaw,
    ):
        speed_xy = math.sqrt(vx**2 + vy**2)
        speed_3d = math.sqrt(vx**2 + vy**2 + vz**2)

        row = {
            "time": t,
            "state": state,
            "x": x,
            "y": y,
            "z": z,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
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

    def create_summary(self):
        if not self.rows:
            return {
                "Result": "No high-speed mission data recorded.",
            }

        active_rows = [
            row
            for row in self.rows
            if row["state"] in [
                "ACCELERATE",
                "CRUISE",
                "DECELERATE",
            ]
        ]

        if active_rows:
            speeds = [row["speed_xy"] for row in active_rows]
            rolls = [abs(row["roll"]) for row in active_rows]
            pitches = [abs(row["pitch"]) for row in active_rows]

            mean_speed = sum(speeds) / len(speeds)
            max_speed = max(speeds)

            mean_roll = sum(rolls) / len(rolls)
            max_roll = max(rolls)

            mean_pitch = sum(pitches) / len(pitches)
            max_pitch = max(pitches)
        else:
            mean_speed = 0.0
            max_speed = 0.0
            mean_roll = 0.0
            max_roll = 0.0
            mean_pitch = 0.0
            max_pitch = 0.0

        total_distance = 0.0

        for previous, current in zip(self.rows, self.rows[1:]):
            dx = current["x"] - previous["x"]
            dy = current["y"] - previous["y"]
            dz = current["z"] - previous["z"]

            total_distance += math.sqrt(
                dx**2 + dy**2 + dz**2
            )

        total_time = self.rows[-1]["time"]

        last_row = self.rows[-1]
        landing_xy_error = math.sqrt(
            last_row["x"] ** 2
            + last_row["y"] ** 2
        )

        state_speeds = {}

        for state in ["ACCELERATE", "CRUISE", "DECELERATE"]:
            state_rows = [
                row
                for row in self.rows
                if row["state"] == state
            ]

            if state_rows:
                values = [
                    row["speed_xy"]
                    for row in state_rows
                ]

                state_speeds[f"{state} Mean Speed"] = (
                    f"{sum(values) / len(values):.3f} m/s"
                )
                state_speeds[f"{state} Max Speed"] = (
                    f"{max(values):.3f} m/s"
                )

        summary = {
            "Total Mission Time": f"{total_time:.2f} s",
            "Mean Active Speed": f"{mean_speed:.3f} m/s",
            "Max Active Speed": f"{max_speed:.3f} m/s",
            "Total Flight Distance": f"{total_distance:.3f} m",
            "Mean Absolute Roll": f"{mean_roll:.2f} deg",
            "Max Absolute Roll": f"{max_roll:.2f} deg",
            "Mean Absolute Pitch": f"{mean_pitch:.2f} deg",
            "Max Absolute Pitch": f"{max_pitch:.2f} deg",
            "Landing XY Error": f"{landing_xy_error:.3f} m",
        }

        summary.update(state_speeds)
        summary["CSV File"] = str(self.csv_path)

        return summary

    def finish(self):
        summary = self.create_summary()

        self.common_logger.write_report(summary)
        self.common_logger.close()

        return str(self.csv_path), str(self.summary_path)
