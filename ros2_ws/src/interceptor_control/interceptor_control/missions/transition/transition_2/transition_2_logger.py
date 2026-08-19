#!/usr/bin/env python3

import math
from typing import Any

from interceptor_control.utils.mission_logger import MissionLogger


class Transition2Logger:

    FIELDNAMES = [
        "time",
        "state",

        "x",
        "y",
        "z",

        "vx",
        "vy",
        "vz",

        "speed_xy",
        "speed_3d",

        "target_pitch",
        "actual_roll",
        "actual_pitch",
        "actual_yaw",
    ]

    def __init__(self, workspace_path):

        self.common_logger = MissionLogger(
            mission_name="transition_2",
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
        vx,
        vy,
        vz,
        target_pitch,
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

            "vx": vx,
            "vy": vy,
            "vz": vz,

            "speed_xy": speed_xy,
            "speed_3d": speed_3d,

            "target_pitch": target_pitch,

            "actual_roll": roll,
            "actual_pitch": pitch,
            "actual_yaw": yaw,
        }

        self.rows.append(row)
        self.common_logger.log(row)


    def create_summary(self):

        if not self.rows:
            return {
                "Result": "No transition_2 mission data recorded."
            }

        pitch_rows = [
            row
            for row in self.rows
            if row["state"] == "PITCH_STEP"
        ]

        if pitch_rows:

            pitch_errors = [
                abs(
                    row["target_pitch"]
                    - row["actual_pitch"]
                )
                for row in pitch_rows
            ]

            speeds = [
                row["speed_xy"]
                for row in pitch_rows
            ]

            altitudes = [
                row["z"]
                for row in pitch_rows
            ]

            mean_pitch_error = (
                sum(pitch_errors) / len(pitch_errors)
            )

            max_pitch_error = max(pitch_errors)

            max_speed = max(speeds)

            min_z = min(altitudes)
            max_z = max(altitudes)

            altitude_change = abs(max_z - min_z)
            # -------------------------------------------------
            # Pitch response time
            # 목표 Pitch ±0.5 deg 범위에 처음 진입한 시간
            # -------------------------------------------------
            pitch_start_time = pitch_rows[0]["time"]

            response_time = None

            for row in pitch_rows:
                pitch_error = abs(
                    row["target_pitch"]
                    - row["actual_pitch"]
                )

                if pitch_error <= 0.5:
                    response_time = (
                        row["time"] - pitch_start_time
                    )
                    break


            # -------------------------------------------------
            # Steady-state pitch error
            # Pitch Step 마지막 0.5초 구간 평균 오차
            # -------------------------------------------------
            pitch_end_time = pitch_rows[-1]["time"]

            steady_rows = [
                row
                for row in pitch_rows
                if row["time"] >= pitch_end_time - 0.5
            ]

            if steady_rows:
                steady_errors = [
                    abs(
                        row["target_pitch"]
                        - row["actual_pitch"]
                    )
                    for row in steady_rows
                ]

                steady_state_pitch_error = (
                    sum(steady_errors)
                    / len(steady_errors)
                )

            else:
                steady_state_pitch_error = 0.0
        else:
            mean_pitch_error = 0.0
            max_pitch_error = 0.0
            max_speed = 0.0
            altitude_change = 0.0

            response_time = None
            steady_state_pitch_error = 0.0

        total_time = self.rows[-1]["time"]

        last_row = self.rows[-1]

        landing_xy_error = math.sqrt(
            last_row["x"] ** 2
            + last_row["y"] ** 2
        )

        summary = {
            "Total Mission Time":
                f"{total_time:.2f} s",

            "Mean Pitch Tracking Error":
                f"{mean_pitch_error:.3f} deg",

            "Max Pitch Tracking Error":
                f"{max_pitch_error:.3f} deg",
                
            "Pitch Response Time":
                (
                    f"{response_time:.3f} s"
                    if response_time is not None
                    else "Not reached"
                ),

            "Steady-State Pitch Error":
                f"{steady_state_pitch_error:.3f} deg",

            "Max Speed During Pitch Step":
                f"{max_speed:.3f} m/s",

            "Altitude Variation During Pitch Step":
                f"{altitude_change:.3f} m",

            "Landing XY Error":
                f"{landing_xy_error:.3f} m",

            "CSV File":
                str(self.csv_path),
        }

        return summary


    def finish(self):

        summary = self.create_summary()

        self.common_logger.write_report(summary)
        self.common_logger.close()

        return (
            str(self.csv_path),
            str(self.summary_path),
        )