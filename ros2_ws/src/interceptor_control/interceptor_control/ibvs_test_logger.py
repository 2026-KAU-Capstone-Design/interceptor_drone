"""IBVS SITL 시험 결과 기록기."""

import csv
from datetime import datetime
from pathlib import Path


class IBVSTestLogger:
    def __init__(self, result_dir, test_name):
        self.test_name = test_name
        self.started_at = datetime.now()
        self.rows = []

        root = Path(result_dir)
        self.csv_dir = root / "csv"
        self.report_dir = root / "reports"

        self.csv_dir.mkdir(parents=True, exist_ok=True)
        self.report_dir.mkdir(parents=True, exist_ok=True)

    def add(self, elapsed, state, dx, dy, distance,
            vx, vy, vz, yaw_rate, lock_on,
            vx_actual=None, vy_actual=None, vz_actual=None,
            x=None, y=None, z=None, track_travel=None):
        self.rows.append({
            "elapsed_sec": round(elapsed, 3),
            "state": str(state),
            "dx_px": float(dx),
            "dy_px": float(dy),
            "distance_est_m": float(distance),
            "vx_cmd_mps": float(vx),
            "vy_cmd_mps": float(vy),
            "vz_cmd_mps": float(vz),
            "yaw_rate_cmd_radps": float(yaw_rate),
            "lock_on": bool(lock_on),
            "vx_actual_mps": vx_actual,
            "vy_actual_mps": vy_actual,
            "vz_actual_mps": vz_actual,
            "x_ned_m": x,
            "y_ned_m": y,
            "z_ned_m": z,
            "track_displacement_m": track_travel,
        })

    def save(self, outcome="UNVERIFIED", reason=""):
        if outcome not in ("PASS", "FAIL", "UNVERIFIED"):
            raise ValueError("잘못된 시험 결과입니다.")

        stamp = self.started_at.strftime("%Y%m%d_%H%M%S")
        stem = f"{self.test_name}_{stamp}"

        csv_path = self.csv_dir / f"{stem}.csv"
        report_path = self.report_dir / f"{stem}.txt"

        fields = [
            "elapsed_sec", "state", "dx_px", "dy_px",
            "distance_est_m", "vx_cmd_mps", "vy_cmd_mps",
            "vz_cmd_mps", "yaw_rate_cmd_radps", "lock_on",
            "vx_actual_mps", "vy_actual_mps", "vz_actual_mps",
            "x_ned_m", "y_ned_m", "z_ned_m",
            "track_displacement_m",
        ]

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)

        report = (
            f"IBVS SITL Test Report\n"
            f"Test: {self.test_name}\n"
            f"Started: {self.started_at.isoformat(timespec='seconds')}\n"
            f"Outcome: {outcome}\n"
            f"Reason: {reason}\n"
            f"Samples: {len(self.rows)}\n"
            f"Note: PASS is a test outcome, not proof of target impact.\n"
        )
        report_path.write_text(report, encoding="utf-8")

        return csv_path, report_path
