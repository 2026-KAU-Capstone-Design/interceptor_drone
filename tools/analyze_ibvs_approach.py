#!/usr/bin/env python3
"""IBVS SITL 제한 접근 시험 결과 분석."""

import csv
import math
import argparse
from pathlib import Path


def load_csv(path):
    with path.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def value(row, key):
    return float(row[key])


def speed(row, prefix):
    if prefix == "cmd":
        return math.hypot(
            value(row, "vx_cmd_mps"),
            value(row, "vy_cmd_mps"),
        )
    return math.hypot(
        value(row, "vx_actual_mps"),
        value(row, "vy_actual_mps"),
    )


def analyze(rows, limit):
    track = [r for r in rows if r["state"] == "TRACK"]

    moving = [
        r for r in track
        if speed(r, "cmd") > 0.05
    ]

    if not moving:
        raise ValueError("전진 명령 데이터가 없습니다.")

    first = moving[0]
    start_t = value(first, "elapsed_sec")
    start_x = value(first, "x_ned_m")
    start_y = value(first, "y_ned_m")

    approach_rows = [
        r for r in track
        if value(r, "elapsed_sec") >= start_t
    ]

    distances = [
        math.hypot(
            value(r, "x_ned_m") - start_x,
            value(r, "y_ned_m") - start_y,
        )
        for r in approach_rows
    ]

    cmd_speeds = [speed(r, "cmd") for r in moving]
    actual_speeds = [speed(r, "actual") for r in moving]

    max_distance = max(distances)
    final_distance = distances[-1]

    return {
        "track_samples": len(track),
        "moving_samples": len(moving),
        "approach_start_sec": start_t,
        "approach_start_x": start_x,
        "approach_start_y": start_y,
        "max_distance_m": max_distance,
        "final_distance_m": final_distance,
        "limit_m": limit,
        "limit_error_m": max_distance - limit,
        "mean_cmd_speed_mps": sum(cmd_speeds) / len(cmd_speeds),
        "mean_actual_speed_mps": sum(actual_speeds) / len(actual_speeds),
        "rtl_recorded": any(r["state"] == "RTL" for r in rows),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--limit", type=float, default=0.5)
    args = parser.parse_args()

    root = Path("build/results/ibvs")

    if args.csv:
        csv_path = args.csv
    else:
        files = sorted(
            (root / "csv").glob("balloon_full_mission_*.csv")
        )
        if not files:
            raise SystemExit("접근 시험 CSV가 없습니다.")
        csv_path = files[-1]

    rows = load_csv(csv_path)
    result = analyze(rows, args.limit)

    report_dir = root / "reports"
    report_dir.mkdir(parents=True, exist_ok=True)

    report_path = report_dir / (
        csv_path.stem + "_analysis.txt"
    )

    lines = [
        "IBVS SITL APPROACH ANALYSIS",
        f"CSV: {csv_path.name}",
        "",
    ]

    for key, val in result.items():
        lines.append(f"{key}: {val}")

    lines += [
        "",
        "Note:",
        "Distance origin is first recorded forward command.",
        "RTL state does not prove landing or disarm.",
        "Target contact is not verified.",
    ]

    report_path.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    print("\n".join(lines))
    print(f"\nREPORT: {report_path}")


if __name__ == "__main__":
    main()
