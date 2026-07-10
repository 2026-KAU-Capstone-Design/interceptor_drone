from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Any


class MissionLogger:
    """미션별 CSV와 요약 보고서를 저장하는 공통 로거."""

    def __init__(
        self,
        mission_name: str,
        workspace_path: str | Path,
        fieldnames: list[str],
    ) -> None:
        self.mission_name = mission_name.lower()
        self.fieldnames = fieldnames
        self.started_at = datetime.now()

        workspace = Path(workspace_path).expanduser().resolve()
        timestamp = self.started_at.strftime("%Y%m%d_%H%M%S")

        self.csv_dir = workspace / "build" / "results" / "csv" / self.mission_name
        self.report_dir = (
            workspace / "build" / "results" / "reports" / self.mission_name
        )

        self.csv_dir.mkdir(parents=True, exist_ok=True)
        self.report_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = self.csv_dir / f"{self.mission_name}_{timestamp}.csv"
        self.report_path = (
            self.report_dir / f"{self.mission_name}_summary_{timestamp}.txt"
        )

        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=self.fieldnames,
            extrasaction="ignore",
        )
        self._writer.writeheader()
        self._csv_file.flush()

    def log(self, row: dict[str, Any]) -> None:
        """CSV에 한 행을 기록한다."""
        complete_row = {field: row.get(field, "") for field in self.fieldnames}
        self._writer.writerow(complete_row)
        self._csv_file.flush()

    def write_report(self, summary: dict[str, Any]) -> None:
        """미션 종료 후 텍스트 요약 보고서를 저장한다."""
        finished_at = datetime.now()

        with self.report_path.open("w", encoding="utf-8") as report:
            report.write(f"Mission: {self.mission_name}\n")
            report.write(f"Started: {self.started_at.isoformat(timespec='seconds')}\n")
            report.write(f"Finished: {finished_at.isoformat(timespec='seconds')}\n")
            report.write(
                f"Elapsed: {(finished_at - self.started_at).total_seconds():.3f} s\n"
            )
            report.write("-" * 50 + "\n")

            for key, value in summary.items():
                report.write(f"{key}: {value}\n")

    def close(self) -> None:
        if not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()

    def __enter__(self) -> "MissionLogger":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
