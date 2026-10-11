from __future__ import annotations

import csv
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ANALYSIS_ROOT = REPO_ROOT / "analysis"
SUMMARY_PATH = ANALYSIS_ROOT / "altitude_prediction_dropout_summary.csv"
EVENTS_PATH = ANALYSIS_ROOT / "altitude_prediction_dropout_events.csv"


def number(row: dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(name, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def previous_barometer_row(
    rows: list[dict[str, str]], index: int
) -> dict[str, str] | None:
    timestamp = int(number(rows[index], "barometer_sample_timestamp_ms", -1.0))
    for candidate in reversed(rows[:index]):
        candidate_timestamp = int(
            number(candidate, "barometer_sample_timestamp_ms", -1.0)
        )
        if candidate_timestamp != timestamp:
            return candidate
    return None


def analyze(path: Path, generation: str) -> tuple[dict[str, object], list[dict[str, object]]]:
    rows = load_rows(path)
    nonzero = [
        index
        for index, row in enumerate(rows)
        if abs(number(row, "predicted_apogee_m")) > 0.0001
    ]
    valid_rows = sum(int(number(row, "altitude_prediction_valid")) == 1 for row in rows)
    peak_altitude = max((number(row, "relative_altitude_m") for row in rows), default=0.0)
    events: list[dict[str, object]] = []

    if len(nonzero) >= 2:
        first = nonzero[0]
        last = nonzero[-1]
        in_zero = False
        for index in range(first, last + 1):
            is_zero = abs(number(rows[index], "predicted_apogee_m")) <= 0.0001
            if is_zero and not in_zero:
                resume_index = next(
                    (
                        candidate
                        for candidate in range(index + 1, last + 1)
                        if abs(number(rows[candidate], "predicted_apogee_m")) > 0.0001
                    ),
                    None,
                )
                current = rows[index]
                previous = previous_barometer_row(rows, index)
                raw_baro_speed = None
                mismatch = None
                if previous is not None:
                    dt_s = (
                        number(current, "barometer_sample_timestamp_ms")
                        - number(previous, "barometer_sample_timestamp_ms")
                    ) / 1000.0
                    if dt_s > 0.0:
                        raw_baro_speed = (
                            number(current, "relative_altitude_m")
                            - number(previous, "relative_altitude_m")
                        ) / dt_s
                        mismatch = abs(
                            raw_baro_speed - number(current, "imu_vertical_speed_mps")
                        )
                events.append(
                    {
                        "generation": generation,
                        "log": path.parent.name,
                        "repository_path": path.relative_to(REPO_ROOT).as_posix(),
                        "zero_start_s": f'{number(current, "time_s"):.3f}',
                        "resume_s": (
                            f'{number(rows[resume_index], "time_s"):.3f}'
                            if resume_index is not None
                            else ""
                        ),
                        "zero_duration_s": (
                            f'{number(rows[resume_index], "time_s") - number(current, "time_s"):.3f}'
                            if resume_index is not None
                            else ""
                        ),
                        "relative_altitude_m": f'{number(current, "relative_altitude_m"):.3f}',
                        "imu_vertical_speed_mps": f'{number(current, "imu_vertical_speed_mps"):.3f}',
                        "raw_baro_speed_mps": (
                            f"{raw_baro_speed:.3f}" if raw_baro_speed is not None else ""
                        ),
                        "baro_imu_mismatch_mps": (
                            f"{mismatch:.3f}" if mismatch is not None else ""
                        ),
                        "barometer_valid": int(number(current, "barometer_valid")),
                        "barometer_sample_age_ms": int(
                            number(current, "barometer_sample_age_ms")
                        ),
                    }
                )
            in_zero = is_zero

    summary = {
        "generation": generation,
        "log": path.parent.name,
        "repository_path": path.relative_to(REPO_ROOT).as_posix(),
        "records": len(rows),
        "peak_relative_altitude_m": f"{peak_altitude:.3f}",
        "prediction_first_s": (
            f'{number(rows[nonzero[0]], "time_s"):.3f}' if nonzero else ""
        ),
        "prediction_last_s": (
            f'{number(rows[nonzero[-1]], "time_s"):.3f}' if nonzero else ""
        ),
        "prediction_valid_rows": valid_rows,
        "internal_zero_runs": len(events),
    }
    return summary, events


def find_logs() -> list[tuple[Path, str]]:
    logs: list[tuple[Path, str]] = []
    legacy_root = REPO_ROOT / "archive" / "legacy_202608"
    for directory in sorted(legacy_root.glob("flight_log_*")):
        path = directory / "flight_log.csv"
        if path.is_file():
            logs.append((path, "legacy_202608"))

    archive_root = REPO_ROOT / "archive"
    for directory in sorted(archive_root.glob("flight_log_202610*")):
        path = directory / "flight_log.csv"
        if path.is_file():
            logs.append((path, "current_202610"))
    return logs


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    summaries: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    for path, generation in find_logs():
        summary, log_events = analyze(path, generation)
        summaries.append(summary)
        events.extend(log_events)

    write_csv(
        SUMMARY_PATH,
        summaries,
        [
            "generation",
            "log",
            "repository_path",
            "records",
            "peak_relative_altitude_m",
            "prediction_first_s",
            "prediction_last_s",
            "prediction_valid_rows",
            "internal_zero_runs",
        ],
    )
    write_csv(
        EVENTS_PATH,
        events,
        [
            "generation",
            "log",
            "repository_path",
            "zero_start_s",
            "resume_s",
            "zero_duration_s",
            "relative_altitude_m",
            "imu_vertical_speed_mps",
            "raw_baro_speed_mps",
            "baro_imu_mismatch_mps",
            "barometer_valid",
            "barometer_sample_age_ms",
        ],
    )
    print(f"Wrote {len(summaries)} summaries and {len(events)} dropout events")


if __name__ == "__main__":
    main()
