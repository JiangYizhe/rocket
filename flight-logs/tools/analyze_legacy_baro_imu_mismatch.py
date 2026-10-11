from __future__ import annotations

import csv
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
LEGACY_ROOT = REPO_ROOT / "archive" / "legacy_202608"
ANALYSIS_ROOT = REPO_ROOT / "analysis"
SUMMARY_PATH = ANALYSIS_ROOT / "legacy_baro_imu_mismatch_summary.csv"
EVENTS_PATH = ANALYSIS_ROOT / "legacy_baro_imu_mismatch_events.csv"

MIN_DT_MS = 20
MAX_DT_MS = 200
MAX_SAMPLE_AGE_MS = 120
MISMATCH_LIMIT_MPS = 20.0
LIFTOFF_SPEED_MPS = 2.0
LIFTOFF_CONSECUTIVE_ROWS = 3


def number(row: dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(name, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def find_liftoff(rows: list[dict[str, str]]) -> int | None:
    consecutive = 0
    for index, row in enumerate(rows):
        valid = int(number(row, "altitude_imu_estimator_valid")) == 1
        ascending = number(row, "imu_vertical_speed_mps") >= LIFTOFF_SPEED_MPS
        consecutive = consecutive + 1 if valid and ascending else 0
        if consecutive >= LIFTOFF_CONSECUTIVE_ROWS:
            return index - LIFTOFF_CONSECUTIVE_ROWS + 1
    return None


def find_deploy(rows: list[dict[str, str]], start: int) -> int:
    for index in range(start, len(rows)):
        if int(number(rows[index], "parachute_deployed")) == 1:
            return index
    return len(rows) - 1


def flight_phase(row: dict[str, str], ignition_first_s: float, ignition_last_s: float) -> str:
    time_s = number(row, "time_s")
    if time_s < ignition_first_s:
        return "stage1_ascent"
    if time_s <= ignition_last_s:
        return "stage2_ignition"
    return "coast_to_deploy"


def candidate_samples(
    rows: list[dict[str, str]], start: int, end: int
) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    previous_sample: dict[str, str] | None = None
    previous_timestamp: int | None = None

    # Keep one pressure sample from before liftoff so the first in-flight delta is valid.
    for index, row in enumerate(rows[: end + 1]):
        timestamp = int(number(row, "barometer_sample_timestamp_ms", -1.0))
        if timestamp < 0 or timestamp == previous_timestamp:
            continue
        if previous_sample is not None:
            if start <= index <= end:
                dt_ms = timestamp - int(
                    number(previous_sample, "barometer_sample_timestamp_ms", -1.0)
                )
                usable = (
                    int(number(row, "barometer_valid")) == 1
                    and number(row, "barometer_sample_age_ms", 999999.0)
                    <= MAX_SAMPLE_AGE_MS
                    and MIN_DT_MS <= dt_ms <= MAX_DT_MS
                    and int(number(row, "altitude_imu_estimator_valid")) == 1
                )
                if usable:
                    dt_s = dt_ms / 1000.0
                    raw_baro_speed = (
                        number(row, "relative_altitude_m")
                        - number(previous_sample, "relative_altitude_m")
                    ) / dt_s
                    imu_speed = number(row, "imu_vertical_speed_mps")
                    mismatch = abs(raw_baro_speed - imu_speed)
                    samples.append(
                        {
                            "row": row,
                            "raw_baro_speed_mps": raw_baro_speed,
                            "imu_vertical_speed_mps": imu_speed,
                            "mismatch_mps": mismatch,
                            "exceeds_limit": mismatch > MISMATCH_LIMIT_MPS,
                        }
                    )
        previous_sample = row
        previous_timestamp = timestamp
    return samples


def analyze(path: Path) -> tuple[dict[str, object], list[dict[str, object]]]:
    rows = load_rows(path)
    start = find_liftoff(rows)
    if start is None:
        raise ValueError(f"Could not identify liftoff in {path}")
    end = find_deploy(rows, start)
    ignition_rows = [row for row in rows[start : end + 1] if int(number(row, "ignition_active"))]
    ignition_first_s = (
        number(ignition_rows[0], "time_s") if ignition_rows else float("inf")
    )
    ignition_last_s = (
        number(ignition_rows[-1], "time_s") if ignition_rows else float("-inf")
    )
    samples = candidate_samples(rows, start, end)

    event_rows: list[dict[str, object]] = []
    run_count = 0
    in_run = False
    phase_counts = {
        "stage1_ascent": 0,
        "stage2_ignition": 0,
        "coast_to_deploy": 0,
    }
    prediction_window_over_limit = 0
    prediction_window_max_mismatch = 0.0
    for sample in samples:
        exceeded = bool(sample["exceeds_limit"])
        if exceeded and not in_run:
            run_count += 1
        in_run = exceeded
        if not exceeded:
            continue
        row = sample["row"]
        assert isinstance(row, dict)
        phase = flight_phase(row, ignition_first_s, ignition_last_s)
        phase_counts[phase] += 1
        if abs(number(row, "predicted_apogee_m")) > 0.0001:
            prediction_window_over_limit += 1
            prediction_window_max_mismatch = max(
                prediction_window_max_mismatch, float(sample["mismatch_mps"])
            )
        event_rows.append(
            {
                "log": path.parent.name,
                "repository_path": path.relative_to(REPO_ROOT).as_posix(),
                "time_s": f'{number(row, "time_s"):.3f}',
                "phase": phase,
                "relative_altitude_m": f'{number(row, "relative_altitude_m"):.3f}',
                "raw_baro_speed_mps": f'{float(sample["raw_baro_speed_mps"]):.3f}',
                "imu_vertical_speed_mps": f'{float(sample["imu_vertical_speed_mps"]):.3f}',
                "mismatch_mps": f'{float(sample["mismatch_mps"]):.3f}',
                "pressure_pa": f'{number(row, "pressure_pa"):.3f}',
                "barometer_sample_age_ms": int(number(row, "barometer_sample_age_ms")),
            }
        )

    mismatches = [float(sample["mismatch_mps"]) for sample in samples]
    exceeded = [sample for sample in samples if bool(sample["exceeds_limit"])]
    summary = {
        "log": path.parent.name,
        "repository_path": path.relative_to(REPO_ROOT).as_posix(),
        "liftoff_s": f'{number(rows[start], "time_s"):.3f}',
        "stage2_ignition_first_s": (
            f"{ignition_first_s:.3f}" if ignition_rows else ""
        ),
        "deploy_s": f'{number(rows[end], "time_s"):.3f}',
        "usable_baro_samples": len(samples),
        "samples_over_20_mps": len(exceeded),
        "percent_over_20_mps": (
            f"{100.0 * len(exceeded) / len(samples):.3f}" if samples else ""
        ),
        "over_limit_runs": run_count,
        "max_mismatch_mps": f"{max(mismatches):.3f}" if mismatches else "",
        "first_over_limit_s": event_rows[0]["time_s"] if event_rows else "",
        "last_over_limit_s": event_rows[-1]["time_s"] if event_rows else "",
        "stage1_over_limit_samples": phase_counts["stage1_ascent"],
        "stage2_over_limit_samples": phase_counts["stage2_ignition"],
        "coast_over_limit_samples": phase_counts["coast_to_deploy"],
        "prediction_nonzero_over_limit_samples": prediction_window_over_limit,
        "prediction_nonzero_max_mismatch_mps": (
            f"{prediction_window_max_mismatch:.3f}"
            if prediction_window_over_limit
            else ""
        ),
    }
    return summary, event_rows


def selected_logs() -> list[Path]:
    summary_path = ANALYSIS_ROOT / "altitude_prediction_dropout_summary.csv"
    with summary_path.open(encoding="utf-8-sig", newline="") as stream:
        selected = {
            row["log"]
            for row in csv.DictReader(stream)
            if row["generation"] == "legacy_202608"
            and row["prediction_first_s"]
            and int(row["internal_zero_runs"]) == 0
        }
    return [
        LEGACY_ROOT / name / "flight_log.csv"
        for name in sorted(selected)
        if (LEGACY_ROOT / name / "flight_log.csv").is_file()
    ]


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    summaries: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    for path in selected_logs():
        summary, log_events = analyze(path)
        summaries.append(summary)
        events.extend(log_events)

    write_csv(
        SUMMARY_PATH,
        summaries,
        [
            "log",
            "repository_path",
            "liftoff_s",
            "stage2_ignition_first_s",
            "deploy_s",
            "usable_baro_samples",
            "samples_over_20_mps",
            "percent_over_20_mps",
            "over_limit_runs",
            "max_mismatch_mps",
            "first_over_limit_s",
            "last_over_limit_s",
            "stage1_over_limit_samples",
            "stage2_over_limit_samples",
            "coast_over_limit_samples",
            "prediction_nonzero_over_limit_samples",
            "prediction_nonzero_max_mismatch_mps",
        ],
    )
    write_csv(
        EVENTS_PATH,
        events,
        [
            "log",
            "repository_path",
            "time_s",
            "phase",
            "relative_altitude_m",
            "raw_baro_speed_mps",
            "imu_vertical_speed_mps",
            "mismatch_mps",
            "pressure_pa",
            "barometer_sample_age_ms",
        ],
    )
    print(f"Analyzed {len(summaries)} logs and found {len(events)} over-limit samples")


if __name__ == "__main__":
    main()
