from __future__ import annotations

import csv
import json
from pathlib import Path


ROOT = Path(r"C:\Users\91321\Desktop\火箭飞控pcb")


def number(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, "") or default)
    except ValueError:
        return default


def summarize(path: Path) -> dict[str, object] | None:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, UnicodeError, csv.Error):
        return None
    if len(rows) < 2 or "relative_altitude_m" not in rows[0]:
        return None

    start_s = number(rows[0], "time_s")
    ignition = [row for row in rows if number(row, "ignition_active") > 0.5]
    deploy = [row for row in rows if number(row, "parachute_deployed") > 0.5]
    apogee = max(rows, key=lambda row: number(row, "relative_altitude_m"))
    max_speed = max(rows, key=lambda row: number(row, "vertical_speed_mps"))

    ignition_start = ignition[0] if ignition else None
    ignition_end = ignition[-1] if ignition else None
    if ignition_start:
        ignition_time = number(ignition_start, "time_s")
        post = [
            row
            for row in rows
            if ignition_time <= number(row, "time_s") <= ignition_time + 1.5
        ]
        post_max = max(post, key=lambda row: number(row, "vertical_speed_mps"))
    else:
        post_max = None

    signature = (
        len(rows),
        rows[0].get("timestamp_ms", ""),
        rows[-1].get("timestamp_ms", ""),
        round(number(apogee, "relative_altitude_m"), 2),
        ignition_start.get("timestamp_ms", "") if ignition_start else "",
    )
    return {
        "signature": signature,
        "file": str(path),
        "rows": len(rows),
        "duration_s": round(number(rows[-1], "time_s") - start_s, 2),
        "apogee_m": round(number(apogee, "relative_altitude_m"), 2),
        "apogee_time_s": round(number(apogee, "time_s") - start_s, 2),
        "ignition_time_s": (
            round(number(ignition_start, "time_s") - start_s, 2)
            if ignition_start
            else None
        ),
        "ignition_altitude_m": (
            round(number(ignition_start, "relative_altitude_m"), 2)
            if ignition_start
            else None
        ),
        "ignition_speed_mps": (
            round(number(ignition_start, "vertical_speed_mps"), 2)
            if ignition_start
            else None
        ),
        "ignition_roll_deg": (
            round(number(ignition_start, "roll_deg"), 2) if ignition_start else None
        ),
        "ignition_pitch_deg": (
            round(number(ignition_start, "pitch_deg"), 2) if ignition_start else None
        ),
        "ignition_end_speed_mps": (
            round(number(ignition_end, "vertical_speed_mps"), 2)
            if ignition_end
            else None
        ),
        "post_ignition_max_speed_mps": (
            round(number(post_max, "vertical_speed_mps"), 2) if post_max else None
        ),
        "max_speed_mps": round(number(max_speed, "vertical_speed_mps"), 2),
        "deploy_time_s": (
            round(number(deploy[0], "time_s") - start_s, 2) if deploy else None
        ),
        "stage_mass_kg": rows[0].get("config_stage_mass_kg", ""),
        "ignition_delay_ms": rows[0].get("config_ignition_delay_ms", ""),
        "ignition_pulse_ms": rows[0].get("config_ignition_pulse_ms", ""),
        "target_altitude_m": rows[0].get("config_target_altitude_m", ""),
    }


def main() -> None:
    unique: dict[tuple[object, ...], dict[str, object]] = {}
    for path in ROOT.rglob("*.csv"):
        if "flight_log" not in path.name.lower():
            continue
        item = summarize(path)
        if item is not None:
            unique.setdefault(tuple(item.pop("signature")), item)

    selected = sorted(
        (item for item in unique.values() if float(item["apogee_m"]) >= 65.0),
        key=lambda item: float(item["apogee_m"]),
        reverse=True,
    )
    print(json.dumps(selected, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
