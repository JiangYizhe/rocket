from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CAPTURE = ROOT / "serial_capture.txt"
TOOL = Path(
    r"C:\Users\91321\Desktop\火箭飞控pcb\基站\蓝牙\火箭-01_RX中断备份_20261003\火箭-01\Tools\flight_log_plot.py"
)
USER_STAGE2_DELAY_MS = 1120


def load_tool():
    spec = importlib.util.spec_from_file_location("flight_log_plot", TOOL)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load parser: {TOOL}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def number(row: dict[str, object], field: str) -> float | None:
    value = row.get(field)
    if value is None or value == "":
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def first_time(rows: list[dict[str, object]], field: str) -> float | None:
    for row in rows:
        value = number(row, field)
        if value is not None and value != 0.0:
            return float(row["time_s"])
    return None


def peak_row(rows: list[dict[str, object]], field: str) -> dict[str, object] | None:
    valid = [row for row in rows if number(row, field) is not None]
    return max(valid, key=lambda row: float(row[field])) if valid else None


def main() -> None:
    tool = load_tool()
    header, records = tool.read_log(CAPTURE)
    if not records:
        raise RuntimeError("No valid flight records were decoded")

    start = float(records[0]["time_s"])
    rows: list[dict[str, object]] = []
    for source in records:
        row = dict(source)
        row["time_s"] = round(float(source["time_s"]) - start, 6)
        if 0.0 <= float(row["time_s"]) <= 16.0:
            rows.append(row)
    if not rows:
        raise RuntimeError("No records remain in the 0-16 second window")

    csv_path = ROOT / "flight_log_0-16s.csv"
    svg_path = ROOT / "flight_log_charts_0-16s.svg"
    summary_path = ROOT / "flight_log_summary.json"
    tool.write_csv(rows, csv_path)
    tool.write_svg(rows, header, svg_path)

    peak = peak_row(rows, "relative_altitude_m")
    configured_delay = number(rows[0], "config_ignition_delay_ms")
    summary = {
        "decoded_records": len(records),
        "chart_records": len(rows),
        "chart_start_s": float(rows[0]["time_s"]),
        "chart_end_s": float(rows[-1]["time_s"]),
        "user_stage2_delay_ms": USER_STAGE2_DELAY_MS,
        "logged_stage2_delay_ms": configured_delay,
        "stage2_delay_matches_user_value": configured_delay == USER_STAGE2_DELAY_MS,
        "ignition_active_first_s": first_time(rows, "ignition_active"),
        "parachute_deployed_first_s": first_time(rows, "parachute_deployed"),
        "flight_control_first_s": first_time(rows, "flight_control_running"),
        "altitude_control_allowed_first_s": first_time(rows, "altitude_control_allowed"),
        "altitude_mix_enabled_first_s": first_time(rows, "altitude_mix_enabled"),
        "height_priority_enabled_first_s": first_time(rows, "height_priority_enabled"),
        "peak_relative_altitude_m": number(peak, "relative_altitude_m") if peak else None,
        "peak_relative_altitude_time_s": float(peak["time_s"]) if peak else None,
        "configured_servo_limit_deg": number(rows[0], "config_servo_limit_deg"),
        "last_rocket_state": rows[-1].get("rocket_state_name"),
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
