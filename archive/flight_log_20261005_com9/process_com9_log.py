from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CAPTURE = ROOT / "serial_capture.txt"
TOOL = Path(
    r"C:\Users\91321\Desktop\火箭飞控pcb\基站\蓝牙\火箭-01_RX中断备份_20261003\火箭-01\Tools\flight_log_plot.py"
)


def load_tool():
    spec = importlib.util.spec_from_file_location("flight_log_plot", TOOL)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load parser: {TOOL}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def first_time(rows, field: str):
    for row in rows:
        if int(row.get(field, 0)) != 0:
            return float(row["time_s"])
    return None


def main():
    tool = load_tool()
    header, records = tool.read_log(CAPTURE)
    if not records:
        raise RuntimeError("No valid flight records were decoded")

    start = float(records[0]["time_s"])
    rows = []
    for source in records:
        row = dict(source)
        row["time_s"] = round(float(source["time_s"]) - start, 6)
        if 0.0 <= row["time_s"] <= 16.0:
            rows.append(row)
    if not rows:
        raise RuntimeError("No records remain in the 0-16 second window")

    csv_path = ROOT / "flight_log_0_16s.csv"
    svg_path = ROOT / "flight_log_charts_0_16s.svg"
    summary_path = ROOT / "flight_log_summary.json"
    tool.write_csv(rows, csv_path)
    tool.write_svg(rows, header, svg_path)

    peak = max(rows, key=lambda row: float(row["relative_altitude_m"]))
    summary = {
        "decoded_records": len(records),
        "chart_records": len(rows),
        "chart_end_s": float(rows[-1]["time_s"]),
        "ignition_active_first_s": first_time(rows, "ignition_active"),
        "parachute_deployed_first_s": first_time(rows, "parachute_deployed"),
        "flight_control_first_s": first_time(rows, "flight_control_running"),
        "altitude_control_allowed_first_s": first_time(rows, "altitude_control_allowed"),
        "peak_relative_altitude_m": float(peak["relative_altitude_m"]),
        "peak_relative_altitude_time_s": float(peak["time_s"]),
        "configured_servo_limit_deg": float(rows[0]["config_servo_limit_deg"]),
    }
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
