import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASES = [
    ("20261002_good_78.72m", "flight_log_20261002_com9_16s", 78.72),
    ("20261002_good_86.10m", "flight_log_20261002_launch2_com9", 86.10),
    ("20261003_good_77.85m", "flight_log_20261003_com9", 77.85),
    ("20261011_board1", "flight_log_20261011_board1_550ms_com9", None),
    ("20261011_board2", "flight_log_20261011_board2_550ms_com9", None),
    ("20261011_board3", "flight_log_20261011_board3_com9", None),
    ("20261011_board4", "flight_log_20261011_board4_com9", None),
]


def num(row, key):
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def norm(row, fields):
    values = [num(row, key) for key in fields]
    return math.sqrt(sum(v * v for v in values)) if all(v is not None for v in values) else None


def first(rows, key):
    return next((r for r in rows if num(r, key) not in (None, 0)), None)


def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def main():
    output = []
    for label, folder, external_height in CASES:
        directory = ROOT / "archive" / folder
        path = next(p for p in [directory / "flight_log_0-16s.csv", directory / "flight_log_0_16s.csv", directory / "flight_log.csv"] if p.exists())
        with path.open(encoding="utf-8-sig", newline="") as f:
            rows = list(csv.DictReader(f))
        start = num(rows[0], "time_s")
        for row in rows:
            row["time_s"] = num(row, "time_s") - start
        ignition = first(rows, "ignition_active")
        control = first(rows, "flight_control_running")
        deploy = first(rows, "parachute_deployed")
        t = ignition["time_s"]
        window = [r for r in rows if t <= r["time_s"] < t + 0.8 - 1e-8]
        acceleration = [norm(r, ["accel_x_mg", "accel_y_mg", "accel_z_mg"]) / 1000 for r in window if num(r, "imu_valid") == 1]
        rates = [norm(r, ["roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps"]) for r in window]
        peak = max(rows, key=lambda r: num(r, "relative_altitude_m") or -math.inf)
        after = [r for r in rows if 2.5 <= r["time_s"] < deploy["time_s"]]
        result = {
            "case": label, "source": path.relative_to(ROOT).as_posix(),
            "records": len(rows), "external_peak_m": external_height,
            "control_start_s": control["time_s"] if control else None,
            "ignition_s": t, "ignition_height_m": num(ignition, "relative_altitude_m"),
            "ignition_fused_speed_mps": num(ignition, "vertical_speed_mps"),
            "ignition_roll_deg": num(ignition, "roll_deg"),
            "ignition_pitch_deg": num(ignition, "pitch_deg"),
            "window_max_abs_roll_deg": max(abs(num(r, "roll_deg")) for r in window),
            "window_max_abs_pitch_deg": max(abs(num(r, "pitch_deg")) for r in window),
            "window_mean_rate_dps": mean(rates), "window_max_rate_dps": max(rates),
            "window_max_abs_z_rate_dps": max(abs(num(r, "yaw_rate_dps")) for r in window),
            "window_mean_specific_accel_g": mean(acceleration),
            "window_max_specific_accel_g": max(acceleration),
            "window_samples_above_2g": sum(v > 2 for v in acceleration),
            "window_fused_speed_change_mps": num(window[-1], "vertical_speed_mps") - num(ignition, "vertical_speed_mps"),
            "deploy_s": deploy["time_s"], "deploy_height_m": num(deploy, "relative_altitude_m"),
            "raw_baro_peak_m": num(peak, "relative_altitude_m"), "raw_peak_time_s": peak["time_s"],
            "baro_peak_after_2_5s_before_deploy_m": max((num(r, "relative_altitude_m") for r in after), default=None),
            "invalid_imu_records": sum(num(r, "imu_valid") != 1 for r in rows),
            "invalid_baro_records": sum(num(r, "barometer_valid") != 1 for r in rows),
            "max_imu_bus_errors": max(num(r, "imu_total_bus_errors") or 0 for r in rows),
            "fifo_reset_initial": num(rows[0], "imu_fifo_reset_count"),
            "fifo_reset_final": num(rows[-1], "imu_fifo_reset_count"),
        }
        output.append({k: round(v, 4) if isinstance(v, float) else v for k, v in result.items()})
    target = ROOT / "analysis" / "flight_comparison_20261011.csv"
    with target.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(output[0]))
        writer.writeheader()
        writer.writerows(output)
    print(json.dumps(output, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
