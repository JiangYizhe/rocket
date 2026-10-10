from __future__ import annotations

import csv
import html
import importlib.util
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CAPTURE = ROOT / "serial_capture.txt"
TOOL = Path(
    r"C:\Users\91321\Desktop\火箭飞控pcb\基站\蓝牙\火箭-01\火箭-01\Tools\flight_log_plot.py"
)


def load_tool():
    spec = importlib.util.spec_from_file_location("flight_log_plot", TOOL)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {TOOL}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def normalize_records(records):
    start = float(records[0]["time_s"])
    normalized = []
    for source in records:
        record = dict(source)
        record["time_s"] = round(float(source["time_s"]) - start, 6)
        if record["time_s"] <= 16.0:
            normalized.append(record)
    return normalized


def read_csv(path: Path):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    numeric = (
        "time_s",
        "yaw_deg",
        "yaw_rate_dps",
        "roll_rate_dps",
        "pitch_rate_dps",
    )
    for row in rows:
        for key in numeric:
            row[key] = float(row[key])
    return rows


def nice_extent(values, padding=0.08):
    lo, hi = min(values), max(values)
    if math.isclose(lo, hi):
        return lo - 1.0, hi + 1.0
    extra = (hi - lo) * padding
    return lo - extra, hi + extra


def path_points(rows, key, x0, y0, width, height, x_min, x_max, y_min, y_max):
    points = []
    for row in rows:
        x = x0 + (row["time_s"] - x_min) * width / (x_max - x_min)
        y = y0 + height - (row[key] - y_min) * height / (y_max - y_min)
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def focused_svg(rows, output: Path):
    deploy_index = next(i for i, row in enumerate(rows) if row["parachute_deployed"] == "1")
    deploy_time = rows[deploy_index]["time_s"]
    window = [row for row in rows if deploy_time - 1.5 <= row["time_s"] <= deploy_time + 1.7]
    post_deploy = [row for row in window if row["time_s"] >= deploy_time]
    peak = max(post_deploy, key=lambda row: abs(row["yaw_rate_dps"]))

    width, height = 1400, 980
    x0, plot_width = 105, 1245
    x_min, x_max = window[0]["time_s"], window[-1]["time_s"]
    panels = [
        ("IMU Z轴角速度", "yaw_rate_dps", "°/s", 120, 300, "#2563eb"),
        ("IMU三轴角速度", None, "°/s", 500, 300, None),
    ]
    all_rates = [
        row[key]
        for row in window
        for key in ("roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps")
    ]
    y_ranges = {
        "yaw_rate_dps": nice_extent([row["yaw_rate_dps"] for row in window]),
        "all": nice_extent(all_rates),
    }

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<g font-family="Microsoft YaHei, SimHei, Arial, sans-serif">',
        '<text x="40" y="46" font-size="25" font-weight="bold" fill="#0f172a">二级开伞前后 IMU Z轴旋转</text>',
        f'<text x="40" y="76" font-size="14" fill="#475569">开伞时刻 {deploy_time:.3f} s（相对日志起点） · 开伞瞬间 {rows[deploy_index]["yaw_rate_dps"]:.1f}°/s · 后续峰值 {peak["yaw_rate_dps"]:.1f}°/s</text>',
    ]

    for title, key, unit, y0, panel_height, color in panels:
        range_key = key if key else "all"
        y_min, y_max = y_ranges[range_key]
        lines.extend([
            f'<text x="{x0}" y="{y0 - 25}" font-size="17" font-weight="bold" fill="#1e293b">{html.escape(title)}</text>',
            f'<rect x="{x0}" y="{y0}" width="{plot_width}" height="{panel_height}" fill="#ffffff" stroke="#cbd5e1"/>',
        ])
        for tick in range(6):
            y = y0 + tick * panel_height / 5
            value = y_max - tick * (y_max - y_min) / 5
            lines.append(f'<line x1="{x0}" y1="{y:.2f}" x2="{x0 + plot_width}" y2="{y:.2f}" stroke="#e2e8f0"/>')
            lines.append(f'<text x="{x0 - 12}" y="{y + 5:.2f}" text-anchor="end" font-size="12" fill="#475569">{value:.0f}</text>')
        for tick in range(7):
            t = x_min + tick * (x_max - x_min) / 6
            x = x0 + tick * plot_width / 6
            lines.append(f'<line x1="{x:.2f}" y1="{y0}" x2="{x:.2f}" y2="{y0 + panel_height}" stroke="#e2e8f0"/>')
            lines.append(f'<text x="{x:.2f}" y="{y0 + panel_height + 25}" text-anchor="middle" font-size="12" fill="#475569">{t:.2f}</text>')
        deploy_x = x0 + (deploy_time - x_min) * plot_width / (x_max - x_min)
        lines.append(f'<line x1="{deploy_x:.2f}" y1="{y0}" x2="{deploy_x:.2f}" y2="{y0 + panel_height}" stroke="#dc2626" stroke-width="2" stroke-dasharray="8 6"/>')
        lines.append(f'<text x="{deploy_x + 9:.2f}" y="{y0 + 22}" font-size="13" fill="#dc2626">二级开伞</text>')

        if key:
            pts = path_points(window, key, x0, y0, plot_width, panel_height, x_min, x_max, y_min, y_max)
            lines.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        else:
            series = (
                ("roll_rate_dps", "X / Roll", "#dc2626"),
                ("pitch_rate_dps", "Y / Pitch", "#16a34a"),
                ("yaw_rate_dps", "Z / Yaw", "#2563eb"),
            )
            legend_x = x0 + 770
            for offset, (series_key, label, series_color) in enumerate(series):
                pts = path_points(window, series_key, x0, y0, plot_width, panel_height, x_min, x_max, y_min, y_max)
                lines.append(f'<polyline points="{pts}" fill="none" stroke="{series_color}" stroke-width="2"/>')
                lx = legend_x + offset * 150
                lines.append(f'<line x1="{lx}" y1="{y0 - 30}" x2="{lx + 25}" y2="{y0 - 30}" stroke="{series_color}" stroke-width="3"/>')
                lines.append(f'<text x="{lx + 32}" y="{y0 - 25}" font-size="13" fill="#334155">{label}</text>')
        lines.append(f'<text x="35" y="{y0 + panel_height / 2}" transform="rotate(-90 35 {y0 + panel_height / 2})" text-anchor="middle" font-size="13" fill="#475569">{unit}</text>')
        lines.append(f'<text x="{x0 + plot_width / 2}" y="{y0 + panel_height + 49}" text-anchor="middle" font-size="13" fill="#475569">时间 (s)</text>')

    lines.extend([
        '<text x="105" y="914" font-size="14" fill="#334155">蓝线为 IMU Z 轴角速度；红色虚线为二级开伞状态首次变为 1 的时刻。</text>',
        '</g>',
        '</svg>',
    ])
    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    tool = load_tool()
    header, records = tool.read_log(CAPTURE)
    normalized = normalize_records(records)
    csv_path = ROOT / "flight_log_0_16s.csv"
    svg_path = ROOT / "flight_log_charts_0_16s.svg"
    tool.write_csv(normalized, csv_path)
    tool.write_svg(normalized, header, svg_path)
    rows = read_csv(csv_path)
    focused_svg(rows, ROOT / "stage2_deploy_imu_z.svg")
    print(csv_path)
    print(svg_path)
    print(ROOT / "stage2_deploy_imu_z.svg")


if __name__ == "__main__":
    main()
