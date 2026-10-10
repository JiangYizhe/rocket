from __future__ import annotations

import csv
import html
import math
from pathlib import Path


ROOT = Path(r"C:\Users\91321\Desktop\火箭飞控pcb")
OUTPUT = ROOT / "flight_log_20261003_com9"
LOGS = [
    ("09-20", ROOT / "flight_log_20260920_com9_run2_full" / "flight_log.csv"),
    ("09-23", ROOT / "flight_log_20260923_com9_full" / "flight_log.csv"),
    ("09-27", ROOT / "COM9_capture_20260927" / "partial_export" / "flight_log.csv"),
    ("09-28", ROOT / "COM25_log_export_20260928_001527" / "COM25_log_export_20260928_001527" / "flight_log.csv"),
    ("10-02 第一次", ROOT / "flight_log_20261002_com9_16s" / "flight_log.csv"),
    ("10-02 第二次", ROOT / "flight_log_20261002_launch2_com9" / "flight_log.csv"),
    ("10-03 本次", ROOT / "flight_log_20261003_com9" / "flight_log.csv"),
]
AXES = (
    ("X", "roll_rate_dps", "#dc2626"),
    ("Y", "pitch_rate_dps", "#16a34a"),
    ("Z", "yaw_rate_dps", "#2563eb"),
)


def as_float(row, key):
    value = row.get(key, "")
    return float(value) if value not in (None, "") else math.nan


def load_result(label: str, path: Path):
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "yaw_rate_dps" not in rows[0]:
        return None
    event_index = next(
        (i for i, row in enumerate(rows) if row.get("parachute_deployed") == "1"),
        None,
    )
    if event_index is None:
        return None
    event = rows[event_index]
    event_time = as_float(event, "time_s")
    start_time = as_float(rows[0], "time_s")
    post = [
        row
        for row in rows[event_index:]
        if as_float(row, "time_s") <= event_time + 1.0 + 1e-6
    ]
    result = {
        "label": label,
        "path": str(path),
        "deploy_time_s": event_time - start_time,
        "deploy_altitude_m": as_float(event, "relative_altitude_m"),
        "max_altitude_m": max(as_float(row, "relative_altitude_m") for row in rows),
    }
    for axis, key, _ in AXES:
        value = as_float(event, key)
        peak_row = max(post, key=lambda row: abs(as_float(row, key)))
        result[f"{axis.lower()}_event_dps"] = value
        result[f"{axis.lower()}_post1_peak_dps"] = as_float(peak_row, key)
    result["event_vector_dps"] = math.sqrt(
        sum(result[f"{axis.lower()}_event_dps"] ** 2 for axis, _, _ in AXES)
    )
    return result


def write_results(results):
    path = OUTPUT / "deploy_xyz_comparison.csv"
    fields = [
        "label",
        "deploy_time_s",
        "deploy_altitude_m",
        "max_altitude_m",
        "x_event_dps",
        "y_event_dps",
        "z_event_dps",
        "event_vector_dps",
        "x_post1_peak_dps",
        "y_post1_peak_dps",
        "z_post1_peak_dps",
        "path",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)
    return path


def rounded_domain(values):
    limit = max(abs(value) for value in values)
    step = 10 ** max(0, math.floor(math.log10(max(limit, 1))) - 1)
    limit = math.ceil(limit * 1.12 / step) * step
    return -limit, limit


def make_svg(results, output):
    width, height = 1500, 1080
    left, right = 105, 35
    plot_width = width - left - right
    panels = [
        ("二级开伞瞬间三轴角速度", "event", 130, 330),
        ("二级开伞后1秒内峰值（保留方向）", "post1_peak", 610, 330),
    ]
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<g font-family="Microsoft YaHei, SimHei, Arial, sans-serif">',
        '<text x="38" y="45" font-size="25" font-weight="bold" fill="#0f172a">历次二级开伞 XYZ 轴角速度比较</text>',
        '<text x="38" y="76" font-size="14" fill="#475569">按 parachute_deployed 首次变为 1 对齐；X=Roll，Y=Pitch，Z=Yaw（IMU Z轴）</text>',
    ]

    group_width = plot_width / len(results)
    bar_width = min(34.0, group_width / 5.0)
    for title, suffix, top, panel_height in panels:
        values = [
            result[f"{axis.lower()}_{suffix}_dps"]
            for result in results
            for axis, _, _ in AXES
        ]
        y_min, y_max = rounded_domain(values)
        zero_y = top + panel_height - (0 - y_min) * panel_height / (y_max - y_min)
        lines.extend([
            f'<text x="{left}" y="{top - 25}" font-size="18" font-weight="bold" fill="#1e293b">{html.escape(title)}</text>',
            f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#ffffff" stroke="#cbd5e1"/>',
        ])
        for tick in range(5):
            value = y_min + tick * (y_max - y_min) / 4
            y = top + panel_height - tick * panel_height / 4
            lines.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#e2e8f0"/>')
            lines.append(f'<text x="{left - 12}" y="{y + 5:.2f}" text-anchor="end" font-size="12" fill="#475569">{value:.0f}</text>')
        lines.append(f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_width}" y2="{zero_y:.2f}" stroke="#64748b" stroke-width="1.5"/>')

        for index, result in enumerate(results):
            center = left + group_width * (index + 0.5)
            for axis_index, (axis, _, color) in enumerate(AXES):
                value = result[f"{axis.lower()}_{suffix}_dps"]
                x = center + (axis_index - 1) * (bar_width + 4) - bar_width / 2
                value_y = top + panel_height - (value - y_min) * panel_height / (y_max - y_min)
                rect_y = min(zero_y, value_y)
                rect_h = max(1.0, abs(value_y - zero_y))
                lines.append(f'<rect x="{x:.2f}" y="{rect_y:.2f}" width="{bar_width:.2f}" height="{rect_h:.2f}" fill="{color}"/>')
                label_y = value_y - 7 if value >= 0 else value_y + 16
                lines.append(f'<text x="{x + bar_width / 2:.2f}" y="{label_y:.2f}" text-anchor="middle" font-size="11" fill="#334155">{value:.1f}</text>')
            lines.append(f'<text x="{center:.2f}" y="{top + panel_height + 28}" text-anchor="middle" font-size="12" fill="#334155">{html.escape(result["label"])}</text>')
        lines.append(f'<text x="34" y="{top + panel_height / 2}" transform="rotate(-90 34 {top + panel_height / 2})" text-anchor="middle" font-size="13" fill="#475569">角速度 (°/s)</text>')

    legend_x = 990
    for index, (axis, _, color) in enumerate(AXES):
        x = legend_x + index * 130
        lines.append(f'<rect x="{x}" y="34" width="18" height="18" fill="{color}"/>')
        lines.append(f'<text x="{x + 27}" y="49" font-size="13" fill="#334155">{axis}轴</text>')
    lines.extend(["</g>", "</svg>"])
    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    results = [result for label, path in LOGS if (result := load_result(label, path))]
    csv_path = write_results(results)
    svg_path = OUTPUT / "deploy_xyz_comparison.svg"
    make_svg(results, svg_path)
    print(csv_path)
    print(svg_path)
    for result in results:
        print(
            result["label"],
            f"t={result['deploy_time_s']:.3f}",
            f"XYZ={result['x_event_dps']:.1f},{result['y_event_dps']:.1f},{result['z_event_dps']:.1f}",
            f"post1={result['x_post1_peak_dps']:.1f},{result['y_post1_peak_dps']:.1f},{result['z_post1_peak_dps']:.1f}",
            f"alt={result['deploy_altitude_m']:.1f}",
        )


if __name__ == "__main__":
    main()
