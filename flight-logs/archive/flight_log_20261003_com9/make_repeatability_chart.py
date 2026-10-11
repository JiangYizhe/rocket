from __future__ import annotations

import csv
import html
import math
from pathlib import Path


OUT = Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261003_com9")
LOGS = [
    (
        "10-02 基准",
        Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261002_com9_16s\flight_log.csv"),
        "#2563eb",
    ),
    (
        "10-03 重复",
        OUT / "flight_log.csv",
        "#dc2626",
    ),
]
PANELS = [
    ("相对高度", "relative_altitude_m", "m"),
    ("垂直速度", "vertical_speed_mps", "m/s"),
    ("IMU Z轴加速度", "accel_z_mg", "mg"),
    ("高度控制实际舵角", "applied_altitude_deflection_deg", "°"),
]


def load(label, path, color):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    ignition = next(float(row["time_s"]) for row in rows if row["ignition_active"] == "1")
    converted = []
    for row in rows:
        t = float(row["time_s"]) - ignition
        if -0.4 <= t <= 6.5:
            converted.append(
                {
                    "t": t,
                    **{key: float(row[key]) for _, key, _ in PANELS},
                }
            )
    return {"label": label, "color": color, "rows": converted}


def extent(values, include_zero=False):
    lo, hi = min(values), max(values)
    if include_zero:
        lo, hi = min(lo, 0.0), max(hi, 0.0)
    if math.isclose(lo, hi):
        return lo - 1.0, hi + 1.0
    pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def points(rows, key, left, top, width, height, x_min, x_max, y_min, y_max):
    result = []
    for row in rows:
        x = left + (row["t"] - x_min) * width / (x_max - x_min)
        y = top + height - (row[key] - y_min) * height / (y_max - y_min)
        result.append(f"{x:.2f},{y:.2f}")
    return " ".join(result)


def main():
    series = [load(*item) for item in LOGS]
    width, height = 1500, 1640
    left, right = 105, 35
    plot_width = width - left - right
    panel_height = 280
    panel_gap = 100
    x_min, x_max = -0.4, 6.5
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<g font-family="Microsoft YaHei, SimHei, Arial, sans-serif">',
        '<text x="38" y="45" font-size="25" font-weight="bold" fill="#0f172a">1.2 s + 117 g 重复性对比</text>',
        '<text x="38" y="76" font-size="14" fill="#475569">两次日志按二级点火开始对齐；虚线为二级点火时刻</text>',
    ]
    legend_x = 1080
    for index, item in enumerate(series):
        x = legend_x + index * 175
        lines.append(f'<line x1="{x}" y1="45" x2="{x + 30}" y2="45" stroke="{item["color"]}" stroke-width="4"/>')
        lines.append(f'<text x="{x + 39}" y="50" font-size="13" fill="#334155">{html.escape(item["label"])}</text>')

    for panel_index, (title, key, unit) in enumerate(PANELS):
        top = 125 + panel_index * (panel_height + panel_gap)
        values = [row[key] for item in series for row in item["rows"]]
        y_min, y_max = extent(values, include_zero=True)
        lines.extend([
            f'<text x="{left}" y="{top - 24}" font-size="18" font-weight="bold" fill="#1e293b">{html.escape(title)}</text>',
            f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#ffffff" stroke="#cbd5e1"/>',
        ])
        for tick in range(5):
            value = y_min + tick * (y_max - y_min) / 4
            y = top + panel_height - tick * panel_height / 4
            lines.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="#e2e8f0"/>')
            lines.append(f'<text x="{left - 12}" y="{y + 5:.2f}" text-anchor="end" font-size="12" fill="#475569">{value:.1f}</text>')
        for tick in range(8):
            value = x_min + tick * (x_max - x_min) / 7
            x = left + tick * plot_width / 7
            lines.append(f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + panel_height}" stroke="#e2e8f0"/>')
            lines.append(f'<text x="{x:.2f}" y="{top + panel_height + 25}" text-anchor="middle" font-size="12" fill="#475569">{value:.1f}</text>')
        ignition_x = left + (0.0 - x_min) * plot_width / (x_max - x_min)
        lines.append(f'<line x1="{ignition_x:.2f}" y1="{top}" x2="{ignition_x:.2f}" y2="{top + panel_height}" stroke="#64748b" stroke-width="1.5" stroke-dasharray="7 6"/>')
        for item in series:
            pts = points(item["rows"], key, left, top, plot_width, panel_height, x_min, x_max, y_min, y_max)
            lines.append(f'<polyline points="{pts}" fill="none" stroke="{item["color"]}" stroke-width="2.4"/>')
        lines.append(f'<text x="34" y="{top + panel_height / 2}" transform="rotate(-90 34 {top + panel_height / 2})" text-anchor="middle" font-size="13" fill="#475569">{unit}</text>')
        lines.append(f'<text x="{left + plot_width / 2}" y="{top + panel_height + 50}" text-anchor="middle" font-size="13" fill="#475569">二级点火后时间 (s)</text>')
    lines.extend(["</g>", "</svg>"])
    path = OUT / "repeatability_1p2s_117g.svg"
    path.write_text("\n".join(lines), encoding="utf-8")
    print(path)


if __name__ == "__main__":
    main()
