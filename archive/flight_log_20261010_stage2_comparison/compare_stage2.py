from __future__ import annotations

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
RUNS = [
    ("Board 1", "#dc2626", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261010_board1_com9\flight_log_0-16s.csv"), False),
    ("Board 2", "#2563eb", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261010_board2_com9_152347\flight_log_0-16s.csv"), False),
    ("2026-10-02 86.28 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261002_launch2_com9\flight_log_0_16s.csv"), True),
    ("2026-10-02 78.22 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261002_com9_16s\flight_log_0_16s.csv"), True),
    ("2026-10-03 78.15 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261003_com9\flight_log_0_16s.csv"), True),
    ("2026-10-05 74.20 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261005_com9_run2\flight_log_0-16s.csv"), True),
    ("2026-09-23 73.86 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20260923_com9_0-16s\flight_log_0-16s.csv"), True),
    ("2026-10-05 73.12 m", "#94a3b8", Path(r"C:\Users\91321\Desktop\火箭飞控pcb\flight_log_20261005_com9\flight_log_0_16s.csv"), True),
]


def value(row: dict[str, str], field: str) -> float:
    try:
        return float(row.get(field, "") or "nan")
    except ValueError:
        return float("nan")


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    for source in values[1:]:
        current = source
        while current - result[-1] > 180.0:
            current -= 360.0
        while current - result[-1] < -180.0:
            current += 360.0
        result.append(current)
    return result


def load_run(spec):
    name, color, path, historical = spec
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    ignition = [row for row in rows if value(row, "ignition_active") != 0.0]
    ignition_time = value(ignition[0], "time_s")
    selected = [
        row for row in rows
        if -0.32 <= value(row, "time_s") - ignition_time <= 0.92
    ]
    times = [value(row, "time_s") - ignition_time for row in selected]
    yaw = unwrap([value(row, "yaw_deg") for row in selected])
    yaw_at_ignition = yaw[min(range(len(times)), key=lambda i: abs(times[i]))]
    data = {
        "roll_deg": [value(row, "roll_deg") for row in selected],
        "pitch_deg": [value(row, "pitch_deg") for row in selected],
        "yaw_delta_deg": [item - yaw_at_ignition for item in yaw],
        "roll_rate_dps": [value(row, "roll_rate_dps") for row in selected],
        "pitch_rate_dps": [value(row, "pitch_rate_dps") for row in selected],
        "yaw_rate_dps": [value(row, "yaw_rate_dps") for row in selected],
    }
    return {
        "name": name,
        "color": color,
        "historical": historical,
        "times": times,
        "data": data,
    }


def font(size: int, bold: bool = False):
    names = [
        "C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf",
    ]
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def main() -> None:
    ROOT.mkdir(parents=True, exist_ok=True)
    runs = [load_run(spec) for spec in RUNS]
    panels = [
        ("Roll angle", "roll_deg", "deg"),
        ("Pitch angle", "pitch_deg", "deg"),
        ("Yaw change from ignition", "yaw_delta_deg", "deg"),
        ("Roll rate", "roll_rate_dps", "deg/s"),
        ("Pitch rate", "pitch_rate_dps", "deg/s"),
        ("Yaw rate", "yaw_rate_dps", "deg/s"),
    ]
    width, panel_height = 1600, 260
    top, left, right, bottom = 150, 105, 40, 42
    height = top + len(panels) * panel_height + 20
    image = Image.new("RGB", (width, height), "#ffffff")
    draw = ImageDraw.Draw(image)
    title_font, label_font, small_font = font(26, True), font(17), font(14)
    draw.text((30, 18), "Stage-2 ignition attitude comparison", fill="#0f172a", font=title_font)
    draw.text((30, 54), "Historical reference: six flights reaching 73.12-86.28 m", fill="#475569", font=label_font)
    legend = [("Historical flights", "#94a3b8"), ("Board 1", "#dc2626"), ("Board 2", "#2563eb")]
    lx = 30
    for label, color in legend:
        draw.line((lx, 102, lx + 34, 102), fill=color, width=5 if label != "Historical flights" else 2)
        draw.text((lx + 42, 91), label, fill="#334155", font=small_font)
        lx += 210
    draw.text((760, 91), "t=0: stage-2 ignition; dashed line: attitude control starts near +0.20 s", fill="#475569", font=small_font)

    xmin, xmax = -0.3, 0.9
    plot_width = width - left - right
    for panel_index, (panel_title, field, unit) in enumerate(panels):
        panel_top = top + panel_index * panel_height
        plot_top = panel_top + 32
        plot_bottom = panel_top + panel_height - bottom
        values = [item for run in runs for item in run["data"][field] if math.isfinite(item)]
        ymin, ymax = min(values), max(values)
        padding = max((ymax - ymin) * 0.08, 2.0)
        ymin -= padding
        ymax += padding

        def xcoord(x):
            return left + (x - xmin) / (xmax - xmin) * plot_width

        def ycoord(y):
            return plot_bottom - (y - ymin) / (ymax - ymin) * (plot_bottom - plot_top)

        draw.text((18, panel_top + 4), f"{panel_title} ({unit})", fill="#0f172a", font=label_font)
        draw.rectangle((left, plot_top, width - right, plot_bottom), outline="#cbd5e1", width=1)
        for grid in range(5):
            yvalue = ymin + (ymax - ymin) * grid / 4
            y = ycoord(yvalue)
            draw.line((left, y, width - right, y), fill="#e2e8f0", width=1)
            draw.text((10, y - 8), f"{yvalue:.0f}", fill="#64748b", font=small_font)
        for tick in [-0.3, 0.0, 0.2, 0.4, 0.6, 0.8]:
            x = xcoord(tick)
            draw.line((x, plot_top, x, plot_bottom), fill="#e2e8f0", width=1)
            draw.text((x - 16, plot_bottom + 8), f"{tick:+.1f}", fill="#64748b", font=small_font)
        draw.line((xcoord(0.0), plot_top, xcoord(0.0), plot_bottom), fill="#dc2626", width=2)
        control_x = xcoord(0.2)
        for y in range(int(plot_top), int(plot_bottom), 12):
            draw.line((control_x, y, control_x, min(y + 6, plot_bottom)), fill="#0f172a", width=1)

        for run in sorted(runs, key=lambda item: not item["historical"]):
            points = [
                (xcoord(t), ycoord(y))
                for t, y in zip(run["times"], run["data"][field])
                if xmin <= t <= xmax and math.isfinite(y)
            ]
            if len(points) >= 2:
                draw.line(points, fill=run["color"], width=2 if run["historical"] else 4)

    output = ROOT / "stage2_attitude_comparison.png"
    image.save(output)
    print(output)


if __name__ == "__main__":
    main()
