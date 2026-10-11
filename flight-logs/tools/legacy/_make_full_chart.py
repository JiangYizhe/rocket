import csv
from PIL import Image, ImageDraw, ImageFont

CSV = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run6_export\flight_log.csv"
OUT = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run6_export\flight_log_charts_0-16s.png"

rows = list(csv.DictReader(open(CSV, encoding="utf-8-sig")))

def f(r, k):
    try:
        return float(r.get(k, "") or 0)
    except ValueError:
        return 0.0

t = [f(r, "time_s") for r in rows]

STATE_NAMES = {"init":0, "wait_pid":1, "pid_control":2, "ignition":3, "flight":4, "deployed":5}
state = [STATE_NAMES.get(r.get("rocket_state_name",""), 0) for r in rows]

# 面板定义: (标题, [(字段, 标签, 颜色), ...], y最小值, y最大值 或 None 自动)
PANELS = [
    ("Attitude angle (deg)", [
        ("roll_deg", "roll", "#ef4444"),
        ("pitch_deg", "pitch", "#22c55e"),
        ("yaw_deg", "yaw", "#3b82f6"),
    ], None),
    ("Angular rate (dps)", [
        ("roll_rate_dps", "roll_rate", "#ef4444"),
        ("pitch_rate_dps", "pitch_rate", "#22c55e"),
        ("yaw_rate_dps", "yaw_rate", "#3b82f6"),
    ], None),
    ("Acceleration (mg)", [
        ("accel_x_mg", "X", "#ef4444"),
        ("accel_y_mg", "Y", "#22c55e"),
        ("accel_z_mg", "Z", "#3b82f6"),
    ], None),
    ("Altitude vs predicted apogee (m)", [
        ("relative_altitude_m", "altitude", "#0ea5e9"),
        ("predicted_apogee_m", "predicted apogee", "#f43f5e"),
    ], None),
    ("Vertical speed (m/s)", [
        ("vertical_speed_mps", "fused", "#6366f1"),
        ("barometer_vertical_speed_mps", "baro", "#0ea5e9"),
        ("imu_vertical_speed_mps", "imu", "#f97316"),
    ], None),
    ("Pressure (Pa)", [
        ("pressure_pa", "BMP280", "#8b5cf6"),
    ], None),
    ("Suggested fin deflection (deg)", [
        ("suggested_deflection_deg", "suggested", "#dc2626"),
        ("height_priority_applied_deflection_deg", "applied", "#2563eb"),
    ], None),
    ("Attitude PID output (us)", [
        ("roll_pid_output_us", "roll PID", "#ef4444"),
        ("pitch_pid_output_us", "pitch PID", "#22c55e"),
    ], None),
    ("Servo PWM (us, center 1500)", [
        ("roll_servo_pwm_us", "roll PWM", "#ef4444"),
        ("pitch_servo_pwm_us", "pitch PWM", "#22c55e"),
    ], None),
    ("Events / sensors", [
        ("ignition_active", "ignition", "#dc2626"),
        ("parachute_deployed", "deployed", "#2563eb"),
        ("imu_valid", "IMU valid", "#16a34a"),
        ("barometer_valid", "BMP valid", "#9333ea"),
        ("altitude_zero_ready", "zero ready", "#ea580c"),
    ], (0, 1.15)),
    ("Rocket state", [], (0, 5.4)),
    ("Temperature (C)", [
        ("temperature_c", "BMP280", "#f97316"),
    ], None),
]

W = 1400
PH = 190          # 每面板高度
PADL, PADR, PADT = 85, 30, 26
plot_w = W - PADL - PADR
H = PADT + len(PANELS) * PH + 8

img = Image.new("RGB", (W, H), "#ffffff")
d = ImageDraw.Draw(img)

def tf(sz):
    for p in ["C:/Windows/Fonts/arial.ttf", "arial.ttf"]:
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            pass
    return ImageFont.load_default()

F = tf(19); Fl = tf(14); Fs = tf(12)

def xp(v):
    return PADL + (v - t[0]) / (t[-1] - t[0]) * plot_w

def yp(v, vmin, vmax, top):
    if vmax == vmin:
        vmax = vmin + 1
    return top + PH - 22 - (v - vmin) / (vmax - vmin) * (PH - 46)

# 点火窗口背景
ign_idx = [i for i, v in enumerate(rows) if f(v, "ignition_active") > 0]
if ign_idx:
    x0, x1 = xp(t[ign_idx[0]]), xp(t[ign_idx[-1]])
    for pi in range(len(PANELS)):
        top = PADT + pi * PH
        d.rectangle([x0, top + 2, x1, top + PH - 2], fill="#fee2e2")
    d.text((x0 + 4, 6), "IGNITION", fill="#991b1b", font=Fs)

# 画所有面板
for pi, (title, series, yrange) in enumerate(PANELS):
    top = PADT + pi * PH
    d.text((10, top + 6), title, fill="#111111", font=F)

    # 图例
    lx = W - PADR - 10
    for (fld, label, color) in series:
        lx -= 8 + len(label) * 8 + 18
        if lx < PADL:
            break
        d.line([lx, top + 14, lx + 14, top + 14], fill=color, width=2)
        d.text((lx + 17, top + 8), label, fill="#444444", font=Fs)

    # 数据范围
    if yrange is not None:
        vmin, vmax = yrange
    else:
        vals = []
        for (fld, _, _) in series:
            vals += [f(r, fld) for r in rows]
        vmin, vmax = min(vals), max(vals)

    # 网格 + Y 标签
    for g in range(5):
        v = vmin + (vmax - vmin) * g / 4
        gy = yp(v, vmin, vmax, top)
        d.line([PADL, gy, W - PADR, gy], fill="#f0f0f0", width=1)
        d.text((PADL - 66, gy - 7), f"{v:.1f}", fill="#777777", font=Fs)

    # 曲线
    if title == "Rocket state":
        # 状态阶跃
        pts = []
        for i in range(len(t)):
            pts.append((xp(t[i]), yp(state[i], vmin, vmax, top)))
            if i + 1 < len(t) and state[i + 1] != state[i]:
                pts.append((xp(t[i + 1]), yp(state[i], vmin, vmax, top)))
        d.line(pts, fill="#0f172a", width=2)
        # 状态名标注
        for name, num in STATE_NAMES.items():
            yy = yp(num, vmin, vmax, top)
            d.text((PADL - 62, yy - 6), name, fill="#555555", font=Fs)
    else:
        for (fld, label, color) in series:
            vals = [f(r, fld) for r in rows]
            pts = [(xp(t[i]), yp(vals[i], vmin, vmax, top)) for i in range(len(t))]
            d.line(pts, fill=color, width=2)

    # X 轴时间标签（每 2s）
    for g in range(0, int(t[-1]) + 2, 2):
        d.text((xp(g) - 10, top + PH - 16), f"{g}s", fill="#888888", font=Fs)

# 顶部标题
d.text((W/2 - 200, 4), "Flight log run6  (0-16s, RLOG v9, 794 rec)", fill="#111111", font=tf(20))

img.save(OUT)
print("saved", OUT, img.size)
