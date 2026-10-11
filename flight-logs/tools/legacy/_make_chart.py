import csv
from PIL import Image, ImageDraw, ImageFont

CSV = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run5_export\flight_log.csv"
OUT = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run5_export\flight_summary.png"

rows = list(csv.DictReader(open(CSV, encoding="utf-8-sig")))

def f(r, k):
    v = r.get(k, "") or ""
    try:
        return float(v)
    except ValueError:
        return 0.0

t = [f(r, "time_s") for r in rows]
alt = [f(r, "relative_altitude_m") for r in rows]
vspd = [f(r, "vertical_speed_mps") for r in rows]
pres = [f(r, "pressure_pa") for r in rows]
defl = [f(r, "suggested_deflection_deg") for r in rows]
az = [f(r, "accel_z_mg") for r in rows]
ctl = [1 if f(r, "altitude_control_allowed") > 0 else 0 for r in rows]
ign = [1 if f(r, "ignition_active") > 0 else 0 for r in rows]
dep = [1 if f(r, "parachute_deployed") > 0 else 0 for r in rows]
state = [r.get("rocket_state_name", "") for r in rows]

W, H = 1500, 1200
PADL, PADR, PADT, PADB = 90, 30, 30, 40
N = 5
ph = (H - PADT - PADB) / N
plot_w = W - PADL - PADR

img = Image.new("RGB", (W, H), "#ffffff")
d = ImageDraw.Draw(img)

def try_font(sz):
    for p in ["arial.ttf", "C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/consola.ttf"]:
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            pass
    return ImageFont.load_default()

f_title = try_font(26)
f_lab = try_font(18)
f_small = try_font(15)

def xp(v):
    tmin, tmax = t[0], t[-1]
    return PADL + (v - tmin) / (tmax - tmin) * plot_w

def yp(v, vmin, vmax, panel_top):
    if vmax == vmin:
        vmax = vmin + 1
    return panel_top + ph - (v - vmin) / (vmax - vmin) * (ph - 30)

def panel_bounds(i):
    return PADT + i * ph

# event shading regions across all panels
def shade_region(label_x0, label_x1, color, text):
    x0, x1 = xp(label_x0), xp(label_x1)
    for i in range(N):
        top = panel_bounds(i)
        d.rectangle([x0, top + 2, x1, top + ph - 2], fill=color)
    d.text((x0 + 4, 8), text, fill="#444444", font=f_small)

# find event windows
def first_last(pred):
    idx = [i for i, v in enumerate(pred) if v]
    if not idx:
        return None
    return t[idx[0]], t[idx[-1]]

ign_w = first_last(ign)
dep_w = first_last(dep)
if ign_w:
    shade_region(ign_w[0], ign_w[1], "#fee2e2", "IGNITION")
if dep_w:
    shade_region(dep_w[0], dep_w[1], "#dcfce7", "DEPLOYED")

def draw_panel(i, title, values, color, unit):
    top = panel_bounds(i)
    d.text((10, top + ph/2 - 10), title, fill="#111111", font=f_title)
    vmin, vmax = min(values), max(values)
    d.line([PADL, top + ph - 20, W - PADR, top + ph - 20], fill="#cccccc", width=1)
    # grid + y labels
    for g in range(5):
        v = vmin + (vmax - vmin) * g / 4
        gy = yp(v, vmin, vmax, top)
        d.line([PADL, gy, W - PADR, gy], fill="#f0f0f0", width=1)
        d.text((PADL - 70, gy - 8), f"{v:.1f}", fill="#555555", font=f_small)
    pts = [(xp(t[i]), yp(values[i], vmin, vmax, top)) for i in range(len(t))]
    if len(pts) > 1:
        d.line(pts, fill=color, width=2)
    d.text((W - PADR - 120, top + 8), unit, fill="#555555", font=f_small)

draw_panel(0, "Altitude (m)", alt, "#2563eb", "m")
draw_panel(1, "Vertical speed (m/s)", vspd, "#16a34a", "m/s")
draw_panel(2, "Pressure (Pa)", pres, "#9333ea", "Pa")
draw_panel(3, "Z accel (mg)", az, "#ea580c", "mg")
draw_panel(4, "Fin deflection (deg)", defl, "#dc2626", "deg")

# x axis time labels
for i in range(N):
    top = panel_bounds(i)
    for g in range(11):
        v = t[0] + (t[-1] - t[0]) * g / 10
        d.text((xp(v) - 14, top + ph - 18), f"{v:.0f}", fill="#888888", font=f_small)

# title
d.text((W/2 - 260, 2), "Flight log summary  (RLOG v9, 794 rec)", fill="#111111", font=f_title)

img.save(OUT)
print("saved", OUT)
