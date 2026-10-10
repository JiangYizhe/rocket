import csv
from PIL import Image, ImageDraw, ImageFont

CSV = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run2_export\flight_log.csv"
OUT = r"C:\Users\91321\Desktop\火箭飞控pcb\_log_com9_run2_export\fin_mix.png"

rows = list(csv.DictReader(open(CSV, encoding="utf-8-sig")))

def f(r, k):
    try:
        return float(r.get(k, "") or 0)
    except ValueError:
        return 0.0

t = [f(r, "time_s") for r in rows]
sug = [f(r, "suggested_deflection_deg") for r in rows]          # 高度控制器建议
app = [f(r, "height_priority_applied_deflection_deg") for r in rows]  # 实际施加
off = [f(r, "height_priority_applied_offset_us") for r in rows]  # 混入 offset(us)
rpwm = [f(r, "roll_servo_pwm_us") for r in rows]
ppwm = [f(r, "pitch_servo_pwm_us") for r in rows]
rpid = [f(r, "roll_pid_output_us") for r in rows]
ppid = [f(r, "pitch_pid_output_us") for r in rows]
ctl = [1 if f(r, "altitude_control_allowed") > 0 else 0 for r in rows]

W, H = 1500, 1100
PADL, PADR, PADT, PADB = 95, 30, 32, 42
N = 4
ph = (H - PADT - PADB) / N
plot_w = W - PADL - PADR

img = Image.new("RGB", (W, H), "#ffffff")
d = ImageDraw.Draw(img)

def tf(sz):
    for p in ["C:/Windows/Fonts/arial.ttf", "arial.ttf"]:
        try:
            return ImageFont.truetype(p, sz)
        except Exception:
            pass
    return ImageFont.load_default()

F = tf(26); Fl = tf(17); Fs = tf(15)

def xp(v):
    tmin, tmax = t[0], t[-1]
    return PADL + (v - tmin) / (tmax - tmin) * plot_w

def yp(v, vmin, vmax, top):
    if vmax == vmin:
        vmax = vmin + 1
    return top + ph - 8 - (v - vmin) / (vmax - vmin) * (ph - 34)

# control-active shaded region
idx = [i for i, v in enumerate(ctl) if v]
if idx:
    x0, x1 = xp(t[idx[0]]), xp(t[idx[-1]])
    for i in range(N):
        top = PADT + i * ph
        d.rectangle([x0, top + 2, x1, top + ph - 2], fill="#fef9c3")

def axis(top, vmin, vmax, unit):
    d.line([PADL, top + ph - 8, W - PADR, top + ph - 8], fill="#c0c0c0", width=1)
    for g in range(5):
        v = vmin + (vmax - vmin) * g / 4
        gy = yp(v, vmin, vmax, top)
        d.line([PADL, gy, W - PADR, gy], fill="#f0f0f0", width=1)
        d.text((PADL - 74, gy - 8), f"{v:.1f}", fill="#666666", font=Fs)

def line(vals, color, w=2):
    pts = [(xp(t[i]), yp(vals[i], min(vals), max(vals), PADT + 0*ph)) for i in range(len(t))]
    d.line(pts, fill=color, width=w)

# panel 0: suggested vs applied (deflection)
top0 = PADT + 0*ph
d.text((10, top0 + 8), "Altitude deflection: suggested vs applied (deg)", fill="#111", font=F)
vmin = min(min(sug), min(app)); vmax = max(max(sug), max(app))
axis(top0, vmin, vmax, "deg")
d.line([(xp(t[i]), yp(sug[i], vmin, vmax, top0)) for i in range(len(t))], fill="#2563eb", width=2)
d.line([(xp(t[i]), yp(app[i], vmin, vmax, top0)) for i in range(len(t))], fill="#dc2626", width=2)
d.text((W-PADR-260, top0+8), "blue=suggested  red=applied", fill="#555", font=Fs)

# panel 1: mixed offset us
top1 = PADT + 1*ph
d.text((10, top1 + 8), "Mixed altitude offset (us)", fill="#111", font=F)
vmin, vmax = 0, max(off + [1])
axis(top1, vmin, vmax, "us")
d.line([(xp(t[i]), yp(off[i], vmin, vmax, top1)) for i in range(len(t))], fill="#9333ea", width=2)

# panel 2: servo PWM
top2 = PADT + 2*ph
d.text((10, top2 + 8), "Servo PWM (us, center 1500)", fill="#111", font=F)
allp = rpwm + ppwm + [1500]
vmin, vmax = min(allp), max(allp)
axis(top2, vmin, vmax, "us")
d.line([(xp(t[i]), yp(rpwm[i], vmin, vmax, top2)) for i in range(len(t))], fill="#16a34a", width=2)
d.line([(xp(t[i]), yp(ppwm[i], vmin, vmax, top2)) for i in range(len(t))], fill="#ea580c", width=2)
cy = yp(1500, vmin, vmax, top2)
d.line([PADL, cy, W-PADR, cy], fill="#999999", width=1)
d.text((W-PADR-220, top2+8), "green=roll  orange=pitch", fill="#555", font=Fs)

# panel 3: attitude PID output
top3 = PADT + 3*ph
d.text((10, top3 + 8), "Attitude PID output (us)", fill="#111", font=F)
vmin = min(min(rpid), min(ppid)); vmax = max(max(rpid), max(ppid))
axis(top3, vmin, vmax, "us")
d.line([(xp(t[i]), yp(rpid[i], vmin, vmax, top3)) for i in range(len(t))], fill="#16a34a", width=2)
d.line([(xp(t[i]), yp(ppid[i], vmin, vmax, top3)) for i in range(len(t))], fill="#ea580c", width=2)
d.text((W-PADR-220, top3+8), "green=roll  orange=pitch", fill="#555", font=Fs)

# x labels
for i in range(N):
    top = PADT + i*ph
    for g in range(11):
        v = t[0] + (t[-1]-t[0])*g/10
        d.text((xp(v)-14, top+ph-6), f"{v:.0f}", fill="#888", font=Fs)

d.text((W/2-230, 2), "Fin mixing (run2)  -  yellow = altitude control active", fill="#111", font=F)

img.save(OUT)
print("saved", OUT)
