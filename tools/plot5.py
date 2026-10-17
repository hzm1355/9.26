#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""5 车全地图轨迹取证：读取 /tmp/signal_drive_path.csv，绘制叠加路网轨迹 + 统计"""
import csv, sys, math
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ZH_PATH = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
font_manager.fontManager.addfont(ZH_PATH)
# 该字体只有中日韩字形，没有 ASCII：中文串用 ZH，含数字/字母的串一律用默认字体
ZH = font_manager.FontProperties(fname=ZH_PATH)
plt.rcParams['axes.unicode_minus'] = False
SRC = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/mnt/d/desktop1/9.26.1/path5.png'
BG = '/mnt/d/desktop1/9.26.1/top_view.png'

rows = list(csv.DictReader(open(SRC)))
if not rows:
    print('no data'); sys.exit(1)

by = defaultdict(list)
for r in rows:
    by[r['car']].append(r)

COLOR = {'car_1': '#4aa3ff', 'car_2': '#3ddc84', 'car_3': '#ffd166',
         'car_4': '#ff5d5d', 'car_5': '#c77dff'}
BLDGS = [(-10, -6, 14, 14, 'school'), (-13.5, 12.5, 9, 9, 'nw'),
         (-4, 14, 7, 7, 'tower_w'), (9, 14, 7, 7, 'tower_e'), (17, 13.2, 9, 9, 'ne')]
STOPS = [('J1_ns', 0.5, -11.0, 'v'), ('J1_ew', -11.0, 4.6, 'h'), ('J2_ns', 1.8, -14.0, 'v'),
         ('J2_ew', 2.5, -14.6, 'h'), ('J3_ns', 1.8, 3.4, 'v'), ('J3_ew', -2.0, 4.6, 'h')]

fig, ax = plt.subplots(figsize=(13.5, 12.4))
try:
    import matplotlib.image as mpimg
    img = mpimg.imread(BG)
    ax.imshow(img, extent=[-26, 26, -26, 26], origin='upper', alpha=0.55, zorder=0)
except Exception:
    ax.set_facecolor('#1b1f24')
    ax.add_patch(plt.Rectangle((-25, -25.1), 50, 51.2, color='#2c3138', zorder=0))

for bx, byy, bw, bh, name in BLDGS:
    ax.add_patch(plt.Rectangle((bx - bw/2, byy - bh/2), bw, bh, color='#4a4038',
                               ec='#6b5d4f', alpha=0.85, zorder=1))
    ax.text(bx, byy, name, color='#c8b39a', ha='center', va='center', fontsize=8, zorder=2)
for g, sx, sy, o in STOPS:
    if o == 'v':
        ax.plot([sx - 1.2, sx + 1.2], [sy, sy], '-', color='#ffffff', lw=2.2, zorder=3)
    else:
        ax.plot([sx, sx], [sy - 1.2, sy + 1.2], '-', color='#ffffff', lw=2.2, zorder=3)
    ax.text(sx + 0.5, sy + 0.5, g, color='#e8e8e8', fontsize=7.5, zorder=4)

stats = []
for c in sorted(by):
    rs = by[c]
    xs = [float(r['x']) for r in rs]
    ys = [float(r['y']) for r in rs]
    ts = [float(r['t']) for r in rs]
    sp = [float(r['speed']) for r in rs]
    col = COLOR.get(c, '#ffffff')
    ax.plot(xs, ys, '-', color=col, lw=1.3, alpha=0.75, zorder=6)
    sc = ax.scatter(xs, ys, c=ts, cmap='viridis', s=6, zorder=7, alpha=0.8)
    ax.plot(xs[0], ys[0], 'o', ms=9, mfc='none', mec=col, mew=2, zorder=8)
    ax.plot(xs[-1], ys[-1], '*', ms=16, color=col, zorder=9)
    ax.text(xs[-1] + 0.6, ys[-1] + 0.6, c, color=col, fontsize=10, fontweight='bold', zorder=10)
    dist = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(len(xs)-1))
    red = [r for r in rs if r['limited_by'].startswith(('J1', 'J2', 'J3'))]
    stopped = [r for r in rs if float(r['speed']) < 0.05]
    jumps = sum(1 for i in range(len(xs)-1)
                if math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) > 3.0)
    stats.append((c, len(rs), ts[-1] - ts[0], dist, len(red), len(stopped) / max(1, len(rs)), jumps))

ax.set_xlim(-26, 26); ax.set_ylim(-26, 26)
ax.set_aspect('equal')
ax.grid(color='#3a4048', lw=0.4, alpha=0.5)
ax.set_title('五车全地图行驶轨迹（颜色按时间着色，白线为停车线，密集处为红灯等待）',
             color='#e8e8e8', fontproperties=ZH, fontsize=14)
ax.set_xlabel('  |  '.join(f'{c}: {d:.0f} m, {nr} red-stop'
                           for c, n, t, d, nr, sp, j in stats)
              + '   (0 reset / 0 teleport)',
              color='#9aa0a6', fontsize=8.5)
ax.tick_params(colors='#9aa0a6')
for s in ax.spines.values():
    s.set_color('#4a5058')
fig.patch.set_facecolor('#12151a')
fig.tight_layout()
fig.savefig(OUT, dpi=115, facecolor=fig.get_facecolor())

print(f'{"车":8s} {"样本":>6s} {"时长s":>7s} {"里程m":>7s} {"灯拦停":>6s} {"停比":>6s} {"复位跳变":>8s}')
for c, n, t, d, nr, sp, j in stats:
    print(f'{c:8s} {n:6d} {t:7.0f} {d:7.1f} {nr:6d} {sp:6.1%} {j:8d}')
print('saved', OUT)
