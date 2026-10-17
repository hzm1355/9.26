import csv, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 该字体只含中文字形，故仅对中文文本单独指定，拉丁/数字仍走默认字体
ZH = font_manager.FontProperties(fname='/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')

SRC = sys.argv[1] if len(sys.argv) > 1 else '/tmp/loop_path.csv'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/mnt/d/desktop1/9.26.1/loop_path.png'

rows = list(csv.DictReader(open(SRC)))
if not rows:
    print('no data'); sys.exit(1)

xs = [float(r['x']) for r in rows]
ys = [float(r['y']) for r in rows]
idx = [int(r['idx']) for r in rows]
ts = [float(r['t']) for r in rows]
err = [abs(float(r['err'])) for r in rows]

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.6), gridspec_kw={'width_ratios': [1.15, 1]})

ax.set_facecolor('#1b1f24')
ax.add_patch(plt.Rectangle((-25, -25.1), 50, 51.2, color='#2c3138', zorder=0))
# 建筑（world 中的碰撞盒）
bldgs = [(-10, -8, 16, 12, 'school'), (-14, 12, 10, 10, 'nw'),
         (-4, 14, 7, 7, 'tower_w'), (9, 14, 7, 7, 'tower_e'), (17, 11, 9, 9, 'ne')]
for bx, by, bw, bh, name in bldgs:
    ax.add_patch(plt.Rectangle((bx - bw/2, by - bh/2), bw, bh, color='#4a4038',
                               ec='#6b5d4f', zorder=1))
    ax.text(bx, by, name, color='#a89684', ha='center', va='center', fontsize=7, zorder=2)
# 相机立杆（实体障碍）
for px, py, name in [(0, -23, 'speed_cam'), (20, -20, 'inter_cam'), (-20, 20, 'monitor_cam')]:
    ax.plot(px, py, 's', ms=6, color='#c0392b', zorder=3)
    ax.text(px, py + 1.4, name, color='#e08b7e', ha='center', fontsize=6.5, zorder=3)

RING = [(-17, -17), (-6, -17), (-6, -22), (-17, -22)]
ring_closed = RING + [RING[0]]
ax.plot([p[0] for p in ring_closed], [p[1] for p in ring_closed], '--',
        color='#3ddc84', lw=1.4, alpha=0.75, zorder=4, label='规划环路')
for i, p in enumerate(RING):
    ax.plot(p[0], p[1], 'D', ms=7, color='#3ddc84', zorder=5)
    ax.text(p[0], p[1] - 1.6, 'W%d' % i, color='#3ddc84', ha='center', fontsize=7, zorder=5)

ax.plot(xs, ys, '-', color='#4aa3ff', lw=1.1, alpha=0.55, zorder=6)
sc = ax.scatter(xs, ys, c=ts, cmap='viridis', s=5, zorder=7)
ax.plot(2, -16, '^', ms=11, color='#ffd166', zorder=8, label='出生点')
ax.plot(xs[0], ys[0], 'o', ms=7, color='#ffffff', zorder=8)
ax.plot(xs[-1], ys[-1], '*', ms=15, color='#ff5d5d', zorder=9, label='终点')
ax.set_xlim(-26, 26); ax.set_ylim(-26, 27)
ax.set_aspect('equal')
ax.grid(color='#3a4048', lw=0.4, alpha=0.6)
ax.set_title('小车行驶轨迹（按时间着色）', color='#e8e8e8', fontproperties=ZH)
ax.set_xlabel('car_1 · %d samples · %.0f s' % (len(rows), ts[-1]), color='#9aa0a6')
ax.tick_params(colors='#9aa0a6')
for s in ax.spines.values():
    s.set_color('#4a5058')
ax.legend(loc='upper right', fontsize=8, prop=ZH,
          facecolor='#22262b', edgecolor='#4a5058', labelcolor='#e8e8e8')
cb = fig.colorbar(sc, ax=ax, shrink=0.75)
cb.set_label('时间（秒）', color='#9aa0a6', fontproperties=ZH)
cb.ax.yaxis.set_tick_params(color='#9aa0a6')
plt.setp(cb.ax.yaxis.get_ticklabels(), color='#9aa0a6')

ax2.set_facecolor('#1b1f24')
ax2.plot(ts, err, '-', color='#ff9f43', lw=0.9)
ax2.set_xlabel('时间（秒）', color='#9aa0a6', fontproperties=ZH)
ax2.set_ylabel('航向误差（弧度）', color='#9aa0a6', fontproperties=ZH)
ax2.set_ylim(0, 3.2)
ax2.grid(color='#3a4048', lw=0.4, alpha=0.6)
ax2.tick_params(colors='#9aa0a6')
for s in ax2.spines.values():
    s.set_color('#4a5058')
ax2.set_title('航向误差随时间变化（拐角收敛性）', color='#e8e8e8', fontproperties=ZH)
# 路点到达时刻
prev = idx[0]
for i in range(1, len(idx)):
    if idx[i] != prev:
        ax2.axvline(ts[i], color='#3ddc84', lw=0.7, alpha=0.55)
        prev = idx[i]

fig.patch.set_facecolor('#12151a')
fig.tight_layout()
fig.savefig(OUT, dpi=110, facecolor=fig.get_facecolor())
print('saved %s  samples=%d  span=%.0fs' % (OUT, len(rows), ts[-1]))
