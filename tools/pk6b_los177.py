#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按视线几何算出"高位相机 → 入口核验位"这条射线上到底谁挡着。

为什么不能只按"中心距"筛（memory「接触几何而非中心距」）：一个 6 m 宽的盒子，
中心离射线 4 m 也照样挡；一个 0.2 m 的杆，中心压在射线上才算。所以这里对每个
model 取它自己名下所有 <size>/<scale> 的最大外接盒，半宽取 max(sx,sy)/2，
半高取 sz/2，再和"射线在该水平位置上的离地高度"比：
    水平：|中心到线段的距离| <= 半宽 + 余量
    竖直：中心 z + 半高 >= 射线同处的 z（且中心 z - 半高 <= 射线 z + 2，否则是头顶的东西）
camera 与目标点都按 world 系给。
"""
import glob
import os
import re
import subprocess
import sys

CAM = (20.0, -20.0, 14.7)          # intersection_cam 的 model pose（world 里读到的）
TARGETS = [(12.00, -12.21, 6.90),  # 入口核验位（car_6 现在停在这）
           (12.00, -12.80, 6.90),  # 入口等待位
           (6.80, -9.95, 6.90),    # 出口核验位
           (8.40, -7.95, 6.90)]    # 车位 P1


def world_path():
    out = subprocess.run(['pgrep', '-f', 'gzserver'], capture_output=True, text=True)
    for pid in out.stdout.split():
        try:
            argv = open('/proc/%s/cmdline' % pid, 'rb').read().decode('utf-8', 'ignore').split('\0')
            cwd = os.readlink('/proc/%s/cwd' % pid)
        except OSError:
            continue
        w = next((a for a in argv if a.endswith('.world')), None)
        if not w:
            continue
        cand = [w] if os.path.isabs(w) else [os.path.join(cwd, w)]
        if not os.path.isfile(cand[0]):
            cand += glob.glob(os.path.expanduser('~/v2x_ws/src/**/' + os.path.basename(w)),
                              recursive=True)
        f = next((c for c in cand if os.path.isfile(c)), None)
        if f:
            print('运行期 world：%s' % f)
            return f
    return None


def models(txt):
    """返回 [(name, x, y, z, 半宽, 半高)]，include 块用 include 自己的 pose。"""
    rows = []
    pat = re.compile(r'^\s*<model name="([^"]+)"', re.M)
    for m in pat.finditer(txt):
        # 往前找最近的 <include>，若有 include 级 <pose> 则以它为准（模型内 pose 是相对它的）
        head = txt[max(0, m.start() - 600):m.start()]
        ip = None
        if '<include>' in head:
            g = re.findall(r'<pose>([^<]+)</pose>', head)
            if g:
                ip = [float(v) for v in g[-1].split()]
        seg = txt[m.end():m.end() + 2500]
        cut = seg.find('\n    </model>')
        if cut > 0:
            seg = seg[:cut]
        p = re.search(r'<pose>([^<]+)</pose>', seg)
        if not p:
            continue
        v = [float(x) for x in p.group(1).split()]
        if len(v) < 6:
            v += [0.0] * (6 - len(v))
        if ip:
            v = [v[0] + ip[0], v[1] + ip[1], v[2] + ip[2]] + v[3:]
        sizes = [[float(t) for t in s.split()] for s in
                 re.findall(r'<(?:size|scale)>([^<]+)</(?:size|scale)>', seg)]
        if not sizes:
            r = re.findall(r'<radius>([\d.]+)</radius>', seg)
            sizes = [[float(x) * 2, float(x) * 2, float(x) * 2] for x in r[:1]]
        if not sizes:
            continue
        big = max(sizes, key=lambda s: s[0] * s[1])
        sx = big[0] if len(big) > 0 else 0.0
        sy = big[1] if len(big) > 1 else sx
        sz = big[2] if len(big) > 2 else 0.0
        rows.append((m.group(1), v[0], v[1], v[2], max(sx, sy) / 2.0, sz / 2.0))
    return rows


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else ((px - ax) * dx + (py - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5, t


def main():
    f = world_path()
    if not f:
        print('!! 找不到运行期 world 文件')
        return 1
    rows = models(open(f, encoding='utf-8', errors='ignore').read())
    print('解析到带几何的 model %d 个' % len(rows))
    hits = []
    for (tx, ty, tz) in TARGETS:
        for name, mx, my, mz, hw, hh in rows:
            d, t = seg_dist(mx, my, CAM[0], CAM[1], tx, ty)
            if t <= 0.02 or t >= 0.98:
                continue                      # 相机脚下与目标点本身不算
            ray_z = CAM[2] + t * (tz - CAM[2])
            if d <= hw + 0.5 and (mz + hh) >= ray_z and (mz - hh) <= ray_z + 3.0:
                hits.append((t, d, name, mx, my, mz, hw, hh, ray_z, tx, ty))
    hits.sort()
    print('%-4s %-22s %-22s %8s %8s %8s %8s' % (
        't', 'model', '目标(核验位/车位)', '水平距', '半宽', '顶高', '射线高'))
    for t, d, name, mx, my, mz, hw, hh, ray_z, tx, ty in hits[:20]:
        print('%-4.2f %-22s (%5.2f,%6.2f)      %8.2f %8.2f %8.2f %8.2f' % (
            t, name, tx, ty, d, hw, mz + hh, ray_z))
    print('挡在视线上的候选 %d 条' % len(hits))

    # 不做任何过滤，按"到射线的水平距离"把全部 model 排一遍：
    # 上面那条判据如果把高度算漏了（mesh 类 model 的 SDF 里没有真实外接盒），
    # 这张全量表就是兜底，能直接看出 t 落在 0.3..0.9 之间、离射线最近的是谁。
    print('=== 全量按水平距排序（目标=入口核验位 12.00,-12.21；t 是沿射线的比例）===')
    tx, ty, tz = TARGETS[0]
    allr = []
    for name, mx, my, mz, hw, hh in rows:
        d, t = seg_dist(mx, my, CAM[0], CAM[1], tx, ty)
        allr.append((d, t, name, mx, my, mz, hw, hh))
    allr.sort()
    print('%-24s %6s %6s %8s %8s %8s %8s' % (
        'model', 't', '水平距', 'x', 'y', 'z', '半宽/半高'))
    for d, t, name, mx, my, mz, hw, hh in allr[:14]:
        print('%-24s %6.2f %6.2f %8.2f %8.2f %8.2f %6.2f/%.2f' % (
            name, t, d, mx, my, mz, hw, hh))
    return 0


if __name__ == '__main__':
    sys.exit(main())
