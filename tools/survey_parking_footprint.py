#!/usr/bin/env python3
"""核算停车场候选占地的净空：用 room_33.obj 顶点高度场栅格化到世界 XY。

世界坐标换算沿用已验证关系（model pose xyz=0 0 6.7 / rpy=1.5708 0 0 / scale=10）：
  wx = 10 * x
  wy = -10 * z
  wz = 10 * y + 6.7

路面基准 wz=6.7，高于基准 pitch 的顶点算障碍。输出逐格最高点的 ASCII 平面图，
用于确定入口/出口车道、闸机、地感线圈、车位的最终点位。
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--x0", type=float, default=7.0)
    ap.add_argument("--x1", type=float, default=15.5)
    ap.add_argument("--y0", type=float, default=-13.5)
    ap.add_argument("--y1", type=float, default=-5.5)
    ap.add_argument("--cell", type=float, default=0.5)
    ap.add_argument("--pitch", type=float, default=0.6, help="高于路面多少米算障碍")
    ap.add_argument("--road", type=float, default=6.7)
    args = ap.parse_args()

    # 先把候选框换成 obj 局部坐标，读文件时就地筛掉绝大多数顶点。
    lx0, lx1 = args.x0 / 10.0, args.x1 / 10.0
    lz0, lz1 = -args.y1 / 10.0, -args.y0 / 10.0
    margin = 0.3  # 一个格子宽的余量，避免漏掉跨框顶点的组

    nx = max(1, int(round((args.x1 - args.x0) / args.cell)))
    ny = max(1, int(round((args.y1 - args.y0) / args.cell)))
    high: dict[tuple[int, int], float] = {}
    hi_all: dict[tuple[int, int], float] = {}
    cnt: dict[tuple[int, int], int] = defaultdict(int)
    total_v = 0
    kept_v = 0
    gmin = [math.inf] * 3
    gmax = [-math.inf] * 3

    with open(args.obj, "rb") as fh:
        for line in fh:
            if not line.startswith(b"v "):
                continue
            total_v += 1
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            wx, wy, wz = 10.0 * x, -10.0 * z, 10.0 * y + args.road
            for i, (lo, hi, v) in enumerate(zip(gmin, gmax, (wx, wy, wz))):
                if v < lo:
                    gmin[i] = v
                if v > hi:
                    gmax[i] = v
            if not (lx0 - margin <= x <= lx1 + margin and lz0 - margin <= z <= lz1 + margin):
                continue
            if not (args.x0 <= wx <= args.x1 and args.y0 <= wy <= args.y1):
                continue
            kept_v += 1
            ix = min(nx - 1, max(0, int((wx - args.x0) / args.cell)))
            iy = min(ny - 1, max(0, int((wy - args.y0) / args.cell)))
            key = (ix, iy)
            cnt[key] += 1
            if wz > hi_all.get(key, -math.inf):
                hi_all[key] = wz
            if wz <= args.road + args.pitch:
                continue
            if wz > high.get(key, -math.inf):
                high[key] = wz

    print(f"顶点总数 {total_v}，落在候选框内 {kept_v}")
    print(f"世界 AABB  x[{gmin[0]:.2f},{gmax[0]:.2f}] y[{gmin[1]:.2f},{gmax[1]:.2f}] z[{gmin[2]:.2f},{gmax[2]:.2f}]")
    print(f"候选框 {args.x0}..{args.x1} × {args.y0}..{args.y1}，格子 {args.cell} m，净空门限 路面+{args.pitch} m")
    print(f"受阻格子 {len(high)} / {nx * ny}")

    # 平面图：纵轴为 wy（上为北），横轴为 wx（右为东）。
    rows = []
    for iy in range(ny - 1, -1, -1):
        line = f"y{args.y0 + iy * args.cell:6.1f} "
        for ix in range(nx):
            key = (ix, iy)
            if key not in cnt:
                line += " "
            elif key in high:
                line += "#"
            elif hi_all[key] > args.road - 0.2:
                line += "." if (ix + iy) % 2 == 0 else "-"
            else:
                line += "o"
        rows.append(line)
    print("\n".join(rows))
    print("       " + "".join(f"{args.x0 + ix * args.cell:>4.1f}" if ix % 2 == 0 else "    " for ix in range(nx)))
    print("       # = 高于路面门槛的物体（建筑/设施）  . - = 净空路面  o = 低于路面  空格 = 无几何（空洞）")

    blocked_x = sorted({ix for ix, _ in high})
    if blocked_x:
        print("\n按 wy 条带给出受阻区间：")
        for iy in range(ny - 1, -1, -1):
            bad = [ix for ix in range(nx) if (ix, iy) in high]
            if not bad:
                continue
            groups = []
            start = prev = bad[0]
            for ix in bad[1:]:
                if ix == prev + 1:
                    prev = ix
                else:
                    groups.append((start, prev))
                    start = prev = ix
            groups.append((start, prev))
            desc = ", ".join(
                f"x{args.x0 + a * args.cell:.1f}..{args.x0 + (b + 1) * args.cell:.1f}"
                f"(最高 {max(high[(i, iy)] for i in range(a, b + 1)) - args.road:.1f} m)"
                for a, b in groups
            )
            print(f"  y {args.y0 + iy * args.cell:5.1f}..{args.y0 + (iy + 1) * args.cell:5.1f}: {desc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
