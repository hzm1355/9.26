#!/usr/bin/env python3
"""用**三角形精确覆盖**（不是顶点落格）复核停车场占地的地面与净空。

为什么还要第二个脚本：survey_parking_footprint.py 统计的是"顶点落在哪一格"。
大块的平面（整片路面、整栋房的墙）在 OBJ 里往往只有 4 个角点，角点全在候选框外
时，框内每格的顶点计数都是 0 —— 看起来就是"空洞"，但它其实被一个大三角形盖住。
所以顶点图只能证明"这里没有密的障碍网格"，不能证明"这里到底有没有地面"。

本脚本逐面取出三角形（多边形按 fan 剖分），对格心做精确的内外判定，同时记录
盖到该格的最高点：
  · '.'/'-'  —— 有几何盖住且与路面齐平（可以直接当停车场地坪）
  · 'o'      —— 盖住的几何低于路面（坑/凹槽，铺我们的地坪即可盖住）
  · '#'      —— 障碍
  · 空格     —— 一个三角形都没盖到 = 真·空洞，地面必须由 gen_parking_world.py 自己铺

坐标换算沿用任务三标定：wx=10*x、wy=-10*z、wz=10*y+road（模型 pose 0 0 6.7
roll=90°、scale=10）。顶点存 array('f') 三个数组（2.97 M 点 ≈ 36 MB），逐行流式
读 f 面，不做二次遍历。
"""

from __future__ import annotations

import argparse
import array
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--x0", type=float, default=7.40)
    ap.add_argument("--x1", type=float, default=14.60)
    ap.add_argument("--y0", type=float, default=-13.80)
    ap.add_argument("--y1", type=float, default=-5.50)
    ap.add_argument("--cell", type=float, default=0.25)
    ap.add_argument("--pitch", type=float, default=0.30, help="高于路面多少米算障碍")
    ap.add_argument("--road", type=float, default=6.7)
    args = ap.parse_args()

    vx = array.array("f")
    vy = array.array("f")
    vz = array.array("f")

    nx = max(1, int(round((args.x1 - args.x0) / args.cell)))
    ny = max(1, int(round((args.y1 - args.y0) / args.cell)))
    total = nx * ny
    hit = bytearray(total)
    hi = array.array("f", bytes(4 * total))   # 全 0，随后写世界 z（>6.0 恒正，够用）
    ntri = 0
    tested = 0
    big = {}

    # 格心坐标表，避免每面重算
    cx = [args.x0 + (i % nx + 0.5) * args.cell for i in range(total)]
    cy = [args.y0 + (i // nx + 0.5) * args.cell for i in range(total)]

    with open(args.obj, "rb") as fh:
        for line in fh:
            head = line[:2]
            if head == b"v ":
                p = line.split()
                vx.append(float(p[1]))
                vy.append(float(p[2]))
                vz.append(float(p[3]))
                continue
            if head != b"f ":
                continue
            idxs = []
            bad = False
            for tok in line.split()[1:]:
                try:
                    idxs.append(int(tok.split(b"/")[0]))
                except ValueError:
                    bad = True
                    break
            if bad or len(idxs) < 3:
                continue
            if max(idxs) > len(vx) or min(idxs) < 1:
                continue
            pts = [(10.0 * vx[i - 1], -10.0 * vz[i - 1], 10.0 * vy[i - 1] + args.road)
                   for i in idxs]
            # fan 剖分：四边形及以上按 (0,k,k+1) 拆
            for k in range(1, len(pts) - 1):
                ax, ay, _ = pts[0][0], pts[0][2], 0
                bx, by, _ = pts[k][0], pts[k][2], 0
                ccx, ccy, _ = pts[k + 1][0], pts[k + 1][2], 0
                top = max(p[1] for p in (pts[0], pts[k], pts[k + 1]))
                ntri += 1
                minx = ax if ax < bx else bx
                if ccx < minx:
                    minx = ccx
                maxx = ax if ax > bx else bx
                if ccx > maxx:
                    maxx = ccx
                miny = ay if ay < by else by
                if ccy < miny:
                    miny = ccy
                maxy = ay if ay > by else by
                if ccy > maxy:
                    maxy = ccy
                if maxx < args.x0 or minx > args.x1 or maxy < args.y0 or miny > args.y1:
                    continue
                ix0 = max(0, int((minx - args.x0) / args.cell))
                ix1 = min(nx - 1, int((maxx - args.x0) / args.cell))
                iy0 = max(0, int((miny - args.y0) / args.cell))
                iy1 = min(ny - 1, int((maxy - args.y0) / args.cell))
                nc = (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
                big[nc] = big.get(nc, 0) + 1
                ex1, ey1 = bx - ax, by - ay
                ex2, ey2 = ccx - ax, ccy - ay
                d = ex1 * ey2 - ex2 * ey1
                if d == 0.0:
                    continue
                for iy in range(iy0, iy1 + 1):
                    base = iy * nx
                    for ix in range(ix0, ix1 + 1):
                        c = base + ix
                        qx = cx[c] - ax
                        qy = cy[c] - ay
                        u = (qx * ey2 - qy * ex2) / d
                        v = (qy * ex1 - qx * ey1) / d
                        if u < 0.0 or v < 0.0 or u + v > 1.0:
                            continue
                        tested += 1
                        hit[c] = 1
                        if top > hi[c]:
                            hi[c] = top
            if ntri and ntri % 500000 == 0:
                print(f"  ... {ntri // 1000}k 三角形", file=sys.stderr, flush=True)

    holes = sum(1 for c in range(total) if not hit[c])
    blocked = sum(1 for c in range(total) if hit[c] and hi[c] > args.road + args.pitch)
    print(f"三角形 {ntri}，顶点 {len(vx)}，落入框内并命中格心 {tested} 次")
    print(f"候选框 {args.x0}..{args.x1} × {args.y0}..{args.y1}"
          f"，格子 {args.cell} m，净空门限 路面+{args.pitch} m")
    print(f"空洞格子 {holes} / {total}，受阻格子 {blocked} / {total}，"
          f"有地面格子 {total - holes - blocked} / {total}")
    top_sizes = sorted(big.items(), key=lambda kv: -kv[0])[:6]
    print("单面跨越格子数 Top6（格数:面数）：" +
          ", ".join(f"{k}:{v}" for k, v in top_sizes))

    for iy in range(ny - 1, -1, -1):
        row = f"y{args.y0 + iy * args.cell:6.1f} "
        for ix in range(nx):
            c = iy * nx + ix
            if not hit[c]:
                row += " "
            elif hi[c] > args.road + args.pitch:
                row += "#"
            elif hi[c] > args.road - 0.2:
                row += "." if (ix + iy) % 2 == 0 else "-"
            else:
                row += "o"
        print(row)
    print("       " + "".join(f"{args.x0 + ix * args.cell:>4.1f}" if ix % 2 == 0 else "    "
                              for ix in range(nx)))
    print("       # 障碍  . - 与路面齐平的几何  o 低于路面的几何  空格 无任何三角形覆盖")
    return 0


if __name__ == "__main__":
    sys.exit(main())
