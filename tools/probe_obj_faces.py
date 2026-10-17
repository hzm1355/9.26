#!/usr/bin/env python3
"""定位"顶点在框内但三角形盖不到框"的矛盾：一次性统计
   ① 落在候选框内的顶点索引集合大小  ② 引用了这些顶点的面数
   ③ 这些面的 2D 包围盒是否真的与框相交（打印 3 个样本）。
"""
import array
import sys

X0, X1, Y0, Y1, ROAD = 7.40, 14.60, -13.80, -5.50, 6.7

vx = array.array("f")
vy = array.array("f")
vz = array.array("f")
inbox = set()

with open(sys.argv[1], "rb") as fh:
    for line in fh:
        if line[:2] == b"v ":
            p = line.split()
            x, y, z = float(p[1]), float(p[2]), float(p[3])
            vx.append(x)
            vy.append(y)
            vz.append(z)
            wx, wy = 10.0 * x, -10.0 * z
            if X0 <= wx <= X1 and Y0 <= wy <= Y1:
                inbox.add(len(vx))
        elif line[:2] == b"f ":
            toks = line.split()[1:]
            idxs = [int(t.split(b"/")[0]) for t in toks]
            if not inbox.isdisjoint(idxs):
                pts = [(10.0 * vx[i - 1], 10.0 * vy[i - 1], -10.0 * vz[i - 1]) for i in idxs]
                minx = min(p[0] for p in pts)
                maxx = max(p[0] for p in pts)
                miny = min(p[2] for p in pts)
                maxy = max(p[2] for p in pts)
                sep = maxx < X0 or minx > X1 or maxy < Y0 or miny > Y1
                if getattr(sys.modules.get(__name__), "_shown", 0) < 3:
                    print(f"引用框内顶点的面 idxs={idxs} pts={[(round(a,3),round(b,3),round(c,3)) for a,b,c in pts]} "
                          f"bbox x[{minx:.2f},{maxx:.2f}] y[{miny:.2f},{maxy:.2f}] 被剔除={sep}")
                    sys.modules[__name__]._shown = getattr(sys.modules[__name__], "_shown", 0) + 1
print(f"框内顶点 {len(inbox)} 个，顶点总数 {len(vx)}")
