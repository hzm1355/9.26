#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按材质名前缀导出沙盘网格里的贴地三角形，并画高分辨率占用图。

用途：pk110 扫到"白色分隔线331"这一组的 AABB 是 x 7.15..16.52 × y −11.63..−7.03，
和那块深绿贴地矩形（Standardmaterialdd3333.001，x 7.30..16.37 × y −11.48..−7.18）
几乎重合 —— 也就是**原始沙盘自己**在这块地上画了白色分隔线。分隔线有几条、
间距多少，直接决定"原始模型标记了几个车位、每个位多大"，这是用户那句
"停车位是原始模型中已经标记的部分"唯一可核实的读法，不能靠目测截图。

同一套代码要跑两次（绿块一次、白线一次），所以材质名走命令行前缀匹配：
`白色分隔线331` 与 `白色分隔线331.001` 是同组的两份（Ka/Kd），前缀能一起命中。

用法（WSL）：
  cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
  python3 tools/pk111_matfaces218.py <材质名前缀> [obj] [x0 x1 y0 y1] [格子]
"""
import os
import sys
from array import array

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREFIX = sys.argv[1]
OBJ = sys.argv[2] if len(sys.argv) > 2 else '/tmp/room_33_214.obj'
MTL = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.mtl')
S, FLAT_H = 10.0, 0.20
X0, X1 = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 5 else (2.0, 22.0)
Y0, Y1 = (float(sys.argv[5]), float(sys.argv[6])) if len(sys.argv) > 7 else (-20.0, -2.0)
CELL = float(sys.argv[7]) if len(sys.argv) > 8 else 0.25
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pk107_green214 import load_mtl       # noqa: E402


def main():
    kd = load_mtl(MTL)
    # 前缀直接比"当前 usemtl 或 g 组名"，不比 mtl 键集合：这份 obj 的"白色分隔线331"
    # 是 **组名（g）**而不是材质名（usemtl），拿 mtl 键当白名单会一个都匹配不上
    # （pk107/pk110 的 cur 同时收 g 和 usemtl，所以它们能统计到，第一版这里却不行）。
    print('按前缀 %r 匹配组名/材质名；mtl 中同前缀的材质：%s' % (
        PREFIX, '、'.join(sorted(k for k in kd if k.startswith(PREFIX))) or '（无）'))

    vx, vy, vz = array('d'), array('d'), array('d')
    tris = []
    cur = '?'
    n_v = 0
    with open(OBJ, 'rb', buffering=1 << 22) as fh:
        for raw in fh:
            c0 = raw[:1]
            if c0 == b'v' and raw[1:2] == b' ':
                p = raw[2:].split()
                vx.append(float(p[0]))
                vy.append(float(p[1]))
                vz.append(float(p[2]))
                n_v += 1
            elif c0 == b'f' and n_v:
                if not cur.startswith(PREFIX):
                    continue
                idx = []
                for tok in raw[1:].split():
                    try:
                        i = int(tok.split(b'/')[0])
                    except ValueError:
                        continue
                    idx.append(n_v + i + 1 if i < 0 else i)
                if len(idx) < 3:
                    continue
                a = idx[0] - 1
                for j in range(1, len(idx) - 1):
                    b, c = idx[j] - 1, idx[j + 1] - 1
                    tri = [(S * vx[k], -S * vz[k], S * vy[k]) for k in (a, b, c)]
                    xs = [t[0] for t in tri]
                    ys = [t[1] for t in tri]
                    hs = [t[2] for t in tri]
                    if max(hs) > FLAT_H:
                        continue
                    if max(xs) < X0 or min(xs) > X1 or max(ys) < Y0 or min(ys) > Y1:
                        continue
                    tris.append(tri)
            elif c0 == b'u' or c0 == b'g':
                sp = raw.split(None, 1)
                if len(sp) > 1:
                    cur = sp[1].decode('utf-8', 'replace').strip()

    print('贴地（h<=%.2f）三角形 %d 个' % (FLAT_H, len(tris)))
    for tri in tris:
        print('  (%.2f,%.2f,h%.3f) (%.2f,%.2f,h%.3f) (%.2f,%.2f,h%.3f)' % (
            tri[0][0], tri[0][1], tri[0][2], tri[1][0], tri[1][1], tri[1][2],
            tri[2][0], tri[2][1], tri[2][2]))

    cells = {}
    for tri in tris:
        for px, py, _ in tri:
            k = (int(px // CELL), int(py // CELL))
            cells[k] = cells.get(k, 0) + 1
    if not cells:
        return 0
    ks = list(cells)
    i0, i1 = min(k[0] for k in ks), max(k[0] for k in ks)
    j0, j1 = min(k[1] for k in ks), max(k[1] for k in ks)
    print('\n占用图（格 %.2f m，列=X 自 %.2f 每列 +%.2f，共 %d 列；行=Y 标左侧）' % (
        CELL, i0 * CELL, CELL, i1 - i0 + 1))
    print('        ' + ''.join('|' if (i0 + i) % 8 == 0 else ' ' for i in range(i1 - i0 + 1)))
    for j in range(j1, j0 - 1, -1):
        print('%7.2f %s' % (j * CELL, ''.join(
            '#' if cells.get((i, j)) else '.' for i in range(i0, i1 + 1))))
    print('列 X 刻度：' + '  '.join('%.2f' % ((i0 + i) * CELL) for i in range(0, i1 - i0 + 1, 8)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
