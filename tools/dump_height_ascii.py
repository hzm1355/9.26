#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 tools/route_corridor_scan.py 建好的落地障碍高度图打成米制字符图。

为什么要它：走廊扫描只给"某条腿平移多少净空"，看不出**整个沙盘的可行驶路网**
长什么样。上一轮路线是按沙盘上的**路面涂装**画的（config 注释里 x≈-20.2 / y≈5.5
那套），而网格里的建筑体块压在同一批坐标上 —— 车照着涂装穿楼。改路线之前必须先
看清"哪些格子是空的"，所以按 1 m 一格打三档高度：
    .   无落地障碍或 <0.45 m（路缘石、地面涂装，车能压过去）
    :   0.45~1.2 m（护栏、柱廊、花坛，车体必然刮到）
    #   >=1.2 m（建筑，用户截图里"穿楼"就是这一档）
用法（WSL，仓库根）：
  python3 tools/route_corridor_scan.py --rebuild   # 先建图
  python3 tools/dump_height_ascii.py               # 再打字符图
"""
import json
import sys

MAP = '/tmp/route_height_map.json'
CELL = 0.25
X0, X1 = -25.0, 25.0
Y0, Y1 = -21.0, 21.0
STEP = int(sys.argv[1]) if len(sys.argv) > 1 else 1     # 每格采样 STEP 个 0.25 m 单元


def main():
    with open(MAP, encoding='utf-8') as fh:
        m = json.load(fh)
    nc, nr = m['nc'], m['nr']
    g = [[0.0] * nc for _ in range(nr)]
    for ix, iy, h, _mat in m['cells']:
        g[iy][ix] = max(g[iy][ix], h)
    # 每米取窗口内的最大值（宁可高估障碍，别把柱子看没了）
    rows = []
    y = Y1 - STEP / 2.0
    while y >= Y0:
        line = []
        x = X0 + STEP / 2.0
        while x <= X1:
            h = 0.0
            i0 = int((x - STEP / 2.0 - X0) / CELL)
            i1 = int((x + STEP / 2.0 - X0) / CELL)
            j0 = int((y - STEP / 2.0 - Y0) / CELL)
            j1 = int((y + STEP / 2.0 - Y0) / CELL)
            for iy in range(max(0, j0), min(nr, j1 + 1)):
                for ix in range(max(0, i0), min(nc, i1 + 1)):
                    if g[iy][ix] > h:
                        h = g[iy][ix]
            line.append('#' if h >= 1.2 else (':' if h >= 0.45 else
                                              ('.' if h > 0 else ' ')))
            x += STEP
        rows.append(('y=%+5.1f ' % y) + ''.join(line))
        y -= STEP
    ruler = []
    for k in range(int((X1 - X0) / STEP)):
        xv = X0 + STEP / 2.0 + STEP * k
        ruler.append('%d' % int(xv) if xv % 5 == 0 else ' ')
    print('每格 %d m   图例：# 落地 >=1.2 m（建筑）   : 0.45~1.2 m（护栏/柱廊/花坛）'
          '   . >0.15 m（路缘）   空格 无' % STEP)
    print('        ' + ''.join(ruler))
    for r in rows:
        print(r)


if __name__ == '__main__':
    main()
