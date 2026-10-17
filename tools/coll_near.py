#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按坐标查碰撞件：给出半径内所有静态盒（名字、footprint、z 区间、离道中心距离）。

用法（WSL，负数一律用 --opt=value 形式）：
  python3 tools/coll_near.py --xy=14.5,-12.5 --r=2.0
  python3 tools/coll_near.py --xy=4.5,1.6 --r=2.0 --json=/mnt/d/desktop1/9.26.1/_world_collisions.json
"""
import argparse
import json


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default='/tmp/world_collisions.json')
    ap.add_argument('--xy', required=True, help='x,y')
    ap.add_argument('--r', type=float, default=2.0)
    ap.add_argument('--zlo', type=float, default=6.6)
    ap.add_argument('--zhi', type=float, default=7.6)
    ap.add_argument('--all', action='store_true', help='连非静态（车体）一起列')
    a = ap.parse_args()

    x, y = [float(v) for v in a.xy.split(',')]
    with open(a.json, encoding='utf-8') as fh:
        boxes = json.load(fh)['boxes']

    hits = []
    for b in boxes:
        if not a.all and not b['static']:
            continue
        a0 = b['aabb']
        # 盒到点的带符号距离：负=点落在 footprint 内
        dx = max(a0[0] - x, 0.0, x - a0[1])
        dy = max(a0[2] - y, 0.0, y - a0[3])
        d = (dx * dx + dy * dy) ** 0.5
        if d > a.r:
            continue
        z_ov = a0[4] < a.zhi and a0[5] > a.zlo
        hits.append((d, '%s/%s' % (b['name'], b['coll'] or b['link']), a0, z_ov,
                    b.get('lane_dist')))
    if not hits:
        print('(%g,%g) 半径 %g m 内没有任何碰撞件' % (x, y, a.r))
        return 1
    hits.sort()
    print('(%g,%g) 半径 %g m 内的碰撞件（按 footprint 距离升序）：' % (x, y, a.r))
    for d, nm, a0, z_ov, ld in hits:
        print('  %-38s d=%+5.2f  x %7.2f..%7.2f  y %7.2f..%7.2f  z %5.2f..%5.2f %s lane=%s' % (
            nm, -d if d else 0.0, a0[0], a0[1], a0[2], a0[3], a0[4], a0[5],
            '受击带' if z_ov else '-- 高度不重叠',
            ('%4.1f' % ld) if isinstance(ld, (int, float)) else '-'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
