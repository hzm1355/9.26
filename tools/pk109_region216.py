#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫 world：列出落在指定世界矩形里的所有实体部件（含模型名、部件名、几何尺寸）。

为什么要扫：第三个车位要落在原始沙盘那块贴地绿色矩形的露出部分
（x 13.00..16.37 × y −11.48..−7.18）。绿色本身只是网格贴图，但这一带还有
B 类设施等效盒（sandbox_fixtures_coll 的几十个小盒）、岗亭、龙门架、信号灯杆，
只要有一个的投影压在车位上，车开进去就会顶住 —— 而顶住的成因事后极难解释
（memory「不动的三态定性」）。所以选位之前先做一次几何体检。

位姿合成按 SDF 语义：部件世界位姿 = 模型位姿 ∘ 部件位姿（这里只做平移+绕 z 旋转，
world 里所有静态设施的部件都没有 roll/pitch）。盒体尺寸取 <box><size>，
非盒几何（圆柱/球/mesh）打印类型名，人工判断。

用法（WSL）：
  cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
  python3 tools/pk109_region216.py [x0 x1 y0 y1]
"""
import math
import sys
import xml.etree.ElementTree as ET

WORLD = 'src/smart_car_sim/worlds/v2x_city_scene.world'
X0, X1 = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (12.5, 17.5)
Y0, Y1 = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (-12.5, -5.5)


def parse_pose(el):
    if el is None or el.text is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    v = [float(t) for t in el.text.split()]
    while len(v) < 6:
        v.append(0.0)
    return tuple(v)


def compose(p, q):
    """世界 = p ∘ q，p/q 都是 (x,y,z,roll,pitch,yaw)，只精确到 z 旋转。"""
    c, s = math.cos(p[5]), math.sin(p[5])
    return (p[0] + c * q[0] - s * q[1],
            p[1] + s * q[0] + c * q[1],
            p[2] + q[2],
            p[3] + q[3], p[4] + q[4], p[5] + q[5])


def main():
    tree = ET.parse(WORLD)
    hits = []
    for m in tree.getroot().iter('model'):
        mname = m.get('name') or ''
        mp = parse_pose(m.find('pose'))
        for part in list(m.iter('visual')) + list(m.iter('collision')):
            pp = compose(mp, parse_pose(part.find('pose')))
            g = part.find('geometry')
            desc = '?'
            half_w = half_l = 0.0
            if g is not None:
                box = g.find('box')
                cyl = g.find('cylinder')
                sph = g.find('sphere')
                mesh = g.find('mesh')
                if box is not None:
                    sx, sy, sz = [float(t) for t in box.find('size').text.split()]
                    desc = 'box %.2fx%.2fx%.2f' % (sx, sy, sz)
                    half_l, half_w = sx / 2.0, sy / 2.0
                elif cyl is not None:
                    r = float(cyl.find('radius').text)
                    desc = 'cylinder r=%.2f' % r
                    half_l = half_w = r
                elif sph is not None:
                    r = float(sph.find('radius').text)
                    desc = 'sphere r=%.2f' % r
                    half_l = half_w = r
                elif mesh is not None:
                    u = mesh.find('uri')
                    desc = 'mesh %s' % (u.text if u is not None else '?')
                    half_l = half_w = 1.0
            # 旋转后的外接正方盒：保守取 max(half_l, half_w)
            rr = max(half_l, half_w)
            if pp[0] + rr < X0 or pp[0] - rr > X1 or pp[1] + rr < Y0 or pp[1] - rr > Y1:
                continue
            hits.append((mname, part.get('name') or '?', pp, desc))

    print('区域 x %.2f..%.2f × y %.2f..%.2f，命中部件 %d 个' % (X0, X1, Y0, Y1, len(hits)))
    for mname, pname, pp, desc in sorted(hits):
        print('  %-28s %-22s (%7.2f,%7.2f) z=%5.2f yaw=%6.3f  %s' % (
            mname[:28], pname[:22], pp[0], pp[1], pp[2], pp[5], desc))
    return 0


if __name__ == '__main__':
    sys.exit(main())
