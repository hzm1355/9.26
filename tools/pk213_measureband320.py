#!/usr/bin/env python3
# 只读几何量带：车队全部堆在 y≈-18.9，而 yaml 南环中心线写的是 y=-16.8，横偏 2.1 m。
# 两种解释的处置完全相反，必须先用世界几何分清：
#   甲) 配置中心线陈旧 —— 南侧真走廊被停车场/路缘挤到 y≈-18.95，那么"恢复正常航线"
#       就是把路线改到实测走廊（同 #103 东腿外移的先例），护栏参数一律回到代码原值；
#   乙) 中心线没错、车自己过冲偏出 —— 那就该动转向（#104 那一族），改配置是掩盖病灶。
# 口径（都是踩过的坑）：
#   * 位姿叠加：collision 的 pose 相对父 link，link 相对父 model，model 相对 world，
#     只叠最外层会把架空件读成地面件；这里按完整父链累乘。
#   * 只看对地面有拦截意义的件：z 区间要和车体（z 6.70..7.10 那一层）重叠才算拦路，
#     灯杆/标志牌那种只到 6.7 以下的要区分开。
#   * mesh 碰撞没有解析出的 AABB，单独列成"未量化"而不是当它不存在。
#   * 量的是文件世代；运行中的 gzserver 于 14.6 h 前加载同一份文件，之后 #126 只改了
#     沥青颜色（生成器侧），所以几何口径一致，但报告里要显式说明。
import os
import sys
import xml.etree.ElementTree as ET

WORLD = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world'

# 查询带：南环配置中心线 y=-16.8，实测车队贴线 y=-18.95。取 -21..-13 覆盖两者。
QUERY = (-21.0, -13.0)
# 拦路判据：件要"立在台面上"且和车体层相交。台面本身（room_33 顶面 z=6.70）不是拦路件，
# 上一版用 z>=6.60 把台面算进去了，切片直接读出"每一行 100% 被拦"——那是假红。
CAR_Z = (6.74, 7.15)


def parse_vec(txt, n, default):
    if txt is None or not str(txt).strip():
        return list(default)
    parts = str(txt).replace(',', ' ').split()
    vals = [float(p) for p in parts[:n]]
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals


def quat_to_rot(roll, pitch, yaw):
    cr, sr = __import__('math').cos(roll), __import__('math').sin(roll)
    cp, sp = __import__('math').cos(pitch), __import__('math').sin(pitch)
    cy, sy = __import__('math').cos(yaw), __import__('math').sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def apply(M, v):
    return [M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
            M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
            M[6] * v[0] + M[7] * v[1] + M[8] * v[2]]


def mul(A, B):
    # 3x3 旋转左乘 3x3
    out = [0.0] * 9
    for i in range(3):
        for j in range(3):
            out[3 * i + j] = sum(A[3 * i + k] * B[3 * k + j] for k in range(3))
    return out


def identity():
    return [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.0, 0.0, 0.0]


def pose_of(elem):
    p = elem.find('pose') if elem is not None else None
    xyz = parse_vec(p.text, 3, [0, 0, 0]) if p is not None else [0, 0, 0]
    rpy = parse_vec(p.text, 6, [0, 0, 0, 0, 0, 0])[3:] if p is not None else [0, 0, 0]
    return quat_to_rot(*rpy), xyz


def walk(node, R, T, depth, path, out):
    """R/T 是 node 父级累乘到 world 的旋转/平移；node 自身 pose 再叠一层。"""
    r, t = pose_of(node)
    Rw = mul(R, r)
    lt = apply(R, t)                      # 本地平移要先被父级旋转带走
    Tw = [lt[0] + T[0], lt[1] + T[1], lt[2] + T[2]]
    name = node.get('name') or '?'
    me = '%s/%s' % (path, name)
    # collision 是 link 的直接子节点，不会自己再被 walk 进去（上一版就是漏在这里：
    # 只在 node.tag=='collision' 时处理，而 recursion 只进 model/link，结果恒 0 命中）。
    for col in node.findall('collision'):
        cname = '%s/%s' % (me, col.get('name') or '?')
        cr, ct = pose_of(col)
        Rc = mul(Rw, cr)
        lc = apply(Rw, ct)
        Tc = [lc[0] + Tw[0], lc[1] + Tw[1], lc[2] + Tw[2]]
        for g in col.findall('geometry'):
            for shape in g:
                if shape.tag == 'box':
                    size = shape.find('size')
                    sz = parse_vec(size.text if size is not None else None, 3, [0, 0, 0])
                    hs = [abs(sz[0]) / 2.0, abs(sz[1]) / 2.0, abs(sz[2]) / 2.0]
                    corners = []
                    for sx in (-1, 1):
                        for sy in (-1, 1):
                            for sz2 in (-1, 1):
                                corners.append(apply(Rc, [sx * hs[0], sy * hs[1], sz2 * hs[2]]))
                    xs = [c[0] + Tc[0] for c in corners]
                    ys = [c[1] + Tc[1] for c in corners]
                    zs = [c[2] + Tc[2] for c in corners]
                    out.append((cname, min(xs), max(xs), min(ys), max(ys),
                                min(zs), max(zs), 'box'))
                elif shape.tag in ('mesh', 'cylinder', 'sphere'):
                    out.append((cname, None, None, None, None, None, None, shape.tag))
    for child in node:
        if child.tag in ('model', 'link'):
            walk(child, Rw, Tw, depth + 1, me, out)
        # include 里的 <uri> 已由 gzserver 展开，文件里看不到 —— 下面单独统计


def main():
    if not os.path.exists(WORLD):
        print('读不到 world：%s' % WORLD)
        return 3
    tree = ET.parse(WORLD)
    root = tree.getroot()
    # 前置闸：标签集要真认出来。Gazebo Classic 的层级是 sdf/world/model，
    # 上一版我按 Unity 的 <scene> 找，命中 0 个却照样报"0 个拦路件"——那是假绿。
    scopes = root.findall('world') + root.findall('scene')
    nmodels = sum(len(sc.findall('model')) for sc in scopes)
    print('顶层标签=%s，world/scene 节点=%d，其下直接 model=%d'
          % (root.tag, len(scopes), nmodels))
    if not scopes or nmodels == 0:
        print('PRECHECK FAIL：没认出任何 model，下面的计数一律不作数')
        return 4
    boxes, unq, includes = [], [], 0
    for sc in scopes:
        for m in sc.findall('model'):
            walk(m, identity()[0], identity()[1], 0, '', boxes)
        includes += len(sc.findall('include'))
    if not boxes:
        print('PRECHECK FAIL：model 认出来了但一个 collision 都没解析到，口径仍有问题')
        return 4
    print('world=%s' % WORLD)
    print('解析出的 collision 项 %d（其中 box %d / 未量化 %d），include %d 个'
          '（include 的内联几何不在这份文件里，需要另行展开）' % (
              len(boxes), sum(1 for b in boxes if b[7] == 'box'),
              sum(1 for b in boxes if b[7] != 'box'), includes))

    band = [b for b in boxes if b[7] == 'box'
            and b[3] <= QUERY[1] and b[4] >= QUERY[0]]
    block = [b for b in band if b[5] < CAR_Z[1] and b[6] > CAR_Z[0]]
    print('')
    print('=== 落在 y %.1f..%.1f 带内、且 z 与车体层 %.2f..%.2f 重叠的拦路件 %d 个 ==='
          % (QUERY[0], QUERY[1], CAR_Z[0], CAR_Z[1], len(block)))
    for me, x0, x1, y0, y1, z0, z1, _ in sorted(block, key=lambda b: (b[3], b[1])):
        print('  %-52s x %7.2f..%7.2f  y %7.2f..%7.2f  z %5.2f..%5.2f'
              % (me[-52:], x0, x1, y0, y1, z0, z1))

    print('')
    print('=== 按 y 切片：南环这一条上，配置线 -16.8 与实测线 -18.95 谁被占 ===')
    for yc in (-15.6, -16.8, -17.9, -18.95, -20.0):
        hit = [b for b in block if b[3] - 0.05 <= yc <= b[4] + 0.05]
        segs = sorted((b[1], b[2]) for b in hit)
        merged = []
        for a, c in segs:
            if merged and a <= merged[-1][1] + 0.3:
                merged[-1] = (merged[-1][0], max(merged[-1][1], c))
            else:
                merged.append((a, c))
        cov = sum(c - a for a, c in merged)
        print('  y=%6.2f 被拦 x 区间 %d 段共 %.2f m，占 x -25..25 的 %.0f%%  %s'
              % (yc, len(merged), cov, 100.0 * cov / 50.0,
                 ['%6.2f..%6.2f' % (a, c) for a, c in merged][:8]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
