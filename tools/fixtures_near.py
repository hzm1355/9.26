#!/usr/bin/env python3
"""按 link 级 <pose> 找 world 里压在某个坐标上的碰撞件（含自动生成的 fixture 盒）。

为什么 model 级粗筛不够：sandbox_fixtures_coll 是 tools/gen_fixture_collisions.py
一次性生成的**单个 model**，里面几十个 link 各自带一个 <pose>，model 级 pose 是
(0,0,0)。上一轮按 model 级筛，它整块被"距车 24 m"划走了，等于没查。而 memory
「清理车道侵入的隐形碰撞盒」说得很明确：这个沙盘咬人的从来都是看不见的盒子。

输出按"盒边到点的距离"而不是"中心到点的距离"，因为中心 1.5 m 外但半宽 1.6 m 的盒子
是真会顶住车的；负数表示该点落在盒子内部。
"""
import math
import re
import sys

# 数字必须能匹配单个 "0"：sandbox_fixtures_coll 的 pose 写的是 `-17.000 4.875 6.850 0 0 0`，
# 后三个欧拉角就是光秃秃的 0。早先用 `[-\d.eE+]+(?:\d)` 要求"至少两个字符且以数字结尾"，
# 单字符的 0 直接匹配失败 → 整条 pose 正则不命中 → 我把它当成"这个 collision 没有 pose"
# 按原点 (0,0,0) 算了，于是几十个隐形盒全被划到 20 m 开外，校准点 (-17.0, 4.875) 报
# "0 命中"。这条是用已知存在的盒子校出来的。
NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)'


def main():
    if len(sys.argv) < 4:
        print('用法: fixtures_near.py <world> <x> <y> [余量=1.5]')
        return 2
    path = sys.argv[1]
    cx, cy = float(sys.argv[2]), float(sys.argv[3])
    pad = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5

    with open(path, encoding='utf-8', errors='replace') as f:
        txt = f.read()

    rows = []
    # 逐 link 扫：collision 紧跟在它自己的 <geometry> 之前或之后，这里按 link 块取
    for mm in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S):
        mname, mbody = mm.group(1), mm.group(2)
        # 只认第一个 <link 之前的 pose，那才是 model 级 pose；往后找会抓到 link 的。
        # **没有 model 级 pose 不等于没有碰撞**：sandbox_fixtures_coll 整块就没有
        # <pose>（它的每个 collision 直接写世界坐标），早先版本在这里 continue 掉它，
        # 于是"隐形盒"这一支查了个空 —— 而它恰恰是这个沙盘历史上最会咬人的一类。
        head = re.split(r'<link\b', mbody, 1)[0]
        mp = re.search(r'<pose>\s*(%s)\s+(%s)\s+(%s)\s+(%s)\s+(%s)\s+(%s)'
                       r'\s*</pose>' % ((NUM,) * 6), head)
        if mp:
            mx, my, mth = float(mp.group(1)), float(mp.group(2)), float(mp.group(6))
        else:
            mx = my = mth = 0.0
        for lm in re.finditer(r'<link name="([^"]+)">(.*?)</link>', mbody, re.S):
            lname, lbody = lm.group(1), lm.group(2)
            if '<collision' not in lbody:
                continue
            # 该 link 内每个 collision 自己的 pose；没有 pose 就按 link 原点
            for cm in re.finditer(r'<collision name="([^"]+)">(.*?)</collision>',
                                  lbody, re.S):
                cname, cbody = cm.group(1), cm.group(2)
                cp = re.search(r'<pose>\s*((?:%s\s*){6})</pose>' % NUM, cbody)
                vals = ([float(v) for v in cp.group(1).split()]
                        if cp else [0.0] * 6)
                lx, ly, lth = vals[0], vals[1], vals[5]
                # 局部 -> 世界（模型只转 yaw，link 再转 yaw）
                wx = mx + lx * math.cos(mth) - ly * math.sin(mth)
                wy = my + lx * math.sin(mth) + ly * math.cos(mth)
                th = mth + lth
                box = re.search(r'<box>\s*<size>\s*((?:%s\s*){3})</size>' % NUM,
                                cbody)
                cyl = re.search(r'<cylinder>\s*<radius>\s*(%s)\s*</radius>' % NUM,
                                cbody)
                if box:
                    sx, sy = [float(v) for v in box.group(1).split()][:2]
                    hw, hh = sx / 2.0, sy / 2.0
                    shape = 'box %.2fx%.2f' % (sx, sy)
                elif cyl:
                    hw = hh = float(cyl.group(1))
                    shape = 'cyl r=%.2f' % hw
                else:
                    continue
                # 把点变换到盒子的局部系再算到边距离
                dx, dy = cx - wx, cy - wy
                u = dx * math.cos(-th) - dy * math.sin(-th)
                v = dx * math.sin(-th) + dy * math.cos(-th)
                ox = max(0.0, abs(u) - hw)
                oy = max(0.0, abs(v) - hh)
                gap = math.hypot(ox, oy)
                inside = (abs(u) <= hw and abs(v) <= hh)
                if gap <= pad or inside:
                    rows.append((gap, inside, '%s/%s/%s' % (mname, lname, cname),
                                 wx, wy, th, shape))
    rows.sort()
    if not rows:
        print('  %.1f m 内没有任何 link 级碰撞件 —— 隐形盒这一支也排除了' % pad)
    for gap, inside, key, wx, wy, th, shape in rows:
        print('  %-46s 中心(%7.2f,%7.2f) yaw=%+.2f %-12s 边距 %s%.2f m' % (
            key, wx, wy, th, shape,
            '压在车下 ' if inside else '', gap if not inside else 0.0))
    print('  --- 命中 %d 个 ---' % len(rows))
    return 0


if __name__ == '__main__':
    sys.exit(main())
