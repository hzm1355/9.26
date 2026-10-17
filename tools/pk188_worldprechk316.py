#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整栈重启前的 world 断言：只查"这次到底改没改对、改的是不是运行期那一份"。

为什么要有这一发：gzserver 只在启动时读 SDF，线圈几何（loop_center/loop_size）是
world 里插件的参数，不是 yaml 的运行时参数 —— yaml 改了而 world 没重新生成，重启后
拿到的还是旧占用框，而现场看起来"一切正常"，直到又出现提前落杆才发现白跑一轮 20 分钟。
四行断言各管一件事：
  1) XML 合法 —— 注释里出现连续两个减号会让 gzserver 拒载**整个** world；
  2) 入口线圈 = 12.00/-10.90 + 1.30/1.10，出口线圈 = 6.80/-10.75 + 1.30/0.80（刻意不动，
     扩它会吃掉 reset#0 那一格，多演一次防砸重抬）；
  3) car_3 出生位 = (12.00,-13.95) —— 这条同时是"重启后拿位姿反证加载文件"的基准；
  4) c130 的 collision 行 0 处、c131 还在 —— 用户只授权删前者（2026-10-16）。

用法（WSL）：python3 tools/pk188_worldprechk316.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')

# (期望文本, 说明)
EXPECT = [
    ('<loop_center>12.0000 -10.9000 6.7000</loop_center>', '入口线圈中心已南移'),
    ('<loop_size>1.3000 1.1000 0.1</loop_size>', '入口线圈 y 向已扩到 1.10'),
    ('<loop_center>6.8000 -10.7500 6.7000</loop_center>', '出口线圈保持原标定（刻意不动）'),
    ('<loop_size>1.3000 0.8000 0.1</loop_size>', '出口线圈 y 向保持 0.80'),
    ('<pose>12.0000 -13.9500 7.0000 0 0 1.5708</pose>', 'car_3 出生位已随等待位南移'),
]

fails = []


def main() -> int:
    try:
        ET.parse(W)
    except ET.ParseError as e:
        print('!! XML 解析失败：gzserver 会拒载这个 world —— %s' % e)
        return 1
    print('1) XML 合法（gzserver 能载）')

    with open(W, encoding='utf-8', errors='replace') as fh:
        lines = fh.read().splitlines()
    for want, why in EXPECT:
        n = sum(1 for ln in lines if ln.strip() == want)
        print('   %s %s（%d 处）%s' % ('OK ' if n else 'FAIL', why, n, want))
        if not n:
            fails.append(why)

    c130 = [i + 1 for i, ln in enumerate(lines) if 'name="c130"' in ln and '<collision' in ln]
    c131 = [i + 1 for i, ln in enumerate(lines) if 'name="c131"' in ln and '<collision' in ln]
    print('4) c130 collision %d 处（要求 0，已授权删除）/ c131 collision %d 处（要求 1，未授权不动）'
          % (len(c130), len(c131)))
    if len(c130) != 0:
        fails.append('c130 又活了（生成器排除登记失效）')
    if len(c131) != 1:
        fails.append('c131 不见了或重复')

    # 两份副本的口径：运行期 gzserver 用 cwd + 相对路径加载 D 盘那份；install 侧是
    # /home/hzm/v2x_ws/src 的符号链接，不是本次改动目标（memory「先确认运行期加载的是
    # 哪份文件副本」）。这里只报不判，交给调用方看 pid/cwd。
    hy = 0.5 * (1.60 + 0.80) + 0.5 * 1.10      # 半边长 = (车长+车宽)/2 + 半线圈 y
    for ln in lines:
        if '<loop_center>12.0000 -10.9000' in ln:
            print('2) 入口占用框 y %.2f..%.2f（核验位 -12.21 距南缘 %.2f m，须 > 到位残差 0.12）'
                  % (-10.90 - hy, -10.90 + hy, abs(-12.21 - (-10.90 - hy))))
            break

    # 5) 六台的出生格两两不得车体重叠 —— 这一条是被 2026-10-16 那次真实事故逼出来的：
    #    把 car_3 的等待位 (12.00,-13.95) 写进 world 出生位时，没注意到 car_2 的出生格
    #    还留在旧的停车场排队位 (12.00,-14.60)，两格只差 0.65 m、同向朝北、车长 1.60 m，
    #    于是 gzserver 一加载就把两台模型一起弹开：car_3 被甩到 (15.71,-44.06) 掉出台面
    #    落到 z≈0 的地面层，car_2 甩到 (20.81,-16.64)。现场看起来像"重启完车就飞了"，
    #    实际是出生格互穿 —— 手工摆位永远测不到它，因为摆位时另一台早开走了。
    #    判据用 AABB 相交（同向 90°/0° 时它就等于车体贴脸），留 0.10 m 余量。
    tree = ET.parse(W)
    born = []
    for model in tree.getroot().findall('world/model'):
        nm = model.get('name') or ''
        if not nm.startswith('car_'):
            continue
        pv = model.find('pose')
        if pv is None:
            print('   FAIL %s 没有 <pose>，出生位无从判起' % nm)
            fails.append('%s 无 pose' % nm)
            continue
        v = [float(t) for t in pv.text.split()]
        th = v[5] if len(v) >= 6 else 0.0
        ex = abs(0.80 * math.cos(th)) + abs(0.40 * math.sin(th))
        ey = abs(0.80 * math.sin(th)) + abs(0.40 * math.cos(th))
        born.append((nm, v[0], v[1], v[0] - ex, v[0] + ex, v[1] - ey, v[1] + ey))
    born.sort()
    print('5) 出生格 %s' % '、'.join('%s(%.2f,%.2f)' % (b[0], b[1], b[2]) for b in born))
    bad = []
    for i in range(len(born)):
        for j in range(i + 1, len(born)):
            a, b = born[i], born[j]
            if a[3] - 0.10 < b[4] and b[3] - 0.10 < a[4] and a[5] - 0.10 < b[6] and b[5] - 0.10 < a[6]:
                bad.append('%s/%s' % (a[0], b[0]))
    if bad:
        print('   FAIL 出生格车体互穿：%s —— 加载瞬间必弹射，绝不重启' % '、'.join(bad))
        fails.append('出生格互穿')
    else:
        print('   OK  任意两台出生格的车体不相交（含 0.10 m 余量）')

    if fails:
        print('结论：不通过 —— %s' % '；'.join(fails))
        return 1
    print('结论：world 与本次方案甲一致，可以整栈重启')
    return 0


if __name__ == '__main__':
    sys.exit(main())
