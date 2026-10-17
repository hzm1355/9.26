#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""改完 world 的启动前体检：XML 是否合法 + c130 是否真的不在了 + 停车场南口那格
按接触几何还剩哪些拦路件。

为什么要单独一个脚本：world 里这段等效盒是 tools/gen_fixture_collisions.py 自动生成的
（BEGIN/END 注释包起来、写着"勿手改"），这次按用户授权手删了一条 collision。手删的
风险有两个：注释里出现连续两个减号会让 XML 解析失败（gzserver 直接拒载整个 world），
以及删完少了东西却没人核对。这里两样一起查，跑一次几秒。

用法（WSL）：python3 tools/pk177_worldprechk308.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')

# 复位原地转那一格：航点 (6.80,-12.80) 车头朝南，瞬时中心在轮轴（前方 axle 0.40）
# 可被 --spin=x,y,th0,th1 覆盖（拿来拿已通过的转点做校准）
SPIN = [6.80, -12.80, -math.pi / 2, 0.0]
AXLE, BL, BW = 0.40, 1.60, 0.80
OVERSHOOT = 0.25


def corners(x, y, th):
    """车体四角（底盘原点 x,y、车头朝向 th）。"""
    out = []
    for dx, dy in ((BL / 2, BW / 2), (BL / 2, -BW / 2),
                   (-BL / 2, BW / 2), (-BL / 2, -BW / 2)):
        out.append((x + dx * math.cos(th) - dy * math.sin(th),
                    y + dx * math.sin(th) + dy * math.cos(th)))
    return out


def main():
    for a in sys.argv[1:]:
        if a.startswith('--spin='):
            # 负数一律写成 "--spin=..." 的等号形式：空格形式会被当成选项（memory
            # 「argparse 负数带逗号会当选项」）。四个数依次是 x,y,起始朝向,结束朝向（弧度）。
            v = [float(t) for t in a.split('=', 1)[1].split(',')]
            SPIN[:] = v
    with open(W, encoding='utf-8', errors='replace') as fh:
        txt = fh.read()
    try:
        tree = ET.parse(W)
    except ET.ParseError as e:
        print('!! XML 解析失败：gzserver 会拒载这个 world —— %s' % e)
        return 1
    print('XML 合法：根 %s，element %d 个' % (tree.getroot().tag, len(list(tree.iter()))))

    hits = [i + 1 for i, ln in enumerate(txt.splitlines())
            if 'name="c130"' in ln and '<collision' in ln]
    print('c130 的 collision 行：%s' % (hits if hits else '0 处（已删除）'))

    # 逐 link 取碰撞盒，位姿口径 = 模型 pose + link pose + collision pose
    # （只叠模型位姿会把架空件读成地面件，见 memory「净空读数先点名位姿叠加口径」）
    worst = []
    for model in tree.getroot().findall('world/model'):
        mname = model.get('name')
        if mname in ('room_33', 'room_20', 'room_12', 'ground_plane') \
                or mname.startswith('car_'):
            continue
        mv = [0.0] * 6
        if model.find('pose') is not None:
            mv = [float(v) for v in model.find('pose').text.split()]
        for link in model.findall('link'):
            lv = [0.0] * 6
            lh = list(link)
            if lh and lh[0].tag == 'pose':
                lv = [float(v) for v in lh[0].text.split()]
            for col in link.findall('collision'):
                cv = [0.0] * 6
                if col.find('pose') is not None:
                    cv = [float(v) for v in col.find('pose').text.split()]
                g = col.find('geometry')
                b = g.find('box') if g is not None else None
                cyl = g.find('cylinder') if g is not None else None
                if b is not None:
                    sx, sy, sz = [float(v) for v in b.find('size').text.split()]
                    hx, hy = sx / 2.0, sy / 2.0
                elif cyl is not None:
                    hx = hy = float(cyl.find('radius').text)
                    sz = float(cyl.find('length').text)
                else:
                    continue
                z = mv[2] + lv[2] + cv[2]
                if z - sz / 2.0 > 6.70 + 0.30:      # 架空件，车从下面过
                    continue
                if z - sz / 2.0 <= 6.70 - 0.05 and z + sz / 2.0 <= 6.70 + 0.06:
                    continue                        # 贴地 slab，不算拦路件
                th = mv[5] + lv[5] + cv[5]
                wx = mv[0] + lv[0] + cv[0]
                wy = mv[1] + lv[1] + cv[1]
                worst.append(('%s/%s/%s' % (mname, link.get('name'), col.get('name')),
                              wx, wy, th, hx, hy))
    print('参与判定的碰撞盒 %d 个' % len(worst))

    # 真扫掠弧：绕轮轴从 SPIN[2] 转到 SPIN[3]，每 5° 取一次；末端多转 OVERSHOOT —— 现场
    # 实测 align 相会冲过目标朝向 0.25 rad（turn_cmd 没有制动前馈），净空要按冲过头
    # 那一拍算，不然又是"贴着墙画圈"。
    # 口径：只用车体**四角**采样到旋转矩形的边距，是乐观近似（盒角戳进车体边中段
    # 这种情形测不到）；这里只用来核对删完 c130 后最近的拦路件还剩谁，权威判据在
    # tools/pk175_spinclr308.py 的整圆口径（更保守）。
    x, y, th0, th1 = SPIN
    ov = OVERSHOOT * (1.0 if th1 >= th0 else -1.0)
    px, py = x + AXLE * math.cos(th0), y + AXLE * math.sin(th0)
    print('轮轴（瞬时中心）= (%.2f, %.2f)' % (px, py))
    rows = []
    tend = th1 + ov
    n = max(2, int(abs(tend - th0) / math.radians(5)) + 1)
    for i in range(n + 1):
        # 端点必须真取到 th1+ov：只按 5° 定步长会停在 10°，把过冲末端那一拍
        # （车尾角压得最深的那一拍）漏掉。
        t = th0 + (tend - th0) * i / float(n)
        for cx, cy in corners(px - AXLE * math.cos(t), py - AXLE * math.sin(t), t):
            rows.append((cx, cy))
    print('扫掠带范围 x %.2f..%.2f / y %.2f..%.2f' % (
        min(p[0] for p in rows), max(p[0] for p in rows),
        min(p[1] for p in rows), max(p[1] for p in rows)))

    def gap(bx):
        _, wx, wy, th, hx, hy = bx
        best = 1e9
        for cx, cy in rows:
            dx, dy = cx - wx, cy - wy
            u = dx * math.cos(-th) - dy * math.sin(-th)
            v = dx * math.sin(-th) + dy * math.cos(-th)
            ox = max(0.0, abs(u) - hx)
            oy = max(0.0, abs(v) - hy)
            g = math.hypot(ox, oy)
            if g < best:
                best = g
        return best

    near = sorted(((gap(b), b[0]) for b in worst), key=lambda z: z[0])[:6]
    print('离扫掠带最近的 6 个盒（间隙 m，负=吃进）：')
    for g, name in near:
        print('  %7.3f  %s' % (g, name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
