#!/usr/bin/env python3
# 车体对车体的接触几何复核（memory「接触几何而非中心距」）。
#
# 为什么单独写这一发：tools/pk177_worldprechk308.py 的净空口径只加载**设施**碰撞件
# （它显式跳过 vehicle 与地板），所以它报"最近件 1.098 m"时，站在南口行车道上的
# car_2 根本没进过判据。而 car_2 从车流退出后一直停在它的 world 出生位
# (12.00,-14.60, yaw=1.5708) —— 正好在复位格 (12.00,-12.80) 的正南 1.8 m，也就是
# car_3 由东转北那个原地转扇形的必经之地。中心距 1.58 m 听着像"还有富余"，
# 但两辆都是 1.6 m 长的箱体，首尾相对时中心距 1.6 m 就是贴着。
#
# 口径（全部点名，方便事后对齐）：
#   * 位姿来源：脚本参数（我手动从 gz model -p 抄进来的实测值），不是 SDF 出生位；
#   * 车体：chassis collision 盒 1.6(x=长) x 0.8(y=宽)，link pose z=+0.25、高 0.4
#     —— 两车的 z 区间都是 [6.699+0.05, 6.699+0.45]，完全重叠，所以 2D SAT 成立；
#   * 旋转瞬时中心 = 车体中心 + 0.40*û(yaw)（diff_drive 绕后轮轴转，memory 里定的
#     AXLE=0.40），车体相对该中心向后 1.2 m、向前 0.4 m、半宽 0.40 m；
#   * 扫掠用角点采样（乐观口径），保守口径另附全圆盘 R=1.265 的对照行。
# 用法：
#   python3 tools/pk180_obb308.py                       # 默认跑本轮的两个病灶姿态
#   python3 tools/pk180_obb308.py x,y,yaw x,y,yaw       # 自定义 A 车首末两个姿态
import math
import sys

L, W = 1.60, 0.80
AXLE = 0.40          # 后轮轴相对车体中心的前移量
Z0, ZH = 0.05, 0.45  # chassis 盒在车体坐标系里的 z 区间（相对地面）


def corners(cx, cy, yaw, length=L, width=W):
    """车体中心 (cx,cy)、朝向 yaw -> 四个角点。"""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    out = []
    for lx, ly in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
        out.append((cx + lx * c - ly * s, cy + lx * s + ly * c))
    return out


def pivot(cx, cy, yaw):
    """瞬时旋转中心：后轮轴 = 车体中心沿车头方向 0.40 m。"""
    return cx + AXLE * math.cos(yaw), cy + AXLE * math.sin(yaw)


def spin_swept_cx(c, yaw):
    """绕轮轴转到 yaw 时，车体中心扫过的圆弧终点（中心轨迹是绕 P 的圆）。"""
    px, py = pivot(c[0], c[1], c[2])
    r = math.hypot(c[0] - px, c[1] - py)
    a0 = math.atan2(c[1] - py, c[0] - px)
    a1 = a0 + (yaw - c[2])
    return px + r * math.cos(a1), py + r * math.sin(a1)


def sat_axes(box):
    e1 = (box[1][0] - box[0][0], box[1][1] - box[0][1])
    e2 = (box[3][0] - box[0][0], box[3][1] - box[0][1])
    axes = []
    for ex, ey in (e1, e2):
        n = math.hypot(ex, ey)
        axes.append((ex / n, ey / n))
    return axes


def proj(box, ax):
    ds = [px * ax[0] + py * ax[1] for px, py in box]
    return min(ds), max(ds)


def obb_gap(a, b):
    """带符号最小分离量：正=净隙（还能挪多少），负=重叠深度（已经顶进去多深）。

    SAT 的标准算法是对每条候选轴算 max(lo1,lo2)-min(hi1,hi2) 再取**最大**值：
    最大的那条就是"最宽松的分离轴"，它的正值就是最小平移距离，负值就是穿透深度。
    （上一版误初始化成 -1e9 配 `ov < worst_overlap`，读出来是 -1e9 的哨兵值，
     看着像"重叠 10 亿米"，其实是哨兵没被替换。）
    """
    sep = -1e9
    for ax in sat_axes(a) + sat_axes(b):
        lo1, hi1 = proj(a, ax)
        lo2, hi2 = proj(b, ax)
        sep = max(sep, max(lo1, lo2) - min(hi1, hi2))
    return sep


def main():
    # 阻塞车（本轮：停在南口车道上的 car_2）
    bx, by, byaw = 12.0047, -14.6125, 1.57375
    blk = corners(bx, by, byaw)
    print('阻塞件：1.60x0.80 箱体 @ (%.4f, %.4f) yaw=%.4f' % (bx, by, byaw))

    if len(sys.argv) == 3:
        a0 = [float(v) for v in sys.argv[1].split(',')]
        a1 = [float(v) for v in sys.argv[2].split(',')]
        cases = [('自定义', a0, a1)]
    else:
        # 病灶姿态（gz model -p 实测）与它要转去的下一个航点朝向
        wedged = (11.9971, -13.0369, 0.6235)     # 探针前 / 挂起时
        moved = (11.9985, -13.0338, 0.6191)      # 挂起后再读的一次
        cases = [('挂起实测姿态', wedged, (wedged[0], wedged[1], 1.5708)),
                 ('探针后再读', moved, (moved[0], moved[1], 1.5708))]

    for name, c, tgt in cases:
        px, py = pivot(*c)
        cur = corners(c[0], c[1], c[2])
        g = obb_gap(cur, blk)
        print('\n=== %s：车体中心 (%.4f, %.4f) yaw=%.4f -> 目标 yaw=%.4f ==='
              % (name, c[0], c[1], c[2], tgt[2]))
        print('  当前位形与阻塞件的最小分离量：%+.3f m（负=已经重叠/顶住）' % g)
        print('  瞬时中心（后轮轴）= (%.4f, %.4f)' % (px, py))

        th0, th1 = c[2], tgt[2]
        ov = 0.25 * (1.0 if th1 >= th0 else -1.0)   # align 相过冲，实测 ~0.25 rad
        tend = th1 + ov
        n = max(2, int(abs(tend - th0) / math.radians(2.5)) + 1)
        hits = []
        min_gap = 1e9
        at = None
        for i in range(n + 1):
            t = th0 + (tend - th0) * i / float(n)
            cx, cy = spin_swept_cx(c, t)
            box = corners(cx, cy, t)
            gp = obb_gap(box, blk)
            if gp < min_gap:
                min_gap, at = gp, (t, cx, cy)
            if gp < 0:
                hits.append((math.degrees(t), -gp))
        print('  扫掠 %d 档（含 %+.2f rad 过冲）：最小分离量 %+.3f m @ yaw=%.2f°'
              % (n + 1, ov, min_gap, math.degrees(at[0])))
        if hits:
            print('  !! 与阻塞件重叠的航向共 %d 档，最深处 %.3f m，从 %.1f° 起'
                  % (len(hits), max(h[1] for h in hits), hits[0][0]))
        else:
            print('  全程无重叠（乐观口径）')

        # 保守口径：绕轮轴的全圆盘 R = hypot(1.2, 0.4)
        R = math.hypot(L - AXLE, W / 2.0)
        d = math.hypot(px - bx, py - by)
        print('  保守口径（全圆盘 R=%.3f 绕轮轴）：轮轴到阻塞件中心 %.3f m，'
              '扣掉阻塞件自身半长 %.2f m 后余量 %+.3f m'
              % (R, d, L / 2.0, d - (R + L / 2.0)))

    print('\n注：以上只算车体 chassis 盒；车牌框（前后各 0.44 宽、x=±0.81）与后视镜'
          '不在判据里，因为它们不拦轮子。')


if __name__ == '__main__':
    main()
