#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /parking/shuttle_status 的逐拍状态行反解成"误差 / 指令 / 实测角速度"三列，
判一次转向不收敛到底是控制自激还是被接触顶回。

为什么不能只看节点日志：挂起那一行只给最后一个误差数（"翻转 5 次 0.32 rad"），
而这两种病状的**处置完全相反** ——
 * 控制自激：实测角速度跟着指令走（|Δyaw/Δt| ≈ |w|），是 k_ang/阻尼/收尾带没调对，
   改控制律；
 * 被接触顶回：指令持续一个方向、实测角速度却≈0 或反向弹，车体位置还会被挤偏，
   改控制律是白改，得看净空（memory「不动的三态定性」）。
状态行里目标朝向（车头=）与实测朝向（航向=）是同一个口径的两个数，所以误差能在
采样端重算，不必信节点自己打的 note。

用法（WSL）：python3 tools/pk190_flipanalyze316.py /tmp/pk190_trace.txt
"""
import math
import re
import sys

ROW = re.compile(
    r'^(?P<t>[\d.]+) (?P<model>\S+) phase=(?P<phase>\S+) (?P<idx>\d+)/(?P<n>\d+) '
    r'(?P<seg>\S+)#(?P<seq>\d+) 目标=\((?P<tx>[-\d.]+),(?P<ty>[-\d.]+)\) '
    r'车头=(?P<th>[-+\d.]+).*?位置=\((?P<px>[-\d.]+),(?P<py>[-\d.]+)\) 距离=(?P<d>[-\d.]+) '
    r'航向=(?P<yaw>[-+\d.]+) 指令 v=(?P<v>[-\d.]+) w=(?P<w>[-+\d.]+)')
FLOOR = 0.15          # 与节点里的 HUNT_FLIP_FLOOR 同口径


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main():
    rows = []
    for ln in open(sys.argv[1], encoding='utf-8', errors='replace'):
        m = ROW.match(ln)
        if not m:
            continue
        g = m.groupdict()
        r = {k: (float(v) if k not in ('model', 'phase', 'seg') else v)
             for k, v in g.items()}
        r['err'] = wrap(r['th'] - r['yaw'])
        rows.append(r)
    print('解析到 %d 拍（原始行数见文件）' % len(rows))
    if len(rows) < 5:
        print('!! 行数太少：要么车在停驶（挂起后不发行），要么正则没吃下当前措辞。'
              '先看一眼原始行再判，别把"读不到"当"没问题"')
        return 1

    spans = []
    for i in range(1, len(rows)):
        dt = rows[i]['t'] - rows[i - 1]['t']
        if dt > 0.02:
            spans.append(dt)
    med = sorted(spans)[len(spans) // 2] if spans else 0.06
    print('相邻拍间隔：中位 %.3f s / 最大 %.3f s（节点 dt=0.05 s 仿真 ≈ 0.06 s 墙钟）' % (med, max(spans) if spans else 0))
    # 算角速度/位移用的"同一拍序"窗口按**实测中位间隔**取，不写死 0.02：
    # 采样端只有 ~6 Hz 时，写死的窄窗会把每一对都判成"跨段丢拍"跳过，
    # 于是"实测角速度峰值"整列静默变 0.000 —— 那正是"被接触顶回"的假指纹。
    LO, HI = 0.5 * med, 3.0 * med
    print('逐拍窗口 [%.3f, %.3f] s（窗外视为跨段/停车等待，不参与差分）' % (LO, HI))

    # 按 (相, 航点) 分段：翻转计数是**按航点**的，跨段混算会把两段各自的小修读成一次大振荡
    segs, cur = [], None
    for r in rows:
        key = (r['phase'], r['seg'], r['seq'])
        if key != cur:
            cur = key
            segs.append((key, []))
        segs[-1][1].append(r)

    for (phase, seg, idx), rs in segs:
        flips = 0
        side = 0.0
        peak = 0.0
        cmdflip = 0
        cprev = 0.0
        yawrng = 0.0
        driftpos = 0.0
        npair = 0             # 真正参与差分的拍对数：为 0 就说明"实测"那两列是空的，不是车没动
        for i, r in enumerate(rs):
            if abs(r['err']) > peak:
                peak = abs(r['err'])
            s = 1.0 if r['err'] > 0 else -1.0
            if side and s != side and abs(r['err']) > FLOOR:
                flips += 1
            side = s
            if cprev and r['w'] and (r['w'] > 0) != (cprev > 0) and abs(r['w']) > 0.02:
                cmdflip += 1
            cprev = r['w']
            if i:
                dt = r['t'] - rs[i - 1]['t']
                # 两端都要挡：太短是采样端两拍撞在同一毫秒（除零），太长是跨段/
                # 停车等待/丢拍，拿它算角速度会把 12 s 的停留摊成一次慢转。
                if dt <= 0.005 or dt < LO or dt > HI:
                    continue
                npair += 1
                yawrng = max(yawrng, abs(wrap(r['yaw'] - rs[i - 1]['yaw'])) / dt)
                driftpos = max(driftpos, math.hypot(r['px'] - rs[i - 1]['px'],
                                                    r['py'] - rs[i - 1]['py']))
        wabs = [abs(r['w']) for r in rs]
        vabs = [abs(r['v']) for r in rs]
        print('  %-8s %s#%-3s %4d 拍  误差峰值 %.3f  翻转 %d 次  '
              '指令峰值 |w|=%.2f |v|=%.2f  实测角速度峰值 %.3f rad/s  拍间最大位移 %.3f m'
              '（差分 %d 对）' % (
                  phase, seg, idx, len(rs), peak, flips,
                  max(wabs), max(vabs), yawrng, driftpos, npair))
        if flips:
            # 把第一次翻转前后 12 拍摊开，看误差符号与指令符号是不是互相追
            k = None
            side = 0.0
            for i, r in enumerate(rs):
                s = 1.0 if r['err'] > 0 else -1.0
                if side and s != side and abs(r['err']) > FLOOR:
                    k = i
                    break
                side = s
            lo = max(0, k - 6)
            print('      首次翻转前后逐拍（err=目标−实测，w=指令）：')
            for r in rs[lo:k + 7]:
                print('        t=%9.2f err=%+.3f  yaw=%+.3f  th=%+.3f  w=%+.2f  v=%.2f  '
                      'pos=(%.2f,%.2f)' % (r['t'] % 100000, r['err'], r['yaw'], r['th'],
                                           r['w'], r['v'], r['px'], r['py']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
