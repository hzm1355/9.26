#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拿 1016 轮真机逐拍流定性"过冲—回修"，不再靠合成模型猜机制。

为什么要这一份：#104 的刹车行程门我先在合成的一阶滞后对象上试过，旧律在同一个模型上
并不翻转（过冲 0.000 rad）—— 说明我对场内 ±0.37 rad 翻转 5 次的机制描述至少缺了一块。
/tmp/pk190_trace.txt 是 /parking/shuttle_status 的逐拍落盘（233 s，起点就是 exit#3 的
align 段），每行同时带"这一拍的指令 w""这一拍的实测航向""这一拍的方向误差"，三列一比
就能直接读出：

  1) 指令与实测角速度反号的拍数 —— 回修指令打在还在原方向转的车上，这是自激的一半；
  2) 指令→实测角速度的互相关滞后 —— BRAKE_LAG 该取多少，用真机数而不是抄 166 轮那句
     "约 0.4 s"；
  3) exit#3 那一段的原始时间线 —— 误差是怎么从 +0.41 走到 -0.37 的，一眼看完。

只读，不改任何状态。跑法（WSL）：python3 tools/pk191_fliptrace316.py [trace 文件]
"""
import math
import re
import sys

LINE = re.compile(
    r'^(?P<t>\d+\.\d+)\s+(?P<m>\S+)\s+phase=(?P<ph>\w+)\s+\d+/\d+\s+'
    r'(?P<leg>\S+)\s+目标=(?P<tgt>\S+)\s+车头=(?P<th>[-+]?\d+\.\d+)\s+'
    r'(?:位置=\((?P<x>[-+]?\d+\.\d+),(?P<y>[-+]?\d+\.\d+)\)\s+'
    r'距离=(?P<d>[-+]?\d+\.\d+)\s+)?'
    r'航向=(?P<yaw>[-+]?\d+\.\d+)\s+指令\s+v=(?P<v>[-+]?\d+\.\d+)'
    r'\s+w=(?P<w>[-+]?\d+\.\d+).*?｜(?P<note>.*)$')
ERR = re.compile(r'差\s*(?P<e>[-+]?\d+\.\d+)\s*rad')
MAXGAP = 0.5           # 窗内才算差分：跨段/停等/落盘的整段空隙都大于这个


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rows(path):
    out = []
    for ln in open(path, encoding='utf-8', errors='replace'):
        m = LINE.match(ln.rstrip('\n'))
        if not m:
            continue
        g = m.groupdict()
        e = ERR.search(g['note'])
        r = {'t': float(g['t']), 'm': g['m'], 'ph': g['ph'], 'leg': g['leg'],
             'th': float(g['th']), 'yaw': float(g['yaw']),
             'v': float(g['v']), 'w': float(g['w']),
             'err': float(e.group('e')) if e else None,
             'note': g['note']}
        if g['x'] is not None:
            r['x'], r['y'] = float(g['x']), float(g['y'])
        # status 是 latched 重发，同一拍的行会成对出现：按 (时刻, 航向, 指令) 去重。
        if out and out[-1]['m'] == r['m'] and abs(r['t'] - out[-1]['t']) < 1e-6 \
                and r['yaw'] == out[-1]['yaw'] and r['w'] == out[-1]['w']:
            continue
        out.append(r)
    return out


def diffs(rs):
    """相邻拍的实测角速度：(后一拍行号, dt, rate)。跨模型/跨窗的差不算。"""
    out = []
    for i in range(1, len(rs)):
        a, b = rs[i - 1], rs[i]
        if b['m'] != a['m']:
            continue
        dt = b['t'] - a['t']
        if not 0.0 < dt <= MAXGAP:
            continue
        out.append((i, dt, wrap(b['yaw'] - a['yaw']) / dt))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pk190_trace.txt'
    rs = rows(path)
    print('解析到 %d 拍（去重后），跨度 %.1f s，模型 %s' % (
        len(rs), rs[-1]['t'] - rs[0]['t'] if rs else 0.0,
        '/'.join(sorted({r['m'] for r in rs}))))
    if len(rs) < 20:
        return 1
    dl = diffs(rs)
    print('可用差分对 %d 组（窗内 <=%.1f s）' % (len(dl), MAXGAP))

    # ── 1) 指令与实测角速度反号 ────────────────────────────────────────────
    conf = [(i, rs[i]['w'], r) for i, _dt, r in dl if abs(rs[i]['w']) >= 0.14
            and abs(r) >= 0.02 and rs[i]['w'] * r < 0.0]
    print('1) 反号拍数 %d / 有效对 %d（%.0f%%）；这些拍里 |指令| 峰 %.2f rad/s、'
          '|实测| 峰 %.2f rad/s' % (
              len(conf), len(dl), 100.0 * len(conf) / max(len(dl), 1),
              max([abs(w) for _, w, _r in conf], default=0.0),
              max([abs(r) for _, _w, r in conf], default=0.0)))

    # ── 2) 互相关：指令 w(t) 与实测 ω(t) 对齐在哪个滞后上 ──────────────────
    print('2) 指令→实测的滞后扫描（同号率最高的那一档就是跟随延迟）：')
    for lag in (0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80):
        hit = tot = 0
        sq = 0.0
        for i, _dt, r in dl:
            j = None
            best = 1e9
            for k in range(max(1, i - 24), i + 1):   # 找 t_i - lag 那一拍
                dd = abs(rs[k]['t'] - (rs[i]['t'] - lag))
                if dd < best:
                    best, j = dd, k
            if best > 0.12 or rs[j]['w'] == 0.0:
                continue
            tot += 1
            if rs[j]['w'] * r > 0.0:
                hit += 1
            sq += (r / rs[j]['w']) ** 2
        if tot:
            print('   +%.2f s：同号率 %5.1f%%（%d/%d）  |ω|/|w| RMS %.2f' % (
                lag, 100.0 * hit / tot, hit, tot, math.sqrt(sq / tot)))

    # ── 3) 每个航点段的误差翻转与幅度 ──────────────────────────────────────
    print('3) 逐航点段（leg 连续段）的误差符号翻转：')
    seg, segs = [], []
    for r in rs:
        if seg and (r['m'] != seg[-1]['m'] or r['leg'] != seg[-1]['leg']):
            segs.append(seg)
            seg = []
        seg.append(r)
    if seg:
        segs.append(seg)
    for s in segs:
        es = [x['err'] for x in s if x['err'] is not None]
        if not es:
            continue
        flips = sum(1 for a, b in zip(es, es[1:])
                    if a * b < 0.0 and abs(b) > 0.15)
        print('   %-12s %-8s %4d 拍  误差 %+.2f→%+.2f  峰值 |%.2f|  '
              '跨 0.15 的翻转 %d 次  |w|峰 %.2f  %s' % (
                  s[0]['leg'], s[0]['ph'], len(s), es[0], es[-1],
                  max(abs(e) for e in es), flips,
                  max(abs(x['w']) for x in s), s[0]['note'][:26]))

    # ── 4) 第一段（现场的 exit#3）原始时间线 ───────────────────────────────
    s0 = segs[0]
    print('4) %s 段逐拍前 45 行：' % s0[0]['leg'])
    print('   %7s %7s %8s %8s %8s %8s  %s' % (
        't-rel', 'err', '航向', '指令w', '实测ω', '位移', 'note 摘'))
    for k, r in enumerate(s0[:45]):
        rate = ''
        if k:
            dt = r['t'] - s0[k - 1]['t']
            if 0.0 < dt <= MAXGAP:
                rate = '%+8.2f' % (wrap(r['yaw'] - s0[k - 1]['yaw']) / dt)
        print('   %7.2f %7s %8.2f %+8.2f %8s %8s  %s' % (
            r['t'] - rs[0]['t'],
            ('%+.2f' % r['err']) if r['err'] is not None else '--',
            r['yaw'], r['w'], rate,
            ('%.2f' % math.hypot(r['x'] - s0[k - 1]['x'], r['y'] - s0[k - 1]['y'])
             ) if k and 'x' in r and 'x' in s0[k - 1] else '',
            r['note'][:30]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
