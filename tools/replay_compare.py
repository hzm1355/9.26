#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回放对照：把回放期间探针实际收到的灯态相变/车辆位姿，
与录制文件里从 t=0 起的同一段时间轴逐条比对，证明回放"完整复现"。
录制器同时订阅 state / pedestrian / state_detail 三个话题，
因此事件流里有大量占位空串（vehicle_phase='' 或 pedestrian_phase=''），
必须先滤掉再取相变，否则会把占位当成灯色跳变。
用法：python3 replay_compare.py <录制json> <观测json> <起播相对探针零点秒数> [窗口]
"""
import json
import math
import sys
from bisect import bisect_left

REC = json.load(open(sys.argv[1], encoding='utf-8'))
OBS = json.load(open(sys.argv[2], encoding='utf-8'))
OFF = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
WINDOW = float(sys.argv[4]) if len(sys.argv) > 4 else OBS['duration'] - OFF - 1.0
GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
illegal = {('GREEN', 'RED'), ('RED', 'YELLOW'), ('YELLOW', 'GREEN')}

print(f'对照窗口 = 回放起播后 0~{WINDOW:.1f}s（1.0x，从录制 t=0 起播，循环）')


def rec_edges(gid, key):
    """录制流中该组的真实相变（忽略占位空串与重复灯色）"""
    out, last = [], None
    for e in REC['events']:
        if e.get('type') == 'vehicle' or e.get('group') != gid:
            continue
        v = e.get(key)
        if not v or v == last:
            continue
        last = v
        out.append([e['t'], v])
    return out


def durations(edges, window):
    kept = [x for x in edges if x[0] <= window]
    return [(ph, (kept[i + 1][0] - t) if i + 1 < len(kept) else None)
            for i, (t, ph) in enumerate(kept)]


def fmt(durs):
    return ' -> '.join(f'{ph}{"" if d is None else f" {d:.1f}s"}' for ph, d in durs)


def illegal_count(durs):
    ph = [d[0] for d in durs]
    return sum(1 for i in range(1, len(ph)) if (ph[i - 1], ph[i]) in illegal)


print('=== A) 机动车灯态：录制相变序列 vs 回放实测序列（逐组并排）===')
tot, bad, worst = 0, 0, 0.0
for gid in GROUPS:
    rd = durations(rec_edges(gid, 'vehicle_phase'), WINDOW)
    ob = sorted([t - OFF, ph] for t, g, ph in OBS['light'] if g == gid and 0.0 <= t - OFF <= WINDOW)
    od = [(ph, (ob[i + 1][0] - ob[i][0]) if i + 1 < len(ob) else None) for i, (t, ph) in enumerate(ob)]
    n = min(len(rd), len(od))
    mis = sum(1 for i in range(n) if rd[i][0] != od[i][0])
    dd = [abs(rd[i][1] - od[i][1]) for i in range(n - 1)
          if rd[i][1] is not None and od[i][1] is not None]
    tot += n
    bad += mis + illegal_count(od)
    worst = max(worst, max(dd) if dd else 0.0)
    print(f'  {gid}: 相变数 录制={len(rd)} 实测={len(od)} 灯色顺序不符={mis} '
          f'非法相变={illegal_count(od)} 单段时长最大差={max(dd) if dd else 0:.2f}s')
    print(f'        录制 {fmt(rd)}')
    print(f'        实测 {fmt(od)}')
print(f'  小结：对齐相变 {tot} 条，异常 {bad} 条，各段时长最大偏差 {worst:.2f}s')

print('=== B) 行人灯态：录制相变序列 vs 回放实测序列 ===')
for gid in ['J1_ns', 'J2_ew', 'J3_ew']:
    rd = durations(rec_edges(gid, 'pedestrian_phase'), WINDOW)
    ob = sorted([t - OFF, ph] for t, g, ph in OBS['pedestrian'] if g == gid and 0.0 <= t - OFF <= WINDOW)
    od = [(ph, (ob[i + 1][0] - ob[i][0]) if i + 1 < len(ob) else None) for i, (t, ph) in enumerate(ob)]
    print(f'  {gid} 录制 {fmt(rd)}')
    print(f'  {" " * len(gid)} 实测 {fmt(od)}')

print('=== C) 车辆位姿复现误差（先自动对齐时间轴，再最近时间戳配对）===')
veh = {}
for e in REC['events']:
    if e.get('type') == 'vehicle':
        veh.setdefault(e['name'], []).append((e['t'], e['x'], e['y']))
for k in veh:
    veh[k].sort()
ts_of = {k: [a[0] for a in v] for k, v in veh.items()}


def nearest(name, t):
    """采样序列已按时间排序：用二分取最近邻，避免 41 次平移搜索退化成 O(n^2)"""
    ts = ts_of[name]
    i = bisect_left(ts, t)
    cands = [j for j in (i - 1, i, i + 1) if 0 <= j < len(ts)]
    return min(cands, key=lambda j: abs(ts[j] - t))


frames = [(t - OFF, snap) for t, snap in OBS['poses'] if 1.0 < t - OFF <= WINDOW]


def mean_err(off_extra):
    tot, cnt = 0.0, 0
    for name in veh:
        s = veh[name]
        for t, snap in frames:
            if name not in snap:
                continue
            j = nearest(name, t + off_extra)
            if abs(ts_of[name][j] - (t + off_extra)) > 0.30:
                continue
            tot += math.hypot(snap[name][0] - s[j][1], snap[name][1] - s[j][2])
            cnt += 1
    return (tot / cnt if cnt else 9e9), cnt


cands = [(x / 20.0,) + mean_err(x / 20.0) for x in range(-20, 21)]
extra, merr, cnt = min(cands, key=lambda c: c[1])
print(f'  时间轴自动对齐：额外平移 {extra:+.2f}s（起播服务往返延迟），'
      f'有效配对 {cnt} 组，对齐后平均误差 {merr:.2f} m')
allerr = []
for name in sorted(veh):
    s = veh[name]
    errs = []
    for t, snap in frames:
        if name not in snap:
            continue
        j = nearest(name, t + extra)
        if abs(ts_of[name][j] - (t + extra)) > 0.30:
            continue
        x, y, _ = snap[name]
        errs.append((math.hypot(x - s[j][1], y - s[j][2]), t, (x, y), (s[j][1], s[j][2])))
    if not errs:
        continue
    d = sorted(e[0] for e in errs)
    p95 = d[min(len(d) - 1, int(0.95 * len(d)))]
    w = max(errs, key=lambda e: e[0])
    allerr.extend([e[0] for e in errs])
    print(f'  {name:6s} 样本={len(d)} 平均={sum(d)/len(d):.2f} m 中位={d[len(d)//2]:.2f} m '
          f'P95={p95:.2f} m 最大={w[0]:.2f} m(t={w[1]:.1f}s 回放({w[2][0]:.1f},{w[2][1]:.1f}) '
          f'录制({w[3][0]:.1f},{w[3][1]:.1f}))')
d = sorted(allerr)
print(f'  合计 {len(d)} 个位姿对：平均 {sum(d)/len(d):.2f} m，'
      f'中位 {d[len(d)//2]:.2f} m，>1 m 的有 {sum(1 for x in d if x > 1.0)} 个'
      f'（录制车辆位姿采样周期 0.5 s，量化误差本身≈0.25 m + 车速×时间残差）')

print('=== D) 起播归位瞬态（t<=1s，回放把车从"停止时位置"搬回"录制起点"）===')
t0snap = next((snap for t, snap in OBS['poses'] if t - OFF <= 1.0), None)
if t0snap:
    for name in sorted(veh):
        s = veh[name]
        x, y, _ = t0snap[name]
        print(f'  {name:6s} 起播前=({x:6.2f},{y:6.2f}) 录制起点=({s[0][1]:6.2f},{s[0][2]:6.2f}) '
              f'需搬运 {math.hypot(x - s[0][1], y - s[0][2]):.1f} m')

print('=== E) 录制文件结构完整性 ===')
m = REC['meta']
light = [e for e in REC['events'] if e.get('type') != 'vehicle']
modes = sorted({e.get('control_mode') for e in light if e.get('control_mode')})
print(f'  时长={m["record_duration"]:.1f}s 事件={m["event_count"]} 信号组={m["groups"]}')
print(f'  车辆={m.get("vehicles")} 灯态事件={len(light)} 位姿样本={m["event_count"]-len(light)}')
print(f'  录制内含控制模式={modes}')
grp_detail = {}
for e in light:
    if e.get('control_mode'):
        grp_detail.setdefault(e['control_mode'], set()).add(e['group'])
for k, v in sorted(grp_detail.items()):
    print(f'    {k:9s} 覆盖 {len(v)} 组：{sorted(v)}')
