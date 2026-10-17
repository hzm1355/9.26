#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闸机动作日志的时序对账（任务五条目 3 的字幕证据）。

yaml 里写死了三个时间常数：hold_time_s=1.5（开到位后保持全开）、
clearance_delay_s=0.5（地感释放后再落杆的确认延迟）、以及单程理论耗时
open_angle/velocity≈0.79 s。它们到底有没有被执行，只能从动作日志的相邻时间差里量：
  抬杆到位 → 下发落杆   应当 ≥ hold_time_s（早了就说明"保持全开"没生效，
                        多出来的是车从核验位爬过杆下的时间）
  下发落杆 → 落杆到位    应当 ≈ 单程理论耗时，且 ≤ lower_timeout_s
  每一条的 elapsed_s     是插件自己按关节积分报的，与上面两行互相核对
面板上那些数字是给眼睛看的，这里给的是能对账的数。读数不足时明确说"还没收尾/无从算起"，
不拿半截数据算差值。

用法：
  pk_action_stat.py --gate gate_in
  pk_action_stat.py --gate gate_out --tail 8
"""
import argparse
import json
import os
import sys

import yaml

YAML = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/parking_zones.yaml'
ACTS = '/tmp/v2x_parking/barrier_actions.jsonl'


def read_jsonl(path):
    if not os.path.exists(path):
        return None
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--zones', default=YAML)
    ap.add_argument('--actions', default=ACTS)
    ap.add_argument('--gate', default='gate_in')
    ap.add_argument('--tail', type=int, default=10)
    a = ap.parse_args()

    pk = yaml.safe_load(open(a.zones, encoding='utf-8'))['parking']
    bar = pk['barrier']
    hold = float(bar['hold_time_s'])
    cdel = float(bar['clearance_delay_s'])
    theory = float(bar['open_angle']) / float(bar['velocity'])
    print('设定 hold=%.2f s｜clearance=%.2f s｜单程理论 open_angle/velocity=%.2f s'
          '｜抬/落超时 %.1f/%.1f s' % (
              hold, cdel, theory, float(bar['raise_timeout_s']),
              float(bar['lower_timeout_s'])))

    acts = read_jsonl(a.actions)
    if acts is None:
        print('!! 没有 %s：停车场记录器没起来' % a.actions)
        return 1
    mine = [r for r in acts if r.get('gate_id') == a.gate]
    print('%s 动作 %d 条（全场 %d 条），列最近 %d 条：' % (
        a.gate, len(mine), len(acts), min(a.tail, len(mine))))
    print('  仿真秒     动作        令源   从°→到°    耗时  到位  故障        备注')
    for r in mine[-a.tail:]:
        print('  %9.2f %-10s %-5s %5.1f→%-5.1f %5.2f %-4s %-11s %s' % (
            float(r.get('sim_ts', 0.0)), r.get('action_name', '?'),
            r.get('source_name', '?'), float(r.get('from_rad', 0)) * 57.29578,
            float(r.get('to_rad', 0)) * 57.29578, float(r.get('elapsed_s', 0.0)),
            '是' if r.get('reached') else '否', r.get('fault_name', 'NONE'),
            (r.get('note') or '')[:44]))
    if not mine:
        print('  （这道闸一条动作都没有）')
        return 0

    # 动作名是记录器写死的中文（ACT_NAME 表），这里按名取，不做模糊匹配：
    # 猜出来的过滤式一旦失配，字幕就是一片空白，读片的人分不清"没发生"和"没查到"。
    name = [r.get('action_name', '') for r in mine]
    ti = None
    for i in range(len(mine) - 1, -1, -1):
        if name[i] == '抬杆到位':
            ti = i
            break
    if ti is None:
        print('  （还没有一次"抬杆到位"，差值无从算起）')
    else:
        up = mine[ti]
        tc = next((i for i in range(ti + 1, len(mine)) if name[i] == '下发落杆'), None)
        td = next((i for i in range((tc or ti) + 1, len(mine)) if name[i] == '落杆到位'), None)
        print('  单程抬杆耗时 %.2f s（理论 open_angle/velocity=%.2f s，超时上限 %.1f s）' % (
            float(up.get('elapsed_s', 0.0)), theory, float(bar['raise_timeout_s'])))
        if tc is None:
            print('  抬到位之后还没出现过"下发落杆"（这一轮尚未收尾）')
        else:
            hold_gap = float(mine[tc]['sim_ts']) - float(up['sim_ts'])
            print('  实测 抬到位→下发落杆 %.2f s：必须 ≥ 保持全开 %.2f s，'
                  '多出来的是车过杆的时间（巡航 %.2f m/s）%s' % (
                      hold_gap, hold, float(pk['shuttle']['cruise_speed']),
                      '符合' if hold_gap >= hold - 0.15 else '!! 短于保持全开时间'))
            if td is None:
                print('  已下发落杆、还没落到位')
            else:
                low_gap = float(mine[td]['sim_ts']) - float(mine[tc]['sim_ts'])
                print('  实测 下发落杆→落到位 %.2f s（记录里的 elapsed=%.2f s，'
                      '落杆超时 %.1f s）%s' % (
                          low_gap, float(mine[td].get('elapsed_s', 0.0)),
                          float(bar['lower_timeout_s']),
                          '到位' if mine[td].get('reached') else '!! 未到位'))
    guard = [r for r in mine if r.get('action_name') in ('防砸重抬', '拒绝动作')]
    for r in guard[-3:]:
        print('  %s @ %.2f s｜%s｜车=%s 牌=%s' % (
            r['action_name'], float(r['sim_ts']), (r.get('note') or '')[:52],
            r.get('vehicle_id') or '-', r.get('plate_number') or '-'))
    if not guard:
        print('  这道闸的日志里还没有"防砸重抬/拒绝动作"（条目 3 会用一次杆下令落杆来触发）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
