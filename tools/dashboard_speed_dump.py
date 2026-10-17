#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 :8090 的 /state.json 里"测速路段"三块原样打成文本，作为界面证据。

浏览器结构快照也能拿，但那份要靠人工读屏；/state.json 是面板渲染的同一份数据源，
脚本化之后每轮取证都能重放，数字也不会抄错。
"""
import json
import sys
import urllib.request

url = (sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:8090/state.json')
d = json.load(urllib.request.urlopen(url, timeout=10))
S = d.get('speed') or {}
Z = S.get('zones') or {}
print('======== 测速路段 点位表（数据源 /state.json → speed.zones，%d 个点位）' % len(Z))
print('表头：路段 | 模式 | 阈值 | 窗口流量/车 | 均速 | 峰/谷 | 超速(占比) | 累计流量/超速 | 在途')
for zid, z in sorted(Z.items()):
    print('  %-20s %-6s 阈值%-6s 流量%-3s/%-2s车 均速%-6s 峰%-6s 谷%-6s 超速%s(%s%%) '
          '累计%s/%s 在途%s   [%s]'
          % (z.get('road') or zid, '瞬时' if z['mode'] == 0 else '区间', z['limit'],
             z['flow'], z['veh'], z['avg'], z['max'], z['min'],
             z['over'], z['ratio'], z['tot_flow'], z['tot_over'], z['pending'], zid))
rows = S.get('rows') or []
print('\n======== 最近过车（speed.rows，保留 %d 行，最新在前）' % len(rows))
for r in rows[:10]:
    print('  %s %s %-20s %s %-8s 雷达/均速 %-6s 位姿 %-6s 峰 %-6s 阈值 %-6s %s %s'
          % (r['ts'], r['mode'], r['zone_id'], r['vehicle_id'], r.get('plate') or '-',
             r['kmh'], r.get('pose', '-'), r['peak'], r['limit'],
             '超速' if r['over'] else '正常', r['extra']))
als = S.get('alerts') or []
print('\n======== 超速告警（speed.alerts，保留 %d 行，画布上标红叉）' % len(als))
for a in als[:8]:
    print('  %s sim %ss %s %-20s %s %-8s 速度%-6s/阈值%-6s 超%-6s 位置(%s,%s) ref=%s'
          % (a['ts'], a['sim_t'], a['note'], a['zone_id'], a['vehicle_id'], a['plate'],
             a['kmh'], a['limit'], a['over'], a['x'], a['y'], a['ref']))
