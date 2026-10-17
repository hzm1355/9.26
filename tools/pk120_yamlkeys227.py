#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""227：本轮"三车位 + 双车分队 + 东环东移"改动的离线一致性闸。

为什么要单独一份而不是靠 check_parking_layout.py：那份体检只管停车场自己的几何自洽
（杆轴/相机反推链/地感防砸/扫掠净空），而本轮最容易出错的是**跨文件的引用**：
  * fleets 的键名必须等于 world 里的模型名，拼错一个字符节点就静默用默认三段航线，
    演出来的还是 car_6 时代那条路；
  * 白名单/临时车两条分支是由 control.plates 与 control.whitelist 的**交集**决定的，
    plates 写了但 whitelist 没写，car_3 就会走成临时车 —— 两条分支就少演一条；
  * 东腿从 x=14.5 挪到 16.20、同日再挪到 20.80，每次都同时改三处（signal_drive_params
    的 route、speed_zones 的区间断面与雷达、world 里 car_4 的出生位），漏改一处就是
    "测速段量不到车"或"car_4 从停车场里起步"。16.20 那次为什么不算到位，见
    tools/route_lot_conflict.py —— 它把两份 yaml 的几何放在同一张图上判相交，
    是"其他小车航线不经过停车场"这条要求的常驻判据，本脚本管的是"三处改齐了没"。
本脚本全部离线判，不起 Gazebo。
"""
import sys

import yaml

SIM = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
PK = SIM + '/src/v2x_demo/config/parking_zones.yaml'
SD = SIM + '/src/v2x_demo/config/signal_drive_params.yaml'
SZ = SIM + '/src/v2x_demo/config/speed_zones.yaml'

fails = []


def ck(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        fails.append(msg)


pk = yaml.safe_load(open(PK, encoding='utf-8'))['parking']
sh = pk['shuttle']
ctl = pk['control']

print('== 1 摆渡车分队 ==')
ck(sh.get('park_pose') is not None, 'shuttle.park_pose 已给（car_6 退役后与 car_3 出生位分开）')
ck(len(sh['park_pose']) == 3, 'park_pose 是 [x, y, yaw] 三个数')
fl = sh.get('fleets') or {}
ck(set(fl) >= {'car_2', 'car_3'}, 'fleets 里有 car_2 与 car_3（键名必须等于模型名）')
for m in ('car_2', 'car_3'):
    for leg in ('entry_route', 'exit_route', 'reset_route'):
        pts = (fl.get(m) or {}).get(leg)
        ck(bool(pts), '%s.%s 非空' % (m, leg))
        if pts:
            ck(all(len(p) == 3 for p in pts), '%s.%s 每点都是 [x,y,朝向]' % (m, leg))
    # 航线首尾相接：复位末点要落回进场首点（同一格或原地转 180° 的那一格）
    a = fl[m]['entry_route'][0][:2]
    b = fl[m]['reset_route'][-1][:2]
    ck(abs(a[0] - b[0]) < 0.01 and abs(a[1] - b[1]) < 0.01,
       '%s 复位末点 (%.2f,%.2f) 回到进场首点 (%.2f,%.2f)' % (m, b[0], b[1], a[0], a[1]))

print('== 2 两条管控分支真的能被触发 ==')
plates = ctl.get('plates') or {}
wl = set(ctl.get('whitelist') or {})
bl = set(ctl.get('blacklist') or [])
p3, p2 = plates.get('car_3'), plates.get('car_2')
ck(bool(p3) and bool(p2), 'control.plates 给了 car_3=%r 与 car_2=%r' % (p3, p2))
ck(p3 in wl, 'car_3 的车牌 %r 在白名单里 → 演"月租核验通过"' % p3)
ck(p2 not in wl and p2 not in bl,
   'car_2 的车牌 %r 既不在白名单也不在黑名单 → 演"未知车→临时车登记"' % p2)
ck(p3 != p2, '两辆车牌不同，条目 5 的记录才分得开')

print('== 3 三个原始车位 ==')
st = pk['stalls']
ck(len(st) == 3, 'stalls 有 3 个（原始沙盘标记的那三个）')
ids = [s['id'] for s in st]
ck(len(set(ids)) == 3, '车位 id 不重名：%s' % ids)
# 每辆车停的车位必须是三个之一，且两辆不同位
lot = pk['lot']
for m in ('car_2', 'car_3'):
    x, y = fl[m]['entry_route'][-1][:2]
    hit = [s['id'] for s in st
           if abs(s['center'][0] - x) <= s['size'][0] / 2 + 0.6
           and abs(s['center'][1] - y) <= s['size'][1] / 2]
    ck(bool(hit), '%s 入库点 (%.2f,%.2f) 落在某个车位内：%s' % (m, x, y, hit))
e = [tuple(fl[m]['entry_route'][-1][:2]) for m in ('car_2', 'car_3')]
ck(e[0] != e[1], '两辆车停不同车位：%s' % (e,))
for m in ('car_2', 'car_3'):
    for leg in ('entry_route', 'exit_route', 'reset_route'):
        for i, (x, y, _h) in enumerate(fl[m][leg]):
            ck(lot['x0'] - 2.6 <= x <= lot['x1'] + 2.6
               and lot['y0'] - 3.6 <= y <= lot['y1'] + 2.6,
               '%s.%s#%d (%.2f,%.2f) 在占地 %s 附近（不外飘）'
               % (m, leg, i, x, y, [lot['x0'], lot['x1'], lot['y0'], lot['y1']]))

print('== 4 东环东移三处同步（16.20→20.80 那次：让开停车场占地 16.50 与 PKG-P3）==')
EAST = 20.8
sd = yaml.safe_load(open(SD, encoding='utf-8'))['signal_drive']['cars']
ck(sd['car_2'].get('drive') is False, 'signal_drive 的 car_2 打了 drive: false（退出车流）')
ck(sd['car_3'].get('drive') is False, 'signal_drive 的 car_3 打了 drive: false（退出车流）')
for m in ('car_4', 'car_5'):
    xs = [abs(p[0]) for p in sd[m]['route']]
    ck(EAST in xs, '%s 的 route 里有东腿 x=%s（实测 %s）' % (m, EAST, sorted(set(xs))))
    ck(14.5 not in xs and 16.2 not in xs,
       '%s 的 route 里旧东腿 14.5/16.2 已清掉（16.20 的车体包络仍横穿 P3）' % m)
sz = yaml.safe_load(open(SZ, encoding='utf-8'))['speed']
zones = sz['zones']
east = [z for z in zones if 'east' in z.get('id', '')]
ck(bool(east), 'speed_zones 里有东段测速（找到 %s）' % [z.get('id') for z in zones])
for z in east:
    for k in ('entry', 'exit'):
        v = z.get(k)
        if v is not None:
            ck(abs(v[0] - EAST) < 0.01, '%s 的 %s x=%.2f 跟着东腿挪到 %s' % (z['id'], k, v[0], EAST))
    ck(abs(z['radar'][0] - (EAST - 2.2)) < 0.01,
       '%s 的雷达仍在车道以西 2.20 m（实测 x=%.2f）' % (z['id'], z['radar'][0]))
    ck(abs(z['entry'][0] - z['exit'][0]) < 0.01,
       '%s 的 entry/exit 在同一 x（区间测速才是同一条腿）' % z['id'])

print('== 5 world 里 car_4 的出生位跟着东腿 ==')
w = open(SIM + '/src/smart_car_sim/worlds/v2x_city_scene.world', encoding='utf-8').read()
ck('<pose>20.8 -5.0 7.0 0 0 -1.5708</pose>' in w,
   'car_4 出生位已是 (20.8,-5.0)')
ck('<pose>14.5 -5.0 7.0' not in w and '<pose>16.2 -5.0 7.0' not in w,
   'world 里不再有旧的 (14.5/-5.0)、(16.2,-5.0) 出生位')
import re
poses = re.findall(r'<model name="(car_\d)">.*?<pose>([^<]+)</pose>', w, re.S)
sp = {}
for name, p in poses:
    if name not in sp:
        sp[name] = p.strip().split()
seen = []
for name, pv in sorted(sp.items()):
    if name.startswith('car_') and len(pv) >= 2:
        seen.append((name, float(pv[0]), float(pv[1])))
for i in range(len(seen)):
    for j in range(i + 1, len(seen)):
        a, b = seen[i], seen[j]
        d = ((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5
        ck(d >= 1.70, '出生位不重叠：%s(%.2f,%.2f) 与 %s(%.2f,%.2f) 相距 %.2f m'
           % (a[0], a[1], a[2], b[0], b[1], b[2], d))

print()
print('结论：%s（%d 条不通过）' % ('全部通过' if not fails else '有账要修', len(fails)))
sys.exit(0 if not fails else 1)
