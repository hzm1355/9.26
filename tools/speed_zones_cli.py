#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/speed/zones 服务命令行：点位与限速阈值热改（条目 1 的"位置可灵活配置"、条目 3 的"自定义阈值"）。

用法：
  speed_zones_cli.py list
  speed_zones_cli.py limit   --zone SZ-A-west-instant --limit 3.0
  speed_zones_cli.py global  --limit 2.5
  speed_zones_cli.py add     --zone SZ-TMP --mode instant --x -11.0 --y 4.6 --limit 2.0
  speed_zones_cli.py add     --zone SZ-TMP2 --mode segment --entry -5.0,-14.6 --exit -16.0,-14.6 --limit 2.0
  speed_zones_cli.py remove  --zone SZ-TMP
  speed_zones_cli.py clear
"""
import argparse
import sys

import rclpy
from rclpy.node import Node

from v2x_msgs.srv import SetSpeedZone

OPS = {'list': SetSpeedZone.Request.OP_LIST,
       'limit': SetSpeedZone.Request.OP_SET_LIMIT,
       'global': SetSpeedZone.Request.OP_SET_GLOBAL_LIMIT,
       'add': SetSpeedZone.Request.OP_ADD,
       'remove': SetSpeedZone.Request.OP_REMOVE,
       'clear': SetSpeedZone.Request.OP_CLEAR}


def xy(s):
    a, b = s.split(',')
    return float(a), float(b)


# argparse 只把形如 -12 / -1.5 的 token 认作"负数选项值"，`-5.0,-14.6` 因为带逗号
# 被当成一个选项名，于是 `--entry -5.0,-14.6` 报 "expected one argument"（取证脚本
# 就死在这里过）。本 CLI 的选项一律只吃一个值，所以把紧跟选项的独立 token 粘成
# `--opt=value` —— 两种写法都能用，且不用改上面的用法说明。
OPTS = ('--zone', '--limit', '--mode', '--x', '--y', '--entry', '--exit',
        '--radar', '--beam', '--radius', '--road')


def normalize(argv):
    out, i = [], 0
    while i < len(argv):
        if argv[i] in OPTS and i + 1 < len(argv):
            out.append('%s=%s' % (argv[i], argv[i + 1]))
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('op', choices=sorted(OPS))
    ap.add_argument('--zone', default='')
    ap.add_argument('--limit', type=float, default=0.0)
    ap.add_argument('--mode', default='instant', choices=['instant', 'segment'])
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--entry', default='')
    ap.add_argument('--exit', dest='exit_', default='')
    ap.add_argument('--radar', default='')
    ap.add_argument('--beam', type=float, default=999.0)
    ap.add_argument('--radius', type=float, default=1.6)
    ap.add_argument('--road', default='')
    a = ap.parse_args(normalize(sys.argv[1:]))

    rclpy.init()
    node = Node('speed_zones_cli')
    cli = node.create_client(SetSpeedZone, '/speed/zones')
    # 服务客户端同样要等发现：刚重启的检测节点，wait_for_service 超时就直接判失败，
    # 不要把"没等到"当成"改成功了"。
    if not cli.wait_for_service(timeout_sec=15.0):
        print('/speed/zones 服务不可用（检测节点没起来？）')
        rclpy.shutdown()
        sys.exit(2)

    req = SetSpeedZone.Request()
    req.op = OPS[a.op]
    if req.op == SetSpeedZone.Request.OP_ADD and a.mode == 'segment' \
            and not (a.entry and a.exit_):
        print('区间点位必须同时给 --entry 与 --exit（x,y），否则长度是 0')
        sys.exit(2)
    req.zone_id = a.zone
    req.mode = 1 if a.mode == 'segment' else 0
    req.limit_kmh = a.limit
    req.radius = a.radius
    req.beam_deg = a.beam
    req.road = a.road
    req.center_x, req.center_y = a.x, a.y
    req.radar_x = req.radar_y = -9999.0
    if a.entry:
        req.entry_x, req.entry_y = xy(a.entry)
    if a.exit_:
        req.exit_x, req.exit_y = xy(a.exit_)
    if a.radar:
        req.radar_x, req.radar_y = xy(a.radar)

    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    resp = fut.result()
    print('success=%s  %s' % (resp.success, resp.message))
    for line in resp.zones:
        print('  · ' + line)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if resp.success else 1)


if __name__ == '__main__':
    main()
