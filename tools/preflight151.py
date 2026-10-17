#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""长跑前的 20 秒预检：yaml 能不能 parse、三节点活没活、识别器在不在。

为什么单独一个文件而不是塞进 run 脚本：parking_zones.yaml 是我刚手改过的，一个缩进错误
会让 shuttle 在启动时就抛异常、日志里只留一行 traceback，然后整轮 7 分钟白等（memory
「长跑脚本先预检依赖」）。这里先花 20 s 把它和依赖节点一起过一遍。
"""
import subprocess
import sys

import yaml

CFG = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/parking_zones.yaml'
rc = 0

try:
    d = yaml.safe_load(open(CFG, encoding='utf-8'))['parking']
except Exception as e:
    print('!! yaml 解析失败：%s' % e)
    sys.exit(2)
sh = d['shuttle']
print('yaml OK｜entry=%d exit=%d reset=%d 点' % (
    len(sh['entry_route']), len(sh['exit_route']), len(sh['reset_route'])))
print('  entry#0 = %s' % sh['entry_route'][0])
print('  reset 末格 = %s' % sh['reset_route'][-1])
if sh['entry_route'][0][:2] != sh['reset_route'][-1][:2]:
    print('!! 等待位和复位末格不是同一格，车每圈都会重新找位置')
    rc = 2
for name, wp in (('entry#0', sh['entry_route'][0]), ('reset 末格', sh['reset_route'][-1])):
    d_ = ((wp[0] - 12.00) ** 2 + (wp[1] + 11.21) ** 2) ** 0.5
    ok = d_ > 2.5
    print('  %s 到 gate_in focus 的 d=%.2f m｜>far 2.5：%s' % (name, d_, 'OK' if ok else '不通过'))
    if not ok:
        rc = 2

out = subprocess.run(['bash', '-lc',
                      "for n in parking_shuttle_node parking_manager_node parking_recorder_node "
                      "plate_recognizer_node; do echo -n \"$n=$(pgrep -cf lib/v2x[_]demo/$n) \""
                      "done; echo"],
                     capture_output=True, text=True, timeout=60).stdout
print('节点实例数：%s' % out.strip())
sys.exit(rc)
