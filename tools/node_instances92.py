#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#92 列出现存的 v2x_demo 常驻节点实例（按 /proc 的 exe+cmdline 判定）。

为什么要单独写成一个文件：上一版脚本用 `python3 -c "…"` 内联，双引号里的换行被
shell 吃掉，整段判断静默失败，日志里只留下 SyntaxError，实例数到底几个反而没看见。
按记忆里那条"重启常驻节点必须杀光同名所有实例并校验实例数"，这一步不能省。
"""
import os
import re
import sys

TARGETS = ['parking_manager_node', 'parking_recorder_node', 'parking_shuttle_node',
           'signal_drive_node', 'plate_recognizer_node', 'speed_detect_node']

rows = []
for pid in sorted((d for d in os.listdir('/proc') if d.isdigit()), key=int):
    try:
        exe = os.path.realpath('/proc/%s/exe' % pid)
        with open('/proc/%s/cmdline' % pid, 'rb') as fh:
            cmd = fh.read().decode('utf-8', 'replace').replace('\x00', ' ').strip()
    except OSError:
        continue
    if not exe.endswith(('python3', 'python3.10')):
        continue
    for t in TARGETS:
        if '/lib/v2x_demo/%s.py' % t in cmd:
            rows.append((t, pid, cmd[:96]))

want = sys.argv[1:] or TARGETS
for t in TARGETS:
    if t not in want:
        continue
    hits = [r for r in rows if r[0] == t]
    print('  %-22s 实例=%d' % (t, len(hits)))
    for _, pid, cmd in hits:
        try:
            st = open('/proc/%s/stat' % pid).read().split(') ', 1)[1].split()[19]
            secs = int(st) / os.sysconf('SC_CLK_TCK')
            boot = float(open('/proc/stat').read().split('btime ')[1].split()[0])
            import time
            print('      PID=%-7s 起于 %s' % (
                pid, time.strftime('%m-%d %H:%M:%S', time.localtime(boot + secs))))
        except Exception as e:                       # 只影响打印，不影响计数
            print('      PID=%-7s (%s)' % (pid, e))
