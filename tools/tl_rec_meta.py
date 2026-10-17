#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印一份录制文件的元信息（时长/事件数/分组/车辆/控制模式分布）。

单独成文件而不是在 shell 里内联 python：内联 heredoc 在 Git Bash 这一层会挂死。
用法：python3 tl_rec_meta.py <录制json>
"""
import collections
import json
import sys

d = json.load(open(sys.argv[1] if len(sys.argv) > 1 else '/tmp/tl_log.json', encoding='utf-8'))
m, ev = d['meta'], d['events']
veh = [e for e in ev if e.get('type') == 'vehicle']
light = [e for e in ev if e.get('type') != 'vehicle']
modes = collections.Counter(e.get('control_mode', '') for e in light if e.get('control_mode'))
print('  录制时长 %.1f s  事件 %d  灯态 %d  车辆位姿 %d' % (
    m.get('record_duration', 0), m.get('event_count', len(ev)), len(light), len(veh)))
print('  信号组 %s  车辆 %s' % (m.get('groups'), m.get('vehicles')))
print('  灯态事件里的控制模式分布: %s' % (dict(modes) or '（无 control_mode 字段）'))
print('  首个灯态事件: %s' % (light[0] if light else None))
