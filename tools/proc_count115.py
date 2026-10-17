#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#115a 停车场链路上还有哪些节点活着，按 /proc/<pid>/cmdline 全文认，不用子串误伤。"""
import os
import re
import sys

WANT = ['parking_shuttle_node', 'parking_manager_node', 'signal_drive_node',
        'parking_recorder_node', 'plate_recog_node', 'vid_rec']
found = {k: [] for k in WANT}
for pid in sorted(os.listdir('/proc'), key=lambda s: int(s) if s.isdigit() else 0):
    if not pid.isdigit():
        continue
    try:
        with open('/proc/%s/cmdline' % pid, 'rb') as f:
            line = ' '.join(f.read().decode('utf8', 'ignore').split('\0'))
    except Exception:
        continue
    for k in WANT:
        if re.search(r'(^|[/ ])%s(\.py)?([ ]|$)' % re.escape(k), line):
            found[k].append((pid, line[:130]))
for k in WANT:
    print('%-22s 实例=%d' % (k, len(found[k])))
    for pid, line in found[k]:
        print('    PID=%-7s %s' % (pid, line))
print('gzserver PID=%s' % (' '.join(
    p for p in os.listdir('/proc') if p.isdigit() and os.path.exists('/proc/%s/exe' % p)
    and 'gzserver' in (os.readlink('/proc/%s/exe' % p) if os.path.islink('/proc/%s/exe' % p) else ''))
    or '无'))
sys.stdout.flush()
