#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位 room_33.obj 里第一条"引用了还没读到的顶点"的面行。

mesh_intrusion_audit.py 在 ht = S*max(vy[a],vy[b],vy[c]) 处 IndexError，
说明 idx 里有超过 len(vx) 的引用。要么 v 行晚于 f 行（交错写法），
要么存在非法的 0/超大索引。这里只报告，不改文件。
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.obj')

nv = 0
nface = 0
bad = []
first_v_line = None
last_v_line = None
first_f_line = None
with open(OBJ, 'rb', buffering=1 << 22) as fh:
    for ln, raw in enumerate(fh, 1):
        if not raw:
            continue
        c0 = raw[0]
        if c0 == 118 and raw[1:2] == b' ':
            nv += 1
            if first_v_line is None:
                first_v_line = ln
            last_v_line = ln
        elif c0 == 102:
            nface += 1
            if first_f_line is None:
                first_f_line = ln
            idx = []
            for tok in raw[1:].split():
                if tok[:1] in (b'/', b'\\'):
                    continue
                s = tok.split(b'/')[0]
                try:
                    i = int(s)
                except ValueError:
                    continue
                idx.append(nv + i + 1 if i < 0 else i)
            if any(i > nv or i < 1 for i in idx) and len(bad) < 5:
                bad.append((ln, nv, idx, raw[:80]))
print('顶点 %d  面 %d' % (nv, nface))
print('v 行区间 %s~%s，首条 f 行 %s' % (first_v_line, last_v_line, first_f_line))
for ln, n, idx, raw in bad:
    print('  行 %d  当前顶点数 %d  idx=%s  %r' % (ln, n, idx, raw))
if not bad:
    print('  没有越界引用（那 IndexError 另有原因）')
