#!/usr/bin/env python3
# C 层判定：留守文件里有真调用（第 1 类）指向要走的人，那它自己多半也是任务五/六的
# 一次性勘查/恢复脚本 —— 用户要的是"全部与任务五六有关的代码"，所以这一批也要走。
# 但不能凭"它调用了 parking_cli.py"就下手：任务一~四的运行链本身就引用过这些名字。
# 于是两条硬规矩：
#   1) 权重比 ratio56 = 56 命中 / (56+14 命中)，ratio56 >= 0.5 才算"它的主业是任务五六"；
#      命中数用 findall 实际数出来，不是我读一眼的感觉。
#   2) LIVE 白名单无条件留下并打补丁 —— 这些是任务一~四的启动链与就绪闸，
#      哪怕它们提到 parking 一百次。白名单逐条写死在这里，可复核。
import os
import re

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
OUT = '/tmp/quarantine_parketc_307'

go = [l.strip() for l in open(os.path.join(OUT, 'MANIFEST.txt'), encoding='utf-8') if l.strip()]
stems = {os.path.splitext(os.path.basename(p))[0]: p for p in go}
alt = '|'.join(re.escape(s) for s in sorted(stems))
INV = re.compile(r'(?:^|[;&|]|\s)(?:bash|sh|source|\./|python3?|colcon|ros2\s+run|ros2\s+launch|timeout\s+\S+\s+python3?)\b[^\n#]*?\b(' + alt + r')\b')
RUNNODE = re.compile(r'ros2\s+run\s+\S+\s+(' + alt + r')(?:\.py)?')

FEAT_56 = re.compile(r'parking|etc_gate|etc_rsu|etc_manager|etc_recorder|etc_zones|etc_cli|etc_selftest|'
                     r'BarrierCommand|BarrierState|car_7|car7|摆渡|道闸|闸杆|停车场|通行记录|扣费|OBU|RSU')
FEAT_14 = re.compile(r'traffic_light|signal_drive|adaptive_timing|plate_gen|plate_layout|plate_recognizer|'
                     r'plate_recorder|speed_detect|speed_recorder|websocket|dashboard|gate_j|卡口|车牌|测速|信号灯|配时')

LIVE = ('tools/start_signal.sh', 'tools/stop_signal.sh', 'tools/restart_clean_stack.sh',
        'tools/place_cars.py', 'tools/node_instances92.py', 'tools/vid_rec.py',
        'tools/vid_common.sh', 'tools/open_gui77.sh', 'src/v2x_demo/v2x_demo/v2x_dashboard.py',
        'tools/state_check304.sh', 'tools/state_check305.sh', 'tools/state_check306.sh',
        'tools/fleet_moving_guard303.py', 'tools/restore_video_stage303.sh',
        'tools/restore_exact_1014_305.sh', 'src/v2x_demo/CMakeLists.txt')

rows = []
for d, sub, fns in os.walk(os.path.join(ROOT, 'tools')):
    sub[:] = [s for s in sub if s not in ('__pycache__', '存档_弃用任务五六_1016')]
    for fn in fns:
        p = os.path.join(d, fn)
        if p in go or 'quarantine' in fn or not fn.endswith(('.py', '.sh')):
            continue
        try:
            txt = open(p, encoding='utf-8', errors='strict').read()
        except (UnicodeDecodeError, OSError):
            continue
        calls = []
        for i, line in enumerate(txt.splitlines(), 1):
            s = line.strip()
            if s and not s.startswith('#') and (INV.search(s) or RUNNODE.search(s)):
                calls.append((i, s[:120]))
        if not calls:
            continue
        n56 = len(FEAT_56.findall(txt))
        n14 = len(FEAT_14.findall(txt))
        rows.append((n56 / float(n56 + n14) if (n56 + n14) else 1.0, n56, n14, p, calls))

rows.sort(reverse=True)
rel = lambda p: p[len(ROOT) + 1:]
C, patch, nosig = [], [], []
for ratio, n56, n14, p, calls in rows:
    if n56 + n14 == 0 and rel(p) not in LIVE:
        # 两边关键词一个都没命中 —— "它属于哪个任务"没有内容证据。
        # 这种文件默认**留下**：移走它没有任何收益（它只是不调用而已），
        # 误伤任务一~四的代价却是真的；只有用户点名才动。
        nosig.append((ratio, n56, n14, p, calls))
    elif rel(p) in LIVE:
        patch.append((ratio, n56, n14, p, calls))
    elif ratio >= 0.5:
        C.append((ratio, n56, n14, p, calls))
    else:
        patch.append((ratio, n56, n14, p, calls))

print('有真调用的留守文件共 %d 个 → C 层（一起走）%d 个｜留下并打补丁/只报数 %d 个｜无内容信号默认留下 %d 个'
      % (len(rows), len(C), len(patch), len(nosig)))
print()
print('===== C 层：主业是任务五/六的一次性脚本（ratio56>=0.5 且不在 LIVE 白名单）=====')
for ratio, n56, n14, p, calls in C:
    print('  %-34s ratio=%.2f  56:%-4d 14:%-4d 调用点 %d 处' % (rel(p), ratio, n56, n14, len(calls)))
print()
print('===== 留下：LIVE 白名单 或 主业是任务一~四（ratio56<0.5）=====')
for ratio, n56, n14, p, calls in patch:
    tag = 'LIVE' if rel(p) in LIVE else '共享'
    print('  %-34s [%s] ratio=%.2f 56:%-4d 14:%-4d 调用点 %d 处' % (rel(p), tag, ratio, n56, n14, len(calls)))
    for i, s in calls:
        print('        :%-5d %s' % (i, s))

with open(os.path.join(OUT, 'MANIFEST_C.txt'), 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(p for _, _, _, p, _ in C) + ('\n' if C else ''))
with open(os.path.join(OUT, 'MANIFEST_NOSIG.txt'), 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(p for _, _, _, p, _ in nosig) + ('\n' if nosig else ''))
print()
print('===== 无内容信号（56/14 关键词都是 0 次）：默认留下，只报不动 =====')
for ratio, n56, n14, p, calls in nosig:
    print('  %-34s 调用点 %d 处（%s）' % (rel(p), len(calls), calls[0][1][:90]))
print()
print('C 层清单 %s/MANIFEST_C.txt（%d 项）｜无信号清单 MANIFEST_NOSIG.txt（%d 项）'
      % (OUT, len(C), len(nosig)))
