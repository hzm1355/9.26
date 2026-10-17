#!/usr/bin/env python3
# 隔离清单推导器（任务五 停车场闸机 + 任务六 ETC）。规则写死在这里，为的是**可复核**：
# 同一份规则跑两遍结果必须一样，谁也不能靠手感挑文件。
#
# 扫描范围：src/v2x_demo 的 v2x_demo/、config/、launch/ 三个目录 + tools/ 一层。
#   不扫 src/smart_car_sim 与 src/v2x_gazebo_plugins —— world 里停车场几何与道闸
#   要留在场景里（用户点名"保留 world"），而那块几何正是靠 barrier 插件驱动的；
#   把插件或模型一起带走会让 gzserver 加载 world 失败，等于砸了任务一~四的现场。
#
# 判定（按优先级）：
#   1) 文件名命中 parking_ / etc_ / *_etc_* / barrier / parking_zones / etc_zones → 必走（A 层）
#   2) 内容命中任务五六特征（parking / etc_ / BarrierCommand / car_7 / 道闸…）
#      且**不**命中任务一~四特征（traffic_light / signal_drive / plate_* / speed_* /
#      websocket / dashboard / 卡口 / 测速 / 信号灯）→ 走（B 层：只服务这两个任务的脚本）
#   3) 内容命中任务五六、又命中任务一~四特征 → **留下**，只报数（共享基建，
#      比如 place_cars.py / vid_rec.py / restart_clean_stack.sh / node_instances92.py）
#   4) 内容没命中任务五六 → 不动
import os
import re
import sys

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
NAME_A = re.compile(r'(^|_)(parking_|etc_|gen_parking|gen_etc|parking_zones|etc_zones|barrier|etc_)')
FEAT_56 = re.compile(
    r'parking|etc_gate|etc_rsu|etc_manager|etc_recorder|etc_zones|etc_cli|etc_selftest|'
    r'BarrierCommand|BarrierState|car_7|car7|摆渡|道闸|闸杆|停车场|通行记录|扣费|OBU|RSU')
FEAT_14 = re.compile(
    r'traffic_light|signal_drive|adaptive_timing|plate_gen|plate_layout|plate_recognizer|'
    r'plate_recorder|speed_detect|speed_recorder|websocket|dashboard|gate_j|卡口|车牌|测速|信号灯|配时')
SKIP_NAME = re.compile(r'(quarantine|\.keep$|\.bak|pre_|archive|_diag15[0-9])')
DIRS = [os.path.join(ROOT, 'src/v2x_demo/v2x_demo'),
        os.path.join(ROOT, 'src/v2x_demo/config'),
        os.path.join(ROOT, 'src/v2x_demo/launch'),
        os.path.join(ROOT, 'tools')]
EXT = ('.py', '.sh', '.yaml')

A, B, KEEP = [], [], []
for d in DIRS:
    if not os.path.isdir(d):
        print('!! 目录不存在：%s（规则前提变了，别看结果）' % d, file=sys.stderr)
        continue
    for fn in sorted(os.listdir(d)):
        p = os.path.join(d, fn)
        if not os.path.isfile(p) or not fn.endswith(EXT):
            continue
        if SKIP_NAME.search(fn):
            continue
        base = fn
        try:
            with open(p, encoding='utf-8', errors='strict') as fh:
                txt = fh.read()
        except (UnicodeDecodeError, OSError):
            continue
        name_hit = bool(NAME_A.search(base))
        feat56 = bool(FEAT_56.search(txt))
        feat14 = bool(FEAT_14.search(txt))
        if name_hit:
            A.append(p)
        elif feat56 and feat14:
            KEEP.append(p)
        elif feat56:
            B.append(p)

out = '/tmp/quarantine_parketc_307'
os.makedirs(out, exist_ok=True)
for nm, lst in (('MANIFEST_A', A), ('MANIFEST_B', B), ('MANIFEST_KEPT_SHARED', KEEP)):
    with open(os.path.join(out, nm + '.txt'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lst) + ('\n' if lst else ''))
with open(os.path.join(out, 'MANIFEST.txt'), 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(sorted(set(A + B))) + '\n')

print('A 层（文件名直接命中）%d 项' % len(A))
print('B 层（内容只服务任务五六）%d 项' % len(B))
print('合计待隔离 %d 项' % len(set(A + B)))
print('共享基建、只报不动 %d 项：' % len(KEEP))
for p in KEEP:
    print('    留 %s' % p[len(ROOT) + 1:])
