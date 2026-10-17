#!/usr/bin/env python3
# 悬空引用分级：把"留下的代码里点了要走的人"分成三类，只有第 1 类必须打补丁。
#   1) 真调用  —— bash/sh/timeout/python3 后面跟着 <要走的文件路径>，或 ros2 run <pkg> <节点>；
#                 apply 之后这一行会指向空气（脚本报 No such file / ros2 run 找不到节点）。
#   2) 读日志/读台账 —— /tmp/parking_manager_node.log、/tmp/v2x_parking/* 这类；节点没了，
#                 文件不再增长，grep 会报"No such file"或空返回，脚本自身多半有 || true 兜住。
#   3) 注释/文本 —— # 行、echo 里的说明。不动。
# 分类靠行上的**形态**而不是我读一遍的感觉：同一规则跑两遍要给出同一份答案。
import os
import re

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
OUT = '/tmp/quarantine_parketc_307'

go = [l.strip() for l in open(os.path.join(OUT, 'MANIFEST.txt'), encoding='utf-8') if l.strip()]
layer = {}
for nm, f in (('A', 'MANIFEST_A.txt'), ('B', 'MANIFEST_B.txt')):
    for p in open(os.path.join(OUT, f), encoding='utf-8'):
        p = p.strip()
        if p:
            layer[p] = nm
stems = {os.path.splitext(os.path.basename(p))[0]: p for p in go}
alt = '|'.join(re.escape(s) for s in sorted(stems))

# 第 1 类：解释器/构建/launch 直接点名
INV = re.compile(r'(?:^|[;&|]|\s)(?:bash|sh|zsh|source|\./|python3?|colcon|ros2\s+run|ros2\s+launch|install\.py|timeout\s+\S+\s+python3?)\b[^\n#]*?\b(' + alt + r')\b')
# 节点名出现在 ros2 run / pgrep / setsid 那一类调用位
RUNNODE = re.compile(r'ros2\s+run\s+\S+\s+(' + alt + r')(?:\.py)?')
# 第 2 类：读运行期产物
LOGREAD = re.compile(r'/tmp/(' + alt + r')|(' + alt + r')\.log|v2x_parking|v2x_etc')
CODE_EXT = ('.py', '.sh', '.bash', '.yaml', '.yml', '.world', '.sdf', '.launch')
SKIP_DIR = ('存档_弃用任务五六_1016', '__pycache__', '.git', 'build', 'install', 'log')
SKIP_FILE = re.compile(r'(\.keep$|\.bak|\.orig$|pre_|quarantine|MANIFEST|_manchr)')

inv, logs, comment = [], [], []
for d, sub, fns in os.walk(ROOT):
    sub[:] = [s for s in sub if s not in SKIP_DIR]
    for fn in fns:
        p = os.path.join(d, fn)
        if p in go or SKIP_FILE.search(fn):
            continue
        if not (fn.endswith(CODE_EXT) or fn in ('CMakeLists.txt', 'setup.py', 'package.xml')):
            continue
        if fn.endswith(('.yaml', '.yml', '.world', '.sdf')):
            continue          # 配置与场景里的都是注释，不参与调用
        try:
            lines = open(p, encoding='utf-8', errors='strict').readlines()
        except (UnicodeDecodeError, OSError):
            continue
        for i, line in enumerate(lines, 1):
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            hit = INV.search(s) or RUNNODE.search(s)
            if hit:
                inv.append((p, i, s[:150]))
            elif LOGREAD.search(s):
                logs.append((p, i, s[:150]))
print('第1类 真调用 %d 处｜第2类 读日志/台账 %d 处（都只在代码文件里查，注释与 yaml/world 已排除）' % (len(inv), len(logs)))
print()
print('===== 第 1 类：apply 后必须补丁，否则指向空气 =====')
for p, i, s in inv:
    print('  %s:%d\n      %s' % (p[len(ROOT) + 1:], i, s))
print()
print('===== 第 2 类：只是读运行期产物，节点走了就读到新东西（列前 40）=====')
for p, i, s in logs[:40]:
    print('  %s:%d  %s' % (p[len(ROOT) + 1:], i, s))
if len(logs) > 40:
    print('  …其余 %d 处' % (len(logs) - 40))
