#!/usr/bin/env python3
# 隔离后的"悬空引用"体检器：站在**留下来**的那一侧反查。
# 若某个留下的文件（脚本/launch/CMakeLists/world/README）按名字引用了将被移走的文件，
# apply 之后它就会指向空气 —— 这一份把每一处点名出来，决定要不要打补丁。
#
# 口径：
#   - 被引用项 = MANIFEST.txt(A+B) 里每个文件的**词干**（去扩展名），
#     因为脚本之间常用 `bash tools/x.sh`、`python3 tools/x.py`、也有的写成 `run x`。
#   - 扫描范围 = 工程树内所有文本文件，排除：清单自身、*.keep/*.bak/pre_*/archive、
#     以及清单里那 243 个要走的人（它们自己的互相引用不用管）。
#   - 特别把 src/v2x_gazebo_plugins 与 worlds/*.world 纳进来：barrier 插件是
#     world 里道闸的动力学来源，插件不能因为"名字像任务五"就被带走。
import os
import re
import sys

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
OUT = '/tmp/quarantine_parketc_307'

go = [l.strip() for l in open(os.path.join(OUT, 'MANIFEST.txt'), encoding='utf-8') if l.strip()]
a_set = set(l.strip() for l in open(os.path.join(OUT, 'MANIFEST_A.txt'), encoding='utf-8') if l.strip())
b_set = set(l.strip() for l in open(os.path.join(OUT, 'MANIFEST_B.txt'), encoding='utf-8') if l.strip())
keep_set = set(l.strip() for l in open(os.path.join(OUT, 'MANIFEST_KEPT_SHARED.txt'), encoding='utf-8') if l.strip())

stems = {}
for p in go:
    stems[os.path.splitext(os.path.basename(p))[0]] = p
pat = re.compile(r'(?<![A-Za-z0-9_])(' + '|'.join(re.escape(s) for s in sorted(stems)) + r')(?![A-Za-z0-9_])')

TEXT_EXT = ('.py', '.sh', '.bash', '.yaml', '.yml', '.txt', '.md', '.world', '.sdf',
            'CMakeLists.txt', 'setup.py', 'package.xml')
SKIP_DIR = ('存档_弃用任务五六_1016', '__pycache__', '.git', 'build', 'install', 'log', 'node_modules')
SKIP_FILE = re.compile(r'(\.keep$|\.bak|\.orig$|pre_|quarantine|MANIFEST|_manchr)')

hits = {}
doc_hits = {}
scanned = 0
doc_scanned = 0
for d, sub, fns in os.walk(ROOT):
    sub[:] = [s for s in sub if s not in SKIP_DIR]
    for fn in fns:
        p = os.path.join(d, fn)
        if p in go:
            continue
        if SKIP_FILE.search(fn):
            continue
        if not (fn.endswith(TEXT_EXT) or fn in ('CMakeLists.txt', 'setup.py', 'package.xml')):
            continue
        # 证据转储（.txt/.md/_diag*）不是调用方：它们里面写着 parking_zones 只是当时的
        # 屏幕文本，移走节点不会让它们"指向空气"。分开计，避免把 139 个读数当 139 个待改点。
        DOC_ONLY = fn.endswith(('.txt', '.md', '.json', '.jsonl', '.csv', '.log')) or \
            '/_diag' in p or '/存档' in p
        try:
            with open(p, encoding='utf-8', errors='strict') as fh:
                lines = fh.readlines()
        except (UnicodeDecodeError, OSError):
            continue
        scanned += 1 if not DOC_ONLY else 0
        doc_scanned += 1 if DOC_ONLY else 0
        found = []
        for i, line in enumerate(lines, 1):
            m = pat.search(line)
            if m:
                found.append((i, m.group(1), line.strip()[:130]))
        if found:
            (doc_hits if DOC_ONLY else hits)[p] = found

print('被引用词干 %d 个｜扫描留下的代码/配置文件（不含证据转储）%d 个｜其中有引用要走的人的 %d 个'
      % (len(stems), scanned, len(hits)))
print('  另有证据转储类文件 %d 个也提到过这些名字（不参与运行，不动它们）' % len(doc_hits))
print()
print('===== 分层归属核对（A=%d B=%d KEEP=%d，三者应互不相交）=====' % (len(a_set), len(b_set), len(keep_set)))
for nm, s in (('A∩B', a_set & b_set), ('A∩KEEP', a_set & keep_set), ('B∩KEEP', b_set & keep_set)):
    print('  %s = %d 项%s' % (nm, len(s), '（应为 0）' if not s else ' !! ' + ', '.join(sorted(os.path.basename(x) for x in s))))
for fn in ('build_parking.sh', 'restart_parking89.sh', 'restart_parking91.sh',
           'park6_only.sh', 'park_car6.py', 'respawn_car3_263.sh', 'drive_home136.py',
           'test_barrier_anchor.py', 'test_barrier_dyn.py', 'gen_parking_world.py'):
    where = [nm for nm, s in (('A', a_set), ('B', b_set), ('KEEP', keep_set))
             if any(x.endswith('/tools/' + fn) for x in s)]
    print('  %-24s -> %s' % (fn, '+'.join(where) if where else '!! 三层都不在'))
print()
print('===== 悬空引用：留下的文件里按名字点了要走的人 =====')
for p in sorted(hits):
    print('  %s' % p[len(ROOT) + 1:])
    for i, name, line in hits[p]:
        tgt = stems[name]
        lay = 'A' if tgt in a_set else 'B'
        print('     :%-5d [%s层] 引用 %s ｜ %s' % (i, lay, name, line))
print()
print('===== 反向：要走的人里有没有引用「留下的共享基建」====')
print('  （不影响 build，但说明归档包不是自洽子集 —— 单独列出即可）')
rev = {}
shared_names = set(os.path.splitext(os.path.basename(x))[0] for x in keep_set)
rpat = re.compile(r'(?<![A-Za-z0-9_])(' + '|'.join(re.escape(s) for s in sorted(shared_names)) + r')(?![A-Za-z0-9_])') if shared_names else None
for p in go:
    if rpat is None:
        break
    try:
        with open(p, encoding='utf-8', errors='strict') as fh:
            txt = fh.read()
    except (UnicodeDecodeError, OSError):
        continue
    ms = set(rpat.findall(txt))
    if ms:
        rev[p] = sorted(ms)
print('  要走且引用了留守基建的文件 %d 个' % len(rev))
for p in sorted(rev)[:40]:
    print('    %s -> %s' % (p[len(ROOT) + 1:], ','.join(rev[p])))
if len(rev) > 40:
    print('    …（其余 %d 个未列）' % (len(rev) - 40))
