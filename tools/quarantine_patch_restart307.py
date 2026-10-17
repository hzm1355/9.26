#!/usr/bin/env python3
# 把 restart_clean_stack.sh 的 "9c) 停车场三节点" 整段摘掉。
# 为什么必须摘而不是留着：那一行 `bash $WS/tools/start_parking_nodes.sh ... || true`
# 有 || true 兜着，不摘也不会让整栈重启失败 —— 但它每次重启都会刷一屏
# "No such file or directory"，而这一份日志是任务一~四每轮取证的现场记录，
# 脏日志会让人分不清"没起来"和"本来就不该起"。
# 边界靠锚点：从 echo '[restart] 9c) 起，到 echo '[restart] 10) 前一行止。
import os
import sys

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
SRC = os.path.join(ROOT, 'tools/restart_clean_stack.sh')
ARCH = os.path.join(ROOT, u'存档_弃用任务五六_1016')

src_txt = open(SRC, encoding='utf-8').read()
lines = src_txt.split('\n')


def find(pred, label):
    for i, l in enumerate(lines):
        if pred(l):
            return i
    raise SystemExit('!! 锚点失配：%s —— 本次一个字节都不改' % label)


a = find(lambda l: "[restart] 9c" in l, "9c 段头")
b = find(lambda l: "[restart] 10" in l, "10 段头")
if b - a < 2:
    raise SystemExit('!! 段头顺序异常：9c 在 %d、10 在 %d' % (a, b))
txt = '\n'.join(lines[:a] + lines[b:])
left = [l for l in txt.split('\n') if 'parking' in l.lower() or 'start_parking' in l]
print('原 %d 行 → 新 %d 行（删 %d-%d 共 %d 行）'
      % (len(lines), len(txt.split('\n')), a + 1, b, b - a))
print('删完后仍含 parking 字样的行 %d 处：%s' % (len(left), [x.strip()[:70] for x in left]))
if '--dry' in sys.argv:
    print('DRY：未写文件')
    raise SystemExit(0)
os.makedirs(ARCH, exist_ok=True)
bak = os.path.join(ARCH, 'restart_clean_stack.sh.pre_quarantine')
if not os.path.exists(bak):
    open(bak, 'w', encoding='utf-8').write(src_txt)
open(SRC, 'w', encoding='utf-8').write(txt)
print('已写入 %s（备份 %s）' % (SRC, bak))
