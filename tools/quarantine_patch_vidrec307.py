#!/usr/bin/env python3
# 把 vid_rec.py 的 --mode parking 停车场面板摘掉（用户 10-16 选的范围里点名了"vid_rec 模式"）。
# 为什么用锚点而不是行号：行号会因为我前面的删除而整体位移，锚点不会。
# 为什么不只删主面板：wrap()/pk_sec()/pk_dur() 这几个小工具经实测只有停车场面板在用，
# 不一起删就是留下永远不会执行的死代码。
# 改前先备份进归档目录；改后三道复核：py_compile、--help 实际回包、残留符号计数必须为 0。
import os
import re
import sys

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
SRC = os.path.join(ROOT, 'tools/vid_rec.py')
ARCH = os.path.join(ROOT, u'存档_弃用任务五六_1016')

src_txt = open(SRC, encoding='utf-8').read()
lines = src_txt.split('\n')


def find(pred, label, frm=0):
    for i in range(frm, len(lines)):
        if pred(lines[i]):
            return i
    raise SystemExit('!! 锚点失配：%s —— 文件与规则前提不一致，本次一个字节都不改' % label)


def topdl(func):
    """整个顶层函数：从 def func( 起，到下一个顶层 def 前一行（把它的尾随空行一起吃掉）。"""
    i = find(lambda l: l.startswith('def %s(' % func), 'def %s' % func)
    j = i + 1
    while j < len(lines) and not lines[j].startswith('def '):
        j += 1
    return i, j - 1


drops = []
a = find(lambda l: l.startswith(u'  --mode parking '), 'docstring')
drops.append((a, find(lambda l: l.startswith("  --views 'http:inter#crop="), 'docstring 末', a)))
a = find(lambda l: l.startswith(u'# 停车场面板的中文译名表'), '常量表')
drops.append((a, find(lambda l: l.strip().startswith(
    'ParkingAction.ACT_HOLD_OPEN,'), '常量表末', a)))
a = find(lambda l: l.strip() == 'self.gates, self.passes, self.acts = {}, [], []', '状态字段')
drops.append((a, a + 2))
a = find(lambda l: l.startswith(u'        # 停车场这四路也要深队列'), '订阅块')
drops.append((a, find(lambda l: l.strip() == "lambda m: setattr(self, 'shut', m.data), 10)",
              '订阅块末', a)))
a = find(lambda l: l.startswith('    def on_pass(self, m):'), 'on_pass')
drops.append((a, find(lambda l: l.strip() == 'del self.acts[0]', 'on_act 末', a)))
for f in ('pk_dur', 'pk_sec', 'wrap', 'panel_parking'):
    drops.append(topdl(f))
# 分发那两行：锚点必须绑在 return panel_parking 上。
# 上一版找 "if A.mode == 'parking':" 命中的是订阅块里那一行（同一句法两处的），
# 于是区块互相重叠，倒序删除会把后一段的位移算进来 —— 多删 6 行。
a = find(lambda l: l.strip() == 'return panel_parking(r, im, y)', '分发')
drops.append((a - 1, a))

for i, j in sorted(drops, reverse=True):
    del lines[i:j + 1]

# 顶层 def 前面保证两个空行（删块会把空行一起吃掉，PEP8 之外的排版问题，但 diff 要干净）
out = []
for i, l in enumerate(lines):
    if l.startswith('def ') and i:
        nb = 0
        while out and out[-1] == '':
            out.pop()
            nb += 1
        out.extend([''] * 2)
    out.append(l)
txt = '\n'.join(out)

txt = re.sub(r'SpeedCapture, SpeedSegment, SpeedAlert, SpeedStats,\n\s*'
              r'BarrierState, ParkingPass, ParkingAction\)',
              'SpeedCapture, SpeedSegment, SpeedAlert, SpeedStats)', txt)
txt = txt.replace("choices=['tl', 'plate', 'both', 'speed', 'parking', 'none']",
                  "choices=['tl', 'plate', 'both', 'speed', 'none']")

left = sorted(set(re.findall(r'BarrierState|ParkingPass|ParkingAction|panel_parking|pk_dur|'
                             r'pk_sec|BARRIER_|PASS_KIND|PASS_RESULT|PASS_DIR|ACT_NAME|'
                             r'ACT_NOTE_KEYS|self\.gates|self\.passes|self\.acts|self\.shut|'
                             r'n_pass|n_act|\'parking\'|wrap\(', txt)))
print('原 %d 行 → 新 %d 行（减 %d）' % (len(src_txt.split('\n')), len(txt.split('\n')),
                                      len(src_txt.split('\n')) - len(txt.split('\n'))))
print('残留停车场符号：%s（应为空列表）' % (left,))
print('删除区块：' + ', '.join('%d-%d' % (i + 1, j + 1) for i, j in sorted(drops)))
try:
    compile(txt, SRC, 'exec')
    print('语法复核：SYNTAX_OK（compile() 直接检新文本，不写盘）')
except SyntaxError as e:
    raise SystemExit('!! 语法错：%s（行 %s）—— 不写文件' % (e.msg, e.lineno))
if '--dry' in sys.argv:
    print('DRY：未写文件')
    raise SystemExit(0)
if left:
    raise SystemExit('!! 还有残留符号，不写文件')
os.makedirs(ARCH, exist_ok=True)
bak = os.path.join(ARCH, 'vid_rec.py.pre_quarantine')
if not os.path.exists(bak):
    open(bak, 'w', encoding='utf-8').write(src_txt)
open(SRC, 'w', encoding='utf-8').write(txt)
print('已写入 %s（备份 %s）' % (SRC, bak))
