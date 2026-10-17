#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 respawn 实际发给 /spawn_entity 的那段 XML 落盘，用来看 model 级 pose 到底是什么。

为什么要单独落盘：#141/#142 两次都是"脚本打印说 XML 里写的是 6.80，车却生在 12.00"。
只打印一个 re.search 的结果不足以定案 —— 要看见发出去的那一整段，确认
(a) 被替换的确实是 model 级 pose 而不是别的，(b) 截取边界没把 world 里的旧 pose 行
留在第二个 <pose> 上，(c) 块里没藏 <include>（有的话 gzserver 会按 uri 自己取模型，
我们的 pose 就完全不算）。
"""
import sys

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from respawn_car6_136 import WORLD, MODEL, model_xml    # noqa: E402

OUT = '/mnt/d/desktop1/9.26.1/_live142/car6_block.xml'
xml = model_xml(WORLD, MODEL, '6.80 -12.80 6.70 0 0 0.0000')
open(OUT, 'w', encoding='utf-8').write(xml)
print('已存 %s（%d 字节）' % (OUT, len(xml)))
print('  含 <include>：%s｜含 <pose> 次数：%d' % ('<include' in xml, xml.count('<pose>')))
print('  前 3 行：')
for ln in xml.splitlines()[:3]:
    print('    %s' % ln[:110])
