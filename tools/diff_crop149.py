#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两帧截图相减，把变化最大的那块区域标出来 —— 用来客观回答"客户端到底有没有在画 car_6"。

为什么要相减而不是肉眼盯：远景里 car_6 只有二十来个像素，肉眼在压缩过的截图上看不出来，
"看不到"和"客户端掉了 visual"两件事就分不开。让车原地转 2 s（不平移、零碰撞风险），
如果客户端在画它，那一块像素必然变；把变化点框出来就是它的屏幕位置。
用法：diff_crop149.py <A.png> <B.png> <输出.png> [x0 y0 x1 y1]
"""
import sys

from PIL import Image, ImageChops

a_path, b_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
box = [int(v) for v in sys.argv[4:8]] if len(sys.argv) >= 8 else None
A = Image.open(a_path).convert('RGB')
B = Image.open(b_path).convert('RGB')
d = ImageChops.difference(A, B)
if box:
    d = d.crop(box)
g = d.convert('L')
bb = g.getbbox()
# 逐像素阈值：状态栏的时钟每帧都在跳，亮度差 >40 才算"画面内容变了"
pts = [(x, y) for y in range(0, g.size[1], 2) for x in range(0, g.size[0], 2) if g.getpixel((x, y)) > 40]
print('差图尺寸 %dx%d｜bbox=%s｜超阈值采样点 %d 个' % (g.size[0], g.size[1], bb, len(pts)))
if pts:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    print('变化范围 x %d..%d, y %d..%d（%s 坐标系）' % (min(xs), max(xs), min(ys), max(ys),
          '裁切框内' if box else '全屏'))
d.convert('RGB').save(out)
print('已存差图 %s' % out)
