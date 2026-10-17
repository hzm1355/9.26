#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""体检 pk175 的 SDF 解析：world 里每个 <model> 是否都被配对上、碰撞件是否都进了表。
（正则会静默漏件，漏一个就等于净空判据少一根柱子，必须先数对。）"""
import re

W = 'src/smart_car_sim/worlds/v2x_city_scene.world'
txt = open(W, encoding='utf-8', errors='replace').read()

names = re.findall(r'<model name="([^"]+)">', txt)
bl = [m.group(1) for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S)]
print('<model name=> 出现 %d 次；成对解析出 %d 个；</model> 共 %d 个；<model 共 %d 个'
      % (len(names), len(bl), txt.count('</model>'), len(re.findall(r'<model\b', txt))))
print('漏掉/重名的: %s' % (sorted(set(names) - set(bl)) or '无'))
for n in sorted(set(names)):
    if names.count(n) != 1:
        print('  重名 model: %s 出现 %d 次' % (n, names.count(n)))
print('解析到的 model: %s' % ' '.join(bl))

# 逐 model 数碰撞件个数，和原文里 <collision 的个数对齐
for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S):
    mname, mbody = m.group(1), m.group(2)
    nraw = len(re.findall(r'<collision\b', mbody))
    npars = 0
    for lm in re.finditer(r'<link name="([^"]+)">(.*?)</link>', mbody, re.S):
        npars += len(re.findall(r'<collision name="([^"]+)">', lm.group(2)))
    if nraw != npars:
        print('  对不上 %-28s 原文 collision %d / 解析进 link 内 %d' % (mname, nraw, npars))
