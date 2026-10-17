#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验 v2x_city_scene.world：能否解析、卡口模型与插件是否齐全。"""
import os
import sys
import xml.etree.ElementTree as E

W = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 '..', 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')
t = E.parse(W)
names = [m.get('name') for m in t.iter('model')]
print('XML 可解析，模型总数 =', len(names))

need = ['gate_j3_north', 'gate_j2_south', 'plate_manager']
for g in need:
    print('  %-16s %s' % (g, 'OK' if g in names else '缺失!'))

ok = True
for m in t.iter('model'):
    for s in m.iter('sensor'):
        if s.get('name') != 'gate_camera':
            continue
        plugins = [p.get('filename') for p in s.iter('plugin')]
        cam = s.find('camera/image')
        wh = {c.tag: c.text for c in cam} if cam is not None else {}
        print('  %s: %sx%s 插件=%s' % (m.get('name'), wh.get('width'),
                                       wh.get('height'), plugins))
        if 'libv2x_gate.so' not in plugins or 'libgazebo_ros_camera.so' not in plugins:
            ok = False

for m in t.iter('model'):
    if m.get('name') in need[:2]:
        pose = m.find('pose')
        print('  %s pose = %s' % (m.get('name'), pose.text))

print('校验结果:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
