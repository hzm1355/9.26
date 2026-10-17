#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 v2x_city_scene.world 里五辆车的 chassis 追加 plate_rear / plate_front 两个车牌面片。
幂等：已插入过的车会跳过。用 Edit 工具做 5 处相同插入容易误伤，故脚本化。
用法（Windows 或 WSL 下均可）：python add_plate_visuals.py"""
import os
import sys

W = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')

BLOCK = '''        <!-- 车牌面片：占位材质由 plate_gen_node 渲染，运行时由 v2x_plate 插件 SetMaterial 替换 -->
        <visual name="plate_rear">
          <pose>-0.81 0 0.15 0 0 3.14159265358979</pose>
          <geometry><box><size>0.02 0.44 0.14</size></box></geometry>
          <material><script>
            <uri>file:///tmp/v2x_plates/placeholder/placeholder.material</uri>
            <name>v2x/plate_placeholder</name>
          </script></material>
        </visual>
        <visual name="plate_front">
          <pose>0.81 0 0.15 0 0 0</pose>
          <geometry><box><size>0.02 0.44 0.14</size></box></geometry>
          <material><script>
            <uri>file:///tmp/v2x_plates/placeholder/placeholder.material</uri>
            <name>v2x/plate_placeholder</name>
          </script></material>
        </visual>
'''

ANCHOR = '        <visual name="cabin">'

src = open(W, encoding='utf-8').read()
if 'name="plate_rear"' in src:
    print('已插入过，跳过')
    sys.exit(0)

n = src.count(ANCHOR)
if n != 5:
    print('锚点数量异常: %d' % n)
    sys.exit(1)

src = src.replace(ANCHOR, BLOCK + ANCHOR)
open(W, 'w', encoding='utf-8', newline='\n').write(src)
print('已在 %d 辆车插入车牌面片' % n)
