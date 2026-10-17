#!/usr/bin/env python3
# 解析 gazebo_models-master 各模型 .dae mesh 的顶点包围盒，得到实际尺寸（单位：模型自身，通常米）
import re
import sys

BASE = '/mnt/d/desktop1/9.26.1/gazebo_models-master/'
MODELS = ['school', 'office_building', 'gas_station', 'law_office',
          'oak_tree', 'pine_tree', 'house_1', 'stop_light']

pat_pos = re.compile(r'<float_array[^>]*positions[^>]*>([\s\S]*?)</float_array>')
pat_num = re.compile(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?')


def mesh_bbox(path):
    with open(path, encoding='utf-8', errors='ignore') as f:
        xml = f.read()
    m = pat_pos.search(xml)
    if not m:
        # 有的 dae 把坐标写在别处，直接全文抓 float_array 太泛；尝试所有 array
        arrays = re.findall(r'<float_array[^>]*>([\s\S]*?)</float_array>', xml)
        best = None
        for a in arrays:
            nums = [float(x) for x in pat_num.findall(a)]
            if len(nums) >= 24 and len(nums) % 3 == 0:
                xs = nums[0::3]; ys = nums[1::3]; zs = nums[2::3]
                bb = (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))
                size = (bb[3]-bb[0]) * (bb[4]-bb[1]) * (bb[5]-bb[2])
                if best is None or size > best[0]:
                    best = (size, bb)
        return best[1] if best else None
    nums = [float(x) for x in pat_num.findall(m.group(1))]
    xs = nums[0::3]; ys = nums[1::3]; zs = nums[2::3]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


for name in MODELS:
    try:
        with open(f'{BASE}{name}/model.sdf', encoding='utf-8') as f:
            sdf = f.read()
        uris = re.findall(r'<uri>(model://[^<]+\.dae)</uri>', sdf)
        seen = set()
        for u in uris:
            if u in seen:
                continue
            seen.add(u)
            p = BASE + u.replace('model://', '').strip() + ''
            p = BASE + u[len('model://'):]
            bb = mesh_bbox(p)
            if bb:
                w, d, hgt = bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2]
                print(f'{name:16s} {u}: {w:7.2f} x {d:6.2f} x {hgt:6.2f}  '
                      f'(origin-corner x[{bb[0]:.1f},{bb[3]:.1f}] y[{bb[1]:.1f},{bb[4]:.1f}] z[{bb[2]:.1f},{bb[5]:.1f}])')
            else:
                print(f'{name:16s} {u}: 未解析到顶点')
    except FileNotFoundError as e:
        print(f'{name:16s} 缺文件: {e}')
