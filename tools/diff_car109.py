#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#110 car_6 与 car_1 的模型描述做逐字 diff，把"六车本体逐字相同"从印象变成证据。

#85 走到现在只剩一个解释空间：同一套插件参数、同一套本体，车队能转、car_6 不能。那就必须
把两辆车的 XML 真Diff 一遍 —— 尤其是轮关节的 <axis>（符号反了会让"差速转向"变成"共模前进"，
正好对上观察到的：两轮同向、车只朝车头挪、yaw 不变）。

做法：解析 world，取每个 car_* 的 <model> 元素，规范化缩进后按行 difflib 对比；另外把
diff_drive 插件的参数表单独抽出来并排打，防"同名不同值"。
"""

import difflib
import os
import re
import sys
import xml.etree.ElementTree as ET

WORLD = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world'
BASE = 'car_1'
TARGETS = ['car_2', 'car_3', 'car_4', 'car_5', 'car_6']


def norm_lines(el):
    """规范化成可 diff 的行列表：去缩进差异、属性排序、每元素一行。"""
    out = []

    def walk(e, depth):
        attrs = ' '.join('%s="%s"' % (k, v) for k, v in sorted(e.attrib.items()))
        txt = re.sub(r'\s+', ' ', (e.text or '')).strip()
        tag = e.tag.split(':')[-1]
        if txt and not len(e):
            out.append('%s<%s%s>%s</%s>' % ('  ' * depth, tag, ' ' + attrs if attrs else '',
                                             txt, tag))
        else:
            out.append('%s<%s%s>%s' % ('  ' * depth, tag, ' ' + attrs if attrs else '',
                                        (' ' + txt) if txt else ''))
            for c in e:
                walk(c, depth + 1)
            out.append('%s</%s>' % ('  ' * depth, tag))
    walk(el, 0)
    return out


def main():
    root = ET.parse(WORLD).getroot()
    models = {}
    for m in root.iter('model'):
        nm = m.get('name')
        if nm:
            models[nm] = m
    if BASE not in models:
        print('!! 找不到 %s' % BASE)
        return 2
    base_lines = norm_lines(models[BASE])
    for t in TARGETS:
        if t not in models:
            print('!! 找不到 %s' % t)
            continue
        tl = norm_lines(models[t])
        # 只比"结构差异"：先把车名/namespace/frame 名这类必然不同的 token 归一化
        def canon(lines, car):
            out = []
            for s in lines:
                s = s.replace(car, '@CAR@').replace(BASE, '@CAR@')
                s = re.sub(r'@[0-9.]+@', '@V@', s)
                out.append(s)
            return out
        d = list(difflib.unified_diff(canon(base_lines, BASE), canon(tl, t),
                                      fromfile='%s(归一)' % BASE,
                                      tofile='%s(归一)' % t, lineterm='', n=2))
        print('\n===== %s vs %s：diff %d 行 =====' % (t, BASE, len(d)))
        lim = int(os.environ.get('DIFFLIM', '120'))
        for line in d[:lim]:
            print('  ' + line)
        if len(d) > lim:
            print('  ...（还有 %d 行）' % (len(d) - lim))

    print('\n===== 轴与关节专项（每车逐条列）=====')
    for car in [BASE] + TARGETS:
        m = models.get(car)
        if m is None:
            continue
        print('  --- %s ---' % car)
        for j in m.iter('joint'):
            axis = j.find('axis/xyz')
            dyn = j.find('axis/dynamics')
            lim = j.find('axis')
            print('    joint %-22s type=%-9s xyz=%-12s dyn=%s' % (
                j.get('name'), j.get('type'),
                (axis.text.strip() if axis is not None else '-'),
                (re.sub(r'\s+', ' ', ET.tostring(dyn, encoding='unicode'))
                 if dyn is not None else '-')))
            if lim is not None:
                for e in lim:
                    if e.tag != 'dynamics':
                        print('      axis/%s = %s' % (e.tag, re.sub(
                            r'\s+', ' ', ET.tostring(e, encoding='unicode')).strip()))
        for ln in m.iter('link'):
            for col in ln.findall('collision'):
                surf = col.find('surface')
                if surf is not None:
                    print('    col %-18s %s' % (col.get('name'), re.sub(
                        r'\s+', ' ', ET.tostring(surf, encoding='unicode')).strip()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
