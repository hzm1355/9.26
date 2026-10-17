#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 world 生成**静态碰撞盒足迹清单**，供 signal_drive_node 做几何避让。

背景：车道中心线两侧 1.4~2.0 m 就立着机动车灯/行人灯立杆（tools/world_collision_inventory.py
实测），车宽 0.8 m，沿中心线走必然刮杆。刮杆把车楔住打转，打转把车甩出车道，
再去穿路边那些根本没有碰撞的装饰设施 —— 所以第一步是让车在几何上就不碰杆。

只收"静态 + 与车体高度带相交 +  footprint 不是大地面"的盒；car_1..car_5 自己的碰撞盒
由跟车/冲突逻辑管，不写进清单。

用法（WSL，仓库根）：
  python3 tools/gen_obstacle_config.py [输出yaml]
默认写到 src/v2x_demo/config/obstacles.yaml。改 world 后需重跑本脚本并 colcon build。
"""
import contextlib
import io
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import world_collision_inventory as wci  # noqa: E402

OUT = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'obstacles.yaml')
# 车体占据的高度带：底盘碰撞 z 7.05~7.45（车在 reset_z 6.70 之上）
Z_LO, Z_HI = 7.05, 7.45
SKIP = ('room_33', 'ground_plane', 'sandbox_fixtures_coll')
# room_33/ground_plane：路面台体，不是"障碍物"
# sandbox_fixtures_coll：tools/gen_fixture_collisions.py 从沙盘网格补的 B 类等效盒，
#   盒面一律 >=2.0 m（> lane_guard 1.5），本来就在可行驶走廊之外。把它们写进避让清单
#   会让控制器为一条永远碰不到的路缘做外让，白白推翻 #75 已验证的控制行为；
#   它们只当物理兜底：车真擦到路缘时被顶住，而不是穿过去。


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUT
    # wci.main() 自己读 sys.argv 取 world 路径，本脚本的参数会串进去当成 world，先清掉
    argv_saved = sys.argv
    sys.argv = [argv_saved[0]]
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            wci.main()                   # 写默认 /tmp/world_collisions.json
    finally:
        sys.argv = argv_saved
    with open('/tmp/world_collisions.json', encoding='utf-8') as fh:
        boxes = json.load(fh)['boxes']

    rows, dropped = [], {'dynamic': 0, 'height': 0, 'floor': 0}
    for b in boxes:
        if b['name'] in SKIP:
            dropped['floor'] += 1
            continue
        if not b['static']:
            dropped['dynamic'] += 1
            continue
        a = b['aabb']
        if not (a[4] < Z_HI and a[5] > Z_LO):
            dropped['height'] += 1       # 高不到车底/低不过车顶，车从下面或上面过去
            continue
        if a[1] - a[0] > 8.0 and a[3] - a[2] > 8.0:
            dropped['floor'] += 1        # 大块平板（龙门架底板一类），当障碍会把路封死
            continue
        rows.append({'name': '%s/%s' % (b['name'], b['coll'] or b['link']),
                     'rect': [round(a[0], 3), round(a[2], 3),
                              round(a[1], 3), round(a[3], 3)],
                     'lane_dist': b['lane_dist'],
                     'z': [round(a[4], 2), round(a[5], 2)]})
    rows.sort(key=lambda r: r['lane_dist'])
    hdr = ('# 由 tools/gen_obstacle_config.py 从 v2x_city_scene.world 自动生成，勿手改。\n'
           '# rect = [x0, y0, x1, y1]（世界系外接矩形 footprint），lane_dist 仅便于人读。\n')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(hdr)
        yaml.safe_dump({'obstacles': rows}, fh, allow_unicode=True, sort_keys=False)
    print('写出 %d 个静态碰撞盒 -> %s' % (len(rows), out))
    print('剔除：动态(车体) %d、高度不重叠 %d、大地面/平板 %d' % (
        dropped['dynamic'], dropped['height'], dropped['floor']))
    near = [r for r in rows if r['lane_dist'] <= 3.0]
    print('其中距车道中心线 <=3.0 m 的 %d 个：' % len(near))
    for r in near:
        print('  %-32s lane_dist=%5.2f  rect x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f' % (
            r['name'], r['lane_dist'], r['rect'][0], r['rect'][2],
            r['rect'][1], r['rect'][3], r['z'][0], r['z'][1]))


if __name__ == '__main__':
    main()
