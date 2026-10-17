#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从安装目录的信号灯参数文件派生一份"联动关系写死在配置里"的临时配置。

   为什么需要它：管理器只在**播种**时读 offset（phase_state 已存在就不会重播种），
   运行中用 /traffic_light/config 改 offset 对正在跑的组不生效，
   还会带上"播种那一刻的配时"残留的相位误差。所以要干净地量联动关系，
   只能把 offset / complementary / anti_phase_of 与配时一起放进配置文件，再重启管理器。

   用法：python3 mk_link_cfg.py [输出路径=/tmp/tl_link.yaml]
"""
import sys

import yaml

SRC = ('/home/hzm/v2x_ws/install/v2x_demo/share/v2x_demo/config/'
       'traffic_light_params.yaml')
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/tl_link.yaml'

# 6 组统一 8/2/8（周期 18 s），只保留联动关系差异
DUR = {'green': 8.0, 'yellow': 2.0, 'red': 8.0}
LINK = {
    'J1_ns': {'offset': 0.0},
    'J1_ew': {'complementary': True},
    'J2_ns': {'offset': 0.0},
    'J2_ew': {'anti_phase_of': 'J2_ns'},
    'J3_ns': {'offset': 3.0},
    'J3_ew': {'anti_phase_of': 'J3_ns'},
}

cfg = yaml.safe_load(open(SRC, encoding='utf-8'))
cfg['durations'] = dict(DUR)
cfg['presets'] = {'default': dict(DUR)}
cfg['preset'] = 'default'
cfg['time_schedule'] = {'enabled': False, 'slots': []}
for gid, rel in LINK.items():
    g = cfg['groups'].setdefault(gid, {})
    g.update(DUR)
    g.update(rel)
    cfg['groups'][gid] = g

with open(OUT, 'w', encoding='utf-8') as f:
    yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
print(f'  已生成 {OUT}')
for gid in LINK:
    print(f'    {gid:<7} {cfg["groups"][gid]}')
