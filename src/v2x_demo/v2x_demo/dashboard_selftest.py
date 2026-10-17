#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v2x_dashboard 数据层无头自测（不创建 Qt 界面，仅验证 DashboardNode 的配置装载、
消息解析与命令构造）。用 QT_QPA_PLATFORM=offscreen 保证无显示环境也能导入 PyQt5。
"""
import os
import sys
import tempfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import Pose, Point, Quaternion  # noqa: E402
from gazebo_msgs.msg import ModelStates  # noqa: E402
from std_msgs.msg import String  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from v2x_dashboard import DashboardNode, _yaw_from_quat  # noqa: E402

FAILS = []


def check(cond, label):
    print(('  PASS ' if cond else '  FAIL ') + label)
    if not cond:
        FAILS.append(label)


def make_cfg(tmp):
    scene = {
        'intersections': {
            'J1': {'pose': [0, 0], 'type': 'cross', 'label': '主路口'},
            'N1': {'pose': [0, 30], 'type': 'endpoint', 'label': '北'},
        },
        'road_segments': {
            'NS-north': {'from_node': 'J1', 'to_node': 'N1', 'length': 30.0,
                         'total_width': 12.8, 'speed_limit': 30, 'label': 'NS'},
        },
        'crosswalks': [{'id': 'CW1', 'node': 'J1', 'center': [0.0, 9.5]}],
        'traffic_lights': [
            {'id': 'TL-NE', 'pose': [6.5, 6.5], 'group': 'J1_ns'},
            {'id': 'TL-NW', 'pose': [-6.5, 6.5], 'group': 'J1_ew'},
        ],
    }
    regions = {'regions': [
        {'id': 'ETC-01_zone', 'type': 'etc_gate', 'center': [0, -8, 1.0], 'size': [14.0, 3.0, 3.0]},
        {'id': 'SPEED-A', 'type': 'speed_point', 'center': [0, -24, 1.0], 'size': [12.8, 0.5, 3.0]},
    ]}
    tl = {'presets': {'default': {}, 'peak': {}, 'night': {}}, 'preset': 'default',
          'groups': {'J1_ns': {'offset': 0.0}, 'J1_ew': {'complementary': True}}}
    paths = {}
    for name, data in (('scene', scene), ('regions', regions), ('tl', tl)):
        p = os.path.join(tmp, f'{name}.yaml')
        with open(p, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True)
        paths[name] = p
    return paths


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
    tmp = tempfile.mkdtemp()
    p = make_cfg(tmp)
    node = DashboardNode()
    # 用测试配置重载（覆盖默认 share 路径）
    node.nodes.clear(); node.segments.clear(); node.crosswalks.clear()
    node.lights.clear(); node.regions.clear()
    node._load_scene(p['scene'])
    node._load_regions(p['regions'])
    node._load_tl_params(p['tl'])

    print('[1] 配置装载')
    check(len(node.nodes) == 2, f'nodes==2 (got {len(node.nodes)})')
    check(len(node.segments) == 1, f'segments==1 (got {len(node.segments)})')
    check(abs(node.segments[0]['width'] - 12.8) < 1e-6, 'segment width=12.8')
    check(len(node.lights) == 2, f'lights==2 (got {len(node.lights)})')
    check(len(node.regions) == 2, f'regions==2 (got {len(node.regions)})')
    check(set(node.presets) == {'default', 'peak', 'night'}, f'presets={node.presets}')
    check(set(node.groups) == {'J1_ns', 'J1_ew'}, f'groups={node.groups}')

    print('[2] ModelStates 解析')
    ms = ModelStates()
    ms.name = ['v2x_car_n1', 'playback_v2x_car_n1', 'ground_plane', 'v2x_prius']
    def pose(x, y, yaw=0.0):
        import math
        q = Quaternion(); q.z = math.sin(yaw / 2); q.w = math.cos(yaw / 2)
        return Pose(position=Point(x=float(x), y=float(y), z=0.2), orientation=q)
    ms.pose = [pose(1.5, -10.0), pose(2.5, -11.0), pose(0, 0), pose(0, 0, 1.5708)]
    node.on_model_states(ms)
    check('v2x_car_n1' in node.vehicles, 'real car tracked')
    check('v2x_prius' in node.vehicles, 'prius tracked')
    check('ground_plane' not in node.vehicles, 'non-v2x ignored')
    check('v2x_car_n1' in node.playback, 'playback ghost separated')
    check('playback_v2x_car_n1' not in node.vehicles, 'ghost not in vehicles')
    check(abs(node.vehicles['v2x_car_n1']['x'] - 1.5) < 1e-6, 'x parsed')
    check(abs(node.vehicles['v2x_prius']['yaw'] - 1.5708) < 1e-2,
          f"yaw parsed ({node.vehicles['v2x_prius']['yaw']:.3f})")

    print('[3] 灯态订阅')
    m = String(); m.data = 'green'
    node.on_light('J1_ns', m)
    check(node.light_state.get('J1_ns') == 'GREEN', f'light upper-stored ({node.light_state})')

    print('[4] 区域事件回调')
    from v2x_msgs.msg import RegionEvent
    ev = RegionEvent()
    ev.region_id = 'ETC-01_zone'; ev.vehicle_id = 'v2x_car_n1'
    ev.event_type = RegionEvent.EVENT_ENTER; ev.speed = 3.4
    got = []
    node.event_listeners.append(lambda e: got.append(e))
    node.on_region_event(ev)
    check(len(node.events) == 1 and len(got) == 1, 'event appended + listener fired')
    check(got[0][3] == 'ENTER', f'kind=ENTER ({got[0][3]})')

    print('[5] 命令构造（不触发网络）')
    # cmd_preset 只发布话题，安全
    node.cmd_preset('peak')
    check(True, 'cmd_preset published without error')

    node.destroy_node()
    rclpy.shutdown()
    print('\n' + ('DASHBOARD_SELFTEST FAILED: ' + str(FAILS) if FAILS else 'DASHBOARD_SELFTEST PASSED'))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
