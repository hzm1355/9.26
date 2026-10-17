#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹记录/回放 合成自测（不依赖 Gazebo）：
  1) sample_at() 插值纯函数：端点夹紧 / 中间线性插值 / 空数据；
  2) 记录器 round-trip：喂入已知 ModelStates -> save -> 读回 JSON 断言内容；
  3) 记录->回放数据一致性：记录器保存的样本经 sample_at 复现原轨迹。
退出码 0=PASS, 1=FAIL。
"""
import json
import os
import sys
import tempfile

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trajectory_recorder as trec  # noqa: E402
import trajectory_player as tplay   # noqa: E402

_failures = []


def check(cond, msg):
    if not cond:
        _failures.append(msg)
        print(f'  FAIL: {msg}')
    else:
        print(f'  ok  : {msg}')


def approx(a, b, eps=1e-6):
    return abs(a - b) <= eps


def make_states(name, x, y, vx):
    ms = ModelStates()
    ms.name = [name, 'road_NS-main']           # 含一个非 v2x_ 干扰项
    p = Pose(); p.position = Point(x=x, y=y, z=0.4); p.orientation = Quaternion(w=1.0)
    ms.pose = [p, Pose()]
    t = Twist(); t.linear.x = vx
    ms.twist = [t, Twist()]
    return ms


def main():
    print('=== 1) sample_at 插值 ===')
    samples = [
        [0.0, 0.0, 0.0, 0.4, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1.0, 2.0, 0.0, 0.4, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [2.0, 4.0, 0.0, 0.4, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
    ]
    check(tplay.sample_at([], 1.0) is None, '空数据 -> None')
    s = tplay.sample_at(samples, -5.0); check(approx(s[0], 0.0), 't<起点 夹紧到首样本 x=0')
    s = tplay.sample_at(samples, 99.0); check(approx(s[0], 4.0), 't>终点 夹紧到末样本 x=4')
    s = tplay.sample_at(samples, 0.5);  check(approx(s[0], 1.0), 't=0.5 线性插值 x=1.0')
    s = tplay.sample_at(samples, 1.5);  check(approx(s[0], 3.0), 't=1.5 线性插值 x=3.0')
    s = tplay.sample_at(samples, 1.0);  check(approx(s[0], 2.0), 't=1.0 命中节点 x=2.0')

    print('=== 2) 记录器 round-trip ===')
    fd, path = tempfile.mkstemp(suffix='.json'); os.close(fd)
    rclpy.init(args=['--ros-args', '-p', f'output_file:={path}', '-p', 'min_interval:=0.0'])
    rec = trec.TrajectoryRecorder()
    # 喂入 5 个已知位姿（x 从 0 递增到 8），另含一个非车辆模型应被过滤
    for i in range(5):
        rec.on_states(make_states('v2x_test', float(i * 2), 3.0, 2.0))
    n = rec.save()
    check(n == 1, f'仅记录 v2x_ 车辆 (保存 {n} 辆)')
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    check('v2x_test' in data['vehicles'], 'JSON 含 v2x_test')
    check('road_NS-main' not in data['vehicles'], '非 v2x_ 模型被过滤')
    sm = data['vehicles']['v2x_test']['samples']
    check(len(sm) == 5, f'样本数=5 (实际 {len(sm)})')
    check(approx(sm[0][1], 0.0) and approx(sm[-1][1], 8.0), 'x 从 0 到 8')
    check(all(sm[i][0] <= sm[i + 1][0] for i in range(len(sm) - 1)), '时间戳单调不减')
    check(approx(sm[0][8], 2.0), '记录了线速度 vx=2.0')

    print('=== 3) 记录->回放一致性 ===')
    replay_samples = sm
    s0 = tplay.sample_at(replay_samples, replay_samples[0][0])
    sN = tplay.sample_at(replay_samples, replay_samples[-1][0])
    check(approx(s0[0], 0.0) and approx(s0[1], 3.0), '回放起点 = 记录起点 (0,3)')
    check(approx(sN[0], 8.0) and approx(sN[1], 3.0), '回放终点 = 记录终点 (8,3)')
    mid_t = (replay_samples[0][0] + replay_samples[-1][0]) / 2.0
    smid = tplay.sample_at(replay_samples, mid_t)
    check(0.0 <= smid[0] <= 8.0, f'中点插值 x 在区间内 ({smid[0]:.2f})')

    rec.destroy_node()
    rclpy.shutdown()
    os.unlink(path)

    print()
    if _failures:
        print(f'SELFTEST FAIL: {len(_failures)} 处不符')
        sys.exit(1)
    print('SELFTEST PASS: 轨迹记录/插值/回放一致性 全部正确')
    sys.exit(0)


if __name__ == '__main__':
    main()
