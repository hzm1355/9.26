#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线合成数据自测：只验 speed_fleet_evidence.py 的采样/真值算术，不碰 ROS 现场。

为什么要有这个文件：预检用的"各车中位速度"曾经按相邻两帧位姿差去除以一个时钟格，
分子是 1 ms 的位移、分母是 100 ms，速度缩成 1/10，于是巡航 2.52 km/h 报成 0.25 km/h，
整轮取证在预检处退出。这个坑靠"看现场日志"要等 120 s 预热才暴露一次，合成数据一秒出结果。

用法：python3 speed_fleet_selftest.py
"""
import importlib.util
import math
import os
import types

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'speed_fleet_evidence', os.path.join(HERE, 'speed_fleet_evidence.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

Fleet = mod.Fleet
POSE_HZ = 97.0
CLOCK_HZ = 9.8


def mk_obj():
    """绕过 rclpy.init 造一个裸 Fleet，只填回调要用的字段。"""
    f = object.__new__(Fleet)
    f.sim_t = None

    f.latest = {}
    f.tick = {}
    f.speeds = {}
    f.cross = {}
    return f


def pose_msg(name, x, y):
    p = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y),
                              orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    return types.SimpleNamespace(name=[name], pose=[p], twist=[None])


def clock_msg(t):
    return types.SimpleNamespace(clock=types.SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9)))


def main():
    ok = True

    # 1) 直线巡航 0.7 m/s：中位速度必须落在 2.52 km/h 附近（±10%），不是 1/10
    f = mk_obj()
    t_pose = 0.0
    for k in range(int(20 * CLOCK_HZ)):
        tc = k / CLOCK_HZ
        # 每个时钟格里插 9 帧位姿，最后那帧才是 latest
        for j in range(int(POSE_HZ / CLOCK_HZ)):
            t_pose = tc + j / POSE_HZ
            Fleet.on_states(f, pose_msg('car_9', 0.7 * t_pose, 0.0))
        Fleet.on_clock(f, clock_msg(tc))
    v = Fleet.car_median_kmh(f, 'car_9')
    print('  巡航 0.70 m/s -> 中位 %.2f km/h（期望 2.52）' % v)
    ok = ok and abs(v - 2.52) < 0.26

    # 2) 断面穿越计时：以 0.7 m/s 穿过 span=11 m 的两断面，独立真值应 ≈ 2.52 km/h
    f2 = mk_obj()
    zid = 'SZ-A-south-segment'
    z = mod.ZONES[zid]
    span = z['span']
    t = 0.0
    x0 = z['u1'] - 2.0 * z['sign']          # 从第一个断面前 2 m（沿通行方向）起步
    n = int((span + 4.0) / (0.7 * (1.0 / POSE_HZ)))
    for i in range(n):
        t = i / POSE_HZ
        x = x0 + 0.7 * t * z['sign']
        Fleet.on_states(f2, pose_msg('car_9', x, 0.0))
        if i % int(POSE_HZ / CLOCK_HZ) == 0:
            Fleet.on_clock(f2, clock_msg(t))
    ev_t = (span + 2.0) / 0.7              # 事件时刻=出口断面通过时刻（节点就是在这儿发区间结果）
    print('  断面穿越记录 %s' % (f2.cross.get(zid, {}).get('car_9'),))
    tv, tt = Fleet.truth(f2, zid, 'car_9', ev_t)
    print('  断面穿越 -> 独立真值 %.2f km/h / %.2f s（span=%.1f m，期望 2.52 / %.2f）'
          % (tv, tt, span, span / 0.7))
    ok = ok and abs(tv - 2.52) < 0.10 and abs(tt - span / 0.7) < 0.20

    # 3) 单帧跳变不得拖动中位数：正常巡航里掺一格 5 m 的复位跳变
    f3 = mk_obj()
    tt_ = 0.0
    x = 0.0
    for k in range(300):
        tt_ = k / CLOCK_HZ
        Fleet.on_states(f3, pose_msg('car_9', x, 0.0))
        Fleet.on_clock(f3, clock_msg(tt_))
        x += 0.7 / CLOCK_HZ
        if k == 150:                        # 复位：位置瞬移 5 m，下一格照常巡航
            x += 5.0
    v3 = Fleet.car_median_kmh(f3, 'car_9')
    jump = max(v for _, v in f3.speeds['car_9'])
    print('  掺入 5 m 跳变（单格峰值 %.1f m/s）-> 中位 %.2f km/h（期望仍是 2.52）' % (jump, v3))
    ok = ok and abs(v3 - 2.52) < 0.26

    print('自检 %s' % ('通过' if ok else '不通过'))
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
