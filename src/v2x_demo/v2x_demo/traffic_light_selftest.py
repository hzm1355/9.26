#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
信号灯协调管理器合成自测（不依赖 Gazebo）：直接驱动相位数学，断言多路口联动正确。

覆盖：
  1) 基准组相位时序
  2) complementary 组严格反相
  3) 数值 offset 绿波
  4) preset 切换边界
  5) 行人信号独立时序（WALK→FLASHING→DONT_WALK）
  6) 行人灯与机动车灯的安全约束

退出码 0=PASS, 1=FAIL。
"""
import os
import sys
import tempfile

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traffic_light_manager as tlm  # noqa: E402

CFG = """
durations: {green: 10.0, yellow: 3.0, red: 10.0}
walk_duration: 5.0
flashing_duration: 3.0
preset: default
presets:
  default: {green: 10.0, yellow: 3.0, red: 10.0}
  peak:    {green: 14.0, yellow: 3.0, red: 14.0}
  offpeak: {green: 7.0,  yellow: 3.0, red: 7.0}
  night:   {green: 5.0,  yellow: 3.0, red: 5.0}
groups:
  J1_ns: {offset: 0.0, stop_x: 0.0, stop_y: -8.0, queue_radius: 25.0}
  J1_ew: {complementary: true, stop_x: 8.0, stop_y: 0.0, queue_radius: 25.0}
  J2_ns: {offset: 6.0, stop_x: 0.0, stop_y: 42.0, queue_radius: 25.0}
default_group: J1_ns
"""

_failures = []


def check(cond, msg):
    if not cond:
        _failures.append(msg)
        print(f'  FAIL: {msg}')
    else:
        print(f'  ok  : {msg}')


def main():
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False, encoding='utf-8') as f:
        f.write(CFG)
        cfg_path = f.name

    rclpy.init(args=['--ros-args', '-p', f'config_file:={cfg_path}'])
    node = tlm.TrafficLightManager()

    print('=== 1) 基准组 J1_ns 时序 (cycle=23: G[0,10) Y[10,13) R[13,23)) ===')
    check(node.phase_for('J1_ns', 0.0) == 'GREEN', 't=0 -> GREEN')
    check(node.phase_for('J1_ns', 9.9) == 'GREEN', 't=9.9 -> GREEN')
    check(node.phase_for('J1_ns', 10.0) == 'YELLOW', 't=10 -> YELLOW')
    check(node.phase_for('J1_ns', 12.9) == 'YELLOW', 't=12.9 -> YELLOW')
    check(node.phase_for('J1_ns', 13.0) == 'RED', 't=13 -> RED')
    check(node.phase_for('J1_ns', 22.9) == 'RED', 't=22.9 -> RED')
    check(node.phase_for('J1_ns', 23.0) == 'GREEN', 't=23 -> GREEN (回环)')

    print('=== 2) complementary 组 J1_ew 与 J1_ns 安全反相（永不同时绿）===')
    for t in [0.0, 1.0, 3.0, 5.0, 9.9, 11.0, 13.0, 15.0, 20.0, 22.0, 22.9]:
        a = node.phase_for('J1_ns', t)
        b = node.phase_for('J1_ew', t)
        check(not (a == 'GREEN' and b == 'GREEN'), f't={t}: 不同时绿 (J1_ns={a}, J1_ew={b})')
    greens_ew = [t for t in [x * 0.5 for x in range(46)] if node.phase_for('J1_ew', t) == 'GREEN']
    check(len(greens_ew) > 0, f'J1_ew 一个周期内确有绿灯 (采样 {len(greens_ew)} 点)')
    check(node.phase_for('J1_ew', 6.0) == 'RED', 't=6: J1_ns 绿中段时 J1_ew=RED')

    print('=== 3) 数值 offset 组 J2_ns 绿波延迟 6s ===')
    check(node.phase_for('J2_ns', 0.0) == 'GREEN', 'J2 t=0 -> GREEN (offset6 -> lt=6<10)')
    check(node.phase_for('J2_ns', 4.0) == 'YELLOW', 'J2 t=4 -> YELLOW (lt=10)')
    check(node.phase_for('J1_ns', 4.0) == 'GREEN', 'J1 t=4 -> GREEN (对照：仍绿)')
    check(node.phase_for('J2_ns', 10.0) == 'RED', 'J2 t=10 -> RED (lt=16)')

    print('=== 4) preset 切换改变周期边界（相位状态机，新配时从下一周期生效）===')
    node._apply_preset('peak', log=False)
    check(node.green == 14.0 and node.red == 14.0, 'peak: G=14 R=14')
    node.phase_state.clear()
    node.dur_latch.clear()
    check(node.phase_for('J1_ns', 0.0) == 'GREEN', 'peak t=0 -> GREEN')
    check(node.phase_for('J1_ns', 13.9) == 'GREEN', 'peak t=13.9 -> GREEN (13.9<14)')
    check(node.phase_for('J1_ns', 14.0) == 'YELLOW', 'peak t=14 -> YELLOW')
    check(node.phase_for('J1_ns', 16.9) == 'YELLOW', 'peak t=16.9 -> YELLOW')
    check(node.phase_for('J1_ns', 17.0) == 'RED', 'peak t=17 -> RED (17>=14+3)')
    check(node.phase_for('J1_ns', 30.9) == 'RED', 'peak t=30.9 -> RED')
    check(node.phase_for('J1_ns', 31.0) == 'GREEN', 'peak t=31 -> GREEN (cycle=31 回环)')

    print('=== 4b) 周期中途换预设：绝不跳黄灯/跳绿灯（真实时序）===')
    node.phase_state.clear()
    node.dur_latch.clear()
    node._apply_preset('default', log=False)
    illegal = {('GREEN', 'RED'), ('RED', 'YELLOW'), ('YELLOW', 'GREEN')}
    prev = node.phase_for('J1_ns', 0.0)
    skips, changes = [], []
    for i in range(1, 1601):
        t = i * 0.05
        if abs(t - 5.0) < 1e-9:
            node._apply_preset('peak', log=False)      # 绿灯进行中切 peak
            changes.append('default->peak@5.0')
        if abs(t - 18.0) < 1e-9:
            node._apply_preset('offpeak', log=False)   # 红灯进行中切 offpeak
            changes.append('peak->offpeak@18.0')
        p = node.phase_for('J1_ns', t)
        if p != prev:
            if (prev, p) in illegal:
                skips.append(f't={t:.2f}: {prev}->{p}')
            prev = p
    check(len(changes) == 2, f'两次预设中途切换均已执行 ({changes})')
    check(not skips, f'1600 步推进无非法相变（黄灯始终保留）: {skips[:4]}')
    node.phase_state.clear()
    node.dur_latch.clear()
    seq = []
    prev = node.phase_for('J1_ns', 0.0)
    seq.append(prev)
    for i in range(1, 1601):
        p = node.phase_for('J1_ns', i * 0.05)
        if p != prev:
            seq.append(p)
            prev = p
    pairs = list(zip(seq, seq[1:]))
    check(all(pr not in illegal for pr in pairs),
          f'相位序列连续合法: {seq[:9]} ... 共 {len(seq)} 次切换')
    check(('GREEN', 'YELLOW') in pairs and ('YELLOW', 'RED') in pairs,
          '中途换预设后黄灯仍然出现（未漏灯）')

    print('=== 5) 行人信号独立时序 ===')
    node._apply_preset('default', log=False)
    ped = node.pedestrian
    # 机动车 GREEN → 行人 DONT_WALK
    check(ped.get_phase('GREEN', 0.0) == 'DONT_WALK', '车绿 -> 行人 DONT_WALK')
    check(ped.get_phase('YELLOW', 0.0) == 'DONT_WALK', '车黄 -> 行人 DONT_WALK')
    # 机动车 RED 期间 → 行人子周期
    check(ped.get_phase('RED', 0.0) == 'WALK', '车红 t=0 -> 行人 WALK')
    check(ped.get_phase('RED', 4.9) == 'WALK', '车红 t=4.9 -> 行人 WALK (walk=5s)')
    check(ped.get_phase('RED', 5.0) == 'FLASHING', '车红 t=5 -> 行人 FLASHING')
    check(ped.get_phase('RED', 7.9) == 'FLASHING', '车红 t=7.9 -> 行人 FLASHING (flash=3s)')
    check(ped.get_phase('RED', 8.0) == 'DONT_WALK', '车红 t=8 -> 行人 DONT_WALK')
    check(ped.get_phase('RED', 9.9) == 'DONT_WALK', '车红 t=9.9 -> 行人 DONT_WALK')

    print('=== 6) 行人-机动车安全约束 ===')
    for t in [x * 0.5 for x in range(46)]:
        veh = node.phase_for('J1_ns', t)
        elapsed_red = node._elapsed_in_red('J1_ns', t)
        ped_phase = ped.get_phase(veh, elapsed_red)
        if veh == 'GREEN':
            check(ped_phase == 'DONT_WALK',
                  f't={t}: 车绿时行人必禁行 (ped={ped_phase})')
        if ped_phase == 'WALK':
            check(veh == 'RED',
                  f't={t}: 行人WALK时车必红 (veh={veh})')

    node.destroy_node()
    rclpy.shutdown()
    os.unlink(cfg_path)

    print()
    if _failures:
        print(f'SELFTEST FAIL: {len(_failures)} 处不符')
        sys.exit(1)
    print('SELFTEST PASS: 信号灯多路口相位联动/绿波/预设切换/行人时序 全部正确')
    sys.exit(0)


if __name__ == '__main__':
    main()
