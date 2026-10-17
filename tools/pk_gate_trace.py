#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抬/落杆过程逐帧采样（任务五条目 3 的字幕证据）。

条目 3 要"真实物理碰撞与升降动力学"，读片人能在画面上看到的只有两类东西：
杆在动（裁切后的 intersection_cam），和面板上的数字在跳。数字一跳不能证明那是
joint 实测值 —— 所以这里按固定间隔把 BarrierState 的原始字段打成一行行的表，
抬升段的角度序列、角速度、力矩一起出来：
  · 角度从 0° 单调爬到 90°、耗时 ≈ open_angle/velocity（yaml 里 1.5708/2.0 ≈ 0.79 s）
    → 动力学是插件按关节积分的，不是节点直接改 state；
  · 杆下压到车身时 effort_nm 顶到 effort 的九成以上而 angvel 归零 → 那才是"物理碰撞"，
    纯视觉的杆永远不会顶力矩。
一条消息都没收到时明确报错退出：把"没订阅到"读成"杆没动"就是假证据。

采样密度：抬/落一整趟只有 1~2 s 量级（yaml 里 velocity=2.0 rad/s，90° 理论 0.79 s，
再加 P/D 收敛），所以间隔取 0.1 s 才爬得出斜坡；但 16 s 窗口有上百拍，字幕装不下，
于是只打印"运动中的拍 + 状态/地感/目标角切换的那一拍"，静止段一行带过。
要看逐拍全表加 --all。

rclpy/v2x_msgs 是**用到时才 import** 的：--selftest 走的是纯离线合成数据，
不该因为没 source ROS 环境就跑不起来。

用法：
  pk_gate_trace.py --gate gate_in --dur 20 --stride 0.2 --on-change   # 等下令再开窗口
  pk_gate_trace.py --gate gate_out --dur 14 --all
  pk_gate_trace.py --selftest
"""
import argparse
import math
import sys
import time

STATE = {0: '落下', 1: '抬杆中', 2: '抬起', 3: '落杆中', 4: '故障'}
SRC = {0: '自动', 1: '手动', 2: '故障令'}


class Trace(object):
    """订阅 /parking/gate_state 并按仿真秒抽样。真正的类在 run_live() 里造，
    这里只留数据形状，方便 --selftest 复用同一套抽样/压缩代码。"""

    def __init__(self, gate, stride, on_change=False):
        self.gate, self.stride = gate, stride
        self.rows, self.n_seen, self.last = [], 0, None
        # --on-change：先把"这一道闸此刻的目标角"当基线，等它被下发新目标（抬或落）
        # 才开播采样窗口。car_6 一个循环 ~150 s，采样器若一上来就开播，16 s 窗口
        # 大概率整段落在空转上，字幕里就成了"角一直是 0"，看着像杆坏了。
        self.on_change = on_change
        self.armed = not on_change
        self.base_target = None
        self.armed_at = None

    def feed(self, gate_id, stamp_s, state, source, target_rad, angle_rad,
             angle_error_rad, angvel, effort, occupied, vehicle_id, elapsed):
        """一条 BarrierState 进来走这里（真订阅与自测共用同一段逻辑）。"""
        self.n_seen += 1
        if self.gate and gate_id != self.gate:
            return
        if not self.armed:
            if self.base_target is None:
                self.base_target = target_rad
                return
            if abs(target_rad - self.base_target) > 0.02:
                self.armed = True
                self.armed_at = stamp_s
                self.base_target = target_rad
            else:
                return
        if self.last is not None and stamp_s - self.last < self.stride:
            return                      # 按仿真秒抽样，不受墙钟抖动影响
        self.last = stamp_s
        self.rows.append((stamp_s, STATE.get(state, '?%d' % state),
                          SRC.get(source, '?%d' % source),
                          math.degrees(target_rad), math.degrees(angle_rad),
                          math.degrees(angle_error_rad), angvel, effort,
                          bool(occupied), vehicle_id, elapsed))


def pick(rows, want_all=False):
    """把上百拍压到字幕装得下的行数：运动中每一拍都留，静止段只留"变了"的那一拍。"""
    if want_all or len(rows) <= 14:
        return rows
    out = [rows[0]]
    for i in range(1, len(rows)):
        r, p = rows[i], rows[i - 1]
        if r[1] in ('抬杆中', '落杆中') or r[1] != p[1] or r[8] != p[8] \
                or abs(r[3] - p[3]) > 0.5 or r[9] != p[9]:
            out.append(r)
    return out if len(out) <= 22 else out[:11] + out[-11:]


def synth_rows():
    """离线自测用的合成一轮：静止 0.8 s → 0°→90°（1.5 s）→ 保持 → 车过 → 落回 0°（1.5 s）。
    20 Hz 造数据，只为验 pick 的压缩逻辑，不代表现场任何读数。"""
    t = Trace('gate_in', 0.0)          # 间隔 0：一拍不落的全收，模拟 20 Hz 原始流
    idle, ramp, hold = 16, 30, 140
    total = idle + ramp + hold + ramp
    ts, n = 0.0, 0
    while n < total:
        if n < idle:
            st, tgt, ang, occ, vid, el = 0, 0.0, 0.0, False, '', 0.0
        elif n < idle + ramp:
            k = (n - idle) / float(ramp)
            st, tgt, ang, occ, vid, el = 1, 1.5708, math.degrees(1.5708 * k), \
                n > idle + 4, 'car_6', k * 1.5
        elif n < idle + ramp + hold:
            past = n - idle - ramp
            occ = past < hold * 0.6
            st, tgt, ang, vid, el = 2, 1.5708, 90.0, 'car_6' if occ else '', past / 20.0
        else:
            k = (n - idle - ramp - hold) / float(ramp)
            st, tgt, ang, occ, vid, el = 3, 0.0, 90.0 * (1 - k), False, '', k * 1.5
        moving = abs(math.degrees(tgt) - ang) > 0.2      # tgt 是弧度、ang 已是角度
        t.feed('gate_in', ts, st, 0, tgt, math.radians(ang),
               tgt - math.radians(ang), 2.0 if moving else 0.0,
               240.0 if moving else 12.0, occ, vid, el)
        n += 1
        ts += 0.05
    return t.rows, t.n_seen


def report(gate, rows, n_seen, stride, base, kept_from):
    if not rows:
        print('!! %s 采样窗口里一行都没收到（话题共 %d 条）：窗口开在了别的闸机上？'
              % (gate, n_seen))
        return 3
    print('%s 采样 %d 行（话题共 %d 条、窗口内 %d 拍，间隔 %.2f 仿真秒，'
          '只留运动中/状态切换的拍）' % (gate, len(rows), n_seen, kept_from, stride))
    print('  t(s)  状态   令源 目标角 实测角   误差 角速度 力矩Nm 地感 车 本次耗时')
    for r in rows:
        print('  %6.2f %-4s %-4s %6.1f %6.1f %+6.1f %7.3f %6.1f  %s %-6s %5.2f' % (
            r[0] - base, r[1], r[2], r[3], r[4], r[5], r[6], r[7],
            '占用' if r[8] else '空闲', (r[9] or '-')[:6], r[10]))
    print('  实测角 %.1f° → %.1f°｜角速度峰值 %.3f rad/s｜力矩峰值 %.1f N·m' % (
        rows[0][4], rows[-1][4],
        max(abs(r[6]) for r in rows), max(r[7] for r in rows)))
    return 0


def run_live(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from v2x_msgs.msg import BarrierState

    t = Trace(a.gate, a.stride, a.on_change)

    class Sub(Node):
        def __init__(self):
            super().__init__('pk_gate_trace')
            # 发布侧是 RELIABLE depth=50，订阅侧必须同样 RELIABLE，否则 QoS 不匹配 = 零消息
            self.create_subscription(BarrierState, '/parking/gate_state',
                                     self.cb, QoSProfile(depth=50))

        def cb(self, m):
            t.feed(m.gate_id, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   int(m.state), int(m.source), float(m.target_rad), float(m.angle_rad),
                   float(m.angle_error_rad), float(m.angvel_rps), float(m.effort_nm),
                   bool(m.loop_occupied), m.vehicle_id, float(m.action_elapsed_s))

    rclpy.init()
    node = Sub()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < a.topic_wait and t.n_seen == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if t.n_seen == 0:
        print('!! %.1f s 内 /parking/gate_state 一条都没收到：parking_manager 没起来，'
              '或 QoS/域名不匹配（不是"杆没动"）' % a.topic_wait)
        node.destroy_node()
        rclpy.shutdown()
        return 2
    t1 = time.time()
    if a.on_change:
        while rclpy.ok() and not t.armed and time.time() - t1 < a.arm_timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not t.armed:
            print('!! 等了 %.0f s，%s 的目标角一直没变过（一直是 %.1f°）：'
                  '这一轮没有抬落动作，采样窗口没打开' % (
                      a.arm_timeout, a.gate, math.degrees(t.base_target or 0.0)))
            node.destroy_node()
            rclpy.shutdown()
            return 3
        t1 = time.time()
    base = t.rows[0][0] if t.rows else (t.armed_at or 0.0)
    while rclpy.ok() and time.time() - t1 < a.dur:
        rclpy.spin_once(node, timeout_sec=0.1)
    kept = len(t.rows)
    rc = report(a.gate, pick(t.rows, a.want_all), t.n_seen, a.stride, base, kept)
    node.destroy_node()
    rclpy.shutdown()
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gate', default='gate_in')
    ap.add_argument('--dur', type=float, default=20.0)
    ap.add_argument('--stride', type=float, default=0.1, help='抽样间隔（仿真秒）')
    ap.add_argument('--all', dest='want_all', action='store_true',
                    help='打印窗口内全部采样行（不做压缩）')
    ap.add_argument('--selftest', action='store_true',
                    help='不连 ROS，用合成的一轮抬落验 pick 的压缩逻辑')
    ap.add_argument('--on-change', dest='on_change', action='store_true',
                    help='等到这道闸被下发新目标角才开始采样')
    ap.add_argument('--arm-timeout', type=float, default=240.0)
    ap.add_argument('--topic-wait', type=float, default=6.0,
                    help='首条消息最长等待（DDS 发现要几秒）')
    a = ap.parse_args()

    if a.selftest:
        rows, n_seen = synth_rows()
        for stride in (0.1, 0.2):
            thinned = [r for i, r in enumerate(rows) if i % int(stride / 0.05) == 0]
            sel = pick(thinned, False)
            print('自测 stride=%.2f：合成 %d 拍 → 抽样 %d 行 → pick %d 行 ｜ '
                  '抬杆中 %d 行 落杆中 %d 行' % (
                      stride, len(rows), len(thinned), len(sel),
                      sum(1 for r in sel if r[1] == '抬杆中'),
                      sum(1 for r in sel if r[1] == '落杆中')))
        return report(a.gate, pick([r for i, r in enumerate(rows) if i % 2 == 0], False),
                      n_seen, 0.1, rows[0][0], len(rows))
    return run_live(a)


if __name__ == '__main__':
    sys.exit(main())
