#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读观察 car_6 的位姿/实测速度 + 两道闸机状态，用来分清"车在滑"和"车在听指令"。

不下发任何 cmd_vel、不调任何服务，纯订阅。

位姿取 /model_states（gzserver 权威，和 rearm_by_driving136 同一口径；/car_6/odom 当前
根本不在话题表里）。速度不是读 odom 的 twist，而是用 /clock 驱动的位姿差分 —— 墙钟在
这台机器上不可信（RTF≈0.8），按墙钟差分会把速度算高 ~1.25 倍，按仿真秒才对得上插件里
那个"@ x.x m/s"。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_msgs')
from v2x_msgs.msg import BarrierState

MODEL = 'car_6'
GATE_IN_FOCUS = (12.00, -11.21)
GATE_OUT_FOCUS = (6.80, -10.95)
STATE_NAME = {0: 'CLOSED', 1: 'OPENING', 2: 'OPEN', 3: 'CLOSING', 4: 'FAULT'}


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class Observe(Node):
    def __init__(self):
        super().__init__('observe_car6_139')
        self.q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pose = None
        self.clock = None
        self.gates = {}
        self.moves = 0
        self.create_subscription(ModelStates, '/model_states', self.on_ms, self.q)
        self.create_subscription(Clock, '/clock', self.on_clock, self.q)
        for topic in ('/parking/gate/gate_in/state', '/parking/gate/gate_out/state',
                      '/parking/gate_state'):
            self.create_subscription(BarrierState, topic, self.on_gate, self.q)

    def on_ms(self, m):
        if MODEL not in m.name:
            return
        p = m.pose[m.name.index(MODEL)]
        v = m.twist[m.name.index(MODEL)]
        self.pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation),
                    math.hypot(v.linear.x, v.linear.y))

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_gate(self, msg):
        self.gates[(msg.header.frame_id or '?', msg.gate_id)] = msg

    def run(self, secs):
        t0 = time.monotonic()
        prev = None
        got_pose = got_clock = 0
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None or self.clock is None:
                continue
            if prev is None:
                prev = (self.clock, self.pose)
                continue
            dt = self.clock - prev[0]
            if dt <= 1e-6:
                continue
            got_pose += 1
            got_clock += 1
            x0, y0, _z0, _w0, _v0 = prev[1]
            x1, y1, z1, w1, v1 = self.pose
            d = math.hypot(x1 - x0, y1 - y0) / dt
            prev = (self.clock, self.pose)
            if got_pose % 10 != 1:
                continue
            gin = math.hypot(x1 - GATE_IN_FOCUS[0], y1 - GATE_IN_FOCUS[1])
            gout = math.hypot(x1 - GATE_OUT_FOCUS[0], y1 - GATE_OUT_FOCUS[1])
            txt = '｜'.join('%s %s ang=%+.2f tgt=%+.2f fault=%d loop=%d' % (
                gid, STATE_NAME.get(st.state, st.state), st.angle_rad, st.target_rad,
                st.fault, int(st.loop_occupied))
                for (_ns, gid), st in sorted(self.gates.items()))
            print('  (%+.2f,%+.2f) yaw=%+.2f z=%.3f ｜|v|=%.2f 位移差分=%.2f ｜in %.2f out %.2f｜%s' % (
                x1, y1, w1, z1, v1, d, gin, gout, txt), flush=True)
            time.sleep(0.4)
        print('  观察 %d s：位姿帧=%d 时钟帧=%d 闸机路数=%d ｜话题表里有 model_states=%s' % (
            secs, got_pose, got_clock, len(self.gates), self.pose is not None), flush=True)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    Observe().run(secs)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
