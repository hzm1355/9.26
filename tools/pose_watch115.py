#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#115b 静置观察：无人指令时 car_6 到底会不会自己动（0.23 m 的漂移要归因）。

只订阅不发布。每 8 拍打一行，按 /clock 的仿真钟算真值速度；同时把 /car_6/cmd_vel 的到达情况
数出来——如果没人发 cmd_vel 而车在走，那就是物理（被顶/被坡）；如果有 cmd_vel 到达，就是还有
发布者没杀干净。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import yaw_of, wrap, BEST   # noqa: E402
import rclpy                                   # noqa: E402
from rclpy.node import Node                      # noqa: E402
from geometry_msgs.msg import Twist              # noqa: E402
from rosgraph_msgs.msg import Clock              # noqa: E402
from gazebo_msgs.msg import ModelStates          # noqa: E402

CAR = 'car_6'


class W(Node):
    def __init__(self):
        super().__init__('pose_watch115')
        self.clock = 0.0
        self.pose = None
        self.prev = None
        self.n_ms = 0
        self.n_cmd = 0
        self.cmd_seen = []
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        self.create_subscription(Twist, '/car_6/cmd_vel', self.on_cmd, 50)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.n_cmd += 1
        if len(self.cmd_seen) < 6:
            self.cmd_seen.append('v=%+.2f w=%+.2f' % (m.linear.x, m.angular.z))

    def on_s(self, m):
        if CAR not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(CAR)
        p = m.pose[i]
        self.pose = (self.clock, p.position.x, p.position.y,
                     yaw_of(p.orientation), p.position.z)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    n = W()
    t0 = time.time()
    last_print = 0.0
    peak = 0.0
    dacc = 0.0
    anchor = None
    k = 0
    while time.time() - t0 < secs:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.pose is None:
            continue
        if anchor is None:
            anchor = n.pose
        if n.prev is not None:
            dt = n.pose[0] - n.prev[0]
            if dt > 1e-9:
                tv = math.hypot(n.pose[1] - n.prev[1], n.pose[2] - n.prev[2]) / dt
                peak = max(peak, tv)
                dacc += tv * dt
                n.prev = n.pose
                k += 1
                if time.time() - last_print > 1.0:
                    last_print = time.time()
                    print('  仿真t=%9.3f (%7.3f,%7.3f) z=%6.4f yaw=%+.4f 真值|v|=%+.3f' % (
                        n.pose[0], n.pose[1], n.pose[2], n.pose[4], n.pose[3], tv))
        elif n.prev is None:
            n.prev = n.pose
    span = n.pose[0] - anchor[0] if n.pose and anchor else 0.0
    print('  窗口内：model_states %d 条 / cmd_vel 到达 %d 条 / 差分 %d 拍' % (
        n.n_ms, n.n_cmd, k))
    print('  仿真时长 %.2f s，累积路径 %.3f m，峰值真值|v| %.3f m/s，'
          '首末位移 %.3f m，首末转角 %+0.4f rad' % (
              span, dacc, peak,
              math.hypot(n.pose[1] - anchor[1], n.pose[2] - anchor[2]) if n.pose else 0.0,
              wrap(n.pose[3] - anchor[3]) if n.pose else 0.0))
    if n.cmd_seen:
        print('  到达的 cmd_vel 前 6 条：%s' % ' | '.join(n.cmd_seen))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
