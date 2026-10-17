#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""正/反号 cmd 探针：把车放到一条长直净空车道上，先后发 +v 和 -v，看它到底往哪走。

为什么要这一发：curb_push_test 的"反向对照"里，车在离盒 1.7 m 处收到 -0.7 m/s
指令后反而朝盒面加速到 2.78 m/s（位移 +1.65 m）。要么这模型的差速插件不认负速，
要么顶推测试的判据本身在骗人。用长直路段把两种可能分开。

用法：python3 tools/cmd_sign_probe.py [车名] [x] [y] [每段秒数]
"""
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

CAR = sys.argv[1] if len(sys.argv) > 1 else 'car_4'
PX = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
PY = float(sys.argv[3]) if len(sys.argv) > 3 else -14.6
SEC = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
V = 0.7
Z = 6.70


class Probe(Node):

    def __init__(self):
        super().__init__('cmd_sign_probe')
        self.sim_t = 0.0
        self.samples = []
        self.latest = {}
        self.create_subscription(ModelStates, '/model_states', self._on_states,
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._on_clock,
                                 qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, 10)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

    def _on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_states(self, msg):
        if CAR in msg.name:
            p = msg.pose[msg.name.index(CAR)].position
            self.latest = {'x': p.x, 'y': p.y}
            self.samples.append((self.sim_t, p.x, p.y))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, vx):
        t = Twist()
        t.linear.x = vx
        self.cmd.publish(t)

    def place(self, x, y):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = CAR
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = Z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)


def dump(series, tag):
    """按仿真时间每 ~0.5 s 打一行，并列出单帧 |dx|>0.3 m 的瞬移。

    只看位移和峰值会骗人：位移里混着摆位瞬移，峰值是逐帧差分的最大值，
    两者口径不同就能同时出现"走了 9 m"和"峰值 0.7 m/s"。
    """
    print('  -- %s 轨迹（sim_t / x / y / 单帧 dt / 单帧 dx）--' % tag)
    last = None
    for i in range(1, len(series)):
        t, x, y = series[i]
        dt = t - series[i - 1][0]
        dx = x - series[i - 1][1]
        if abs(dx) > 0.3:
            print('     瞬移 idx %d t=%.2f dt=%.3f dx=%+.2f' % (i, t, dt, dx))
        if last is None or t - last >= 0.5:
            last = t
            print('     %8.2f  x=%7.2f y=%7.2f  dt=%.3f dx=%+.3f' % (t, x, y, dt, dx))


def run(node, vx, tag):
    node.samples.clear()
    if not node.place(PX, PY):
        print('!! set_entity_state 失败')
        return None
    node.spin_for(3.0)
    x_after_place = node.latest['x']
    print('  %s 摆位目标 x=%.2f，静止 3 s 后 x=%.2f（差 %.2f m）' % (
        tag, PX, x_after_place, abs(x_after_place - PX)))
    mark = len(node.samples)          # 只统计真正踩油门之后的样本
    t0 = time.time()
    while time.time() - t0 < SEC and rclpy.ok():
        node.send(vx)
        node.spin_for(0.05)
    node.send(0.0)
    node.spin_for(1.0)
    s = node.samples[mark:]
    if len(s) < 5:
        print('  !! 采样过少：%d 条' % len(s))
        return None
    dx = s[-1][1] - s[0][1]
    peak = 0.0
    for i in range(1, len(s)):
        dt = s[i][0] - s[i - 1][0]
        if dt > 1e-6:
            peak = max(peak, math.hypot(s[i][1] - s[i - 1][1],
                                        s[i][2] - s[i - 1][2]) / dt)
    print('  %s cmd=%+.2f  位移 dx=%+.2f m  逐帧峰值 %.2f m/s  采样 %d' % (
        tag, vx, dx, peak, len(s)))
    dump(s, tag)
    return dx


def main():
    rclpy.init()
    node = Probe()
    for _ in range(30):
        node.spin_for(1.0)
        if node.latest and node.sim_t > 0.0:
            break
    if not node.latest:
        print('!! 收不到 /model_states')
        return 1
    print('== %s 探针 @ (%.2f, %.2f)  每段 %.0f s ==' % (CAR, PX, PY, SEC))
    fwd = run(node, V, 'forward')
    rev = run(node, -V, 'reverse')
    node.destroy_node()
    rclpy.shutdown()
    if fwd is None or rev is None:
        return 1
    print('== 判读 ==')
    print('  正号往 %+0.2f m，负号往 %+0.2f m' % (fwd, rev))
    if fwd > 1.0 and rev < -1.0:
        print('  → 负速正常：顶推测试里那次"倒车向东"是被物理弹出，不是指令反向')
    elif fwd > 1.0 and rev > 1.0:
        print('  → 负号也被开成向前：该模型/插件不认负速，反向对照判据不成立')
    else:
        print('  → 两种指令都没跑出位移，cmd 链路本身有问题')
    return 0


if __name__ == '__main__':
    sys.exit(main())
