#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同一时钟刻里连瞬移两次：专门用来验 cmd_pose_trace 按 sim_t 去重会不会吞掉瞬移帧。

/clock 只有 10 Hz，位姿流约 97 Hz，所以老写法 `if t - prev[0] <= 1e-6: continue` 每个
100 ms 窗口只留第一条。两次摆位之间只驻留 40 ms（比一帧位姿流长、比一个时钟格短），
中间那个位姿一定会被发布出来，却一定落在同一个窗口里 —— 于是第二次的位移在读表时凭空
消失，真实事故里"复位瞬移"就发生在这个时刻。实测 0 ms 间隔时物理只保留中间位 1 ms，
三轮一次都没采到，所以必须留这一小段。

每轮都摆退 1.2 m 再摆回原位，净位移为零，不会把车甩离车道，也不会在界外触发复位。

用法（驱动节点在跑没关系）：
  python3 tools/teleport_twice.py --car=car_5 --step=1.2 --rounds=3
"""
import argparse
import math
import time

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class Twice(Node):

    def __init__(self, car):
        super().__init__('teleport_twice')
        self.car = car
        self.t = 0.0
        self.pose = None
        self.frames = 0
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Clock, '/clock', self._clk,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self._st, 10)

    def _clk(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def _st(self, m):
        self.frames += 1
        if self.car not in m.name or self.t <= 0:
            return
        i = m.name.index(self.car)
        p = m.pose[i].position
        q = m.pose[i].orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (self.t, p.x, p.y, p.z, yaw)

    def put(self, x, y, z, yaw):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.car
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_5')
    ap.add_argument('--step', type=float, default=1.2)
    ap.add_argument('--rounds', type=float, default=3.0)
    ap.add_argument('--hold', type=float, default=0.04,
                    help='两次摆位之间让中间位姿存在的墙钟秒数，要大于位姿流一帧(约 10 ms)')
    a = ap.parse_args()

    rclpy.init()
    node = Twice(a.car)
    while not node.cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn('/set_entity_state 未就绪，继续等待')

    # 上一版写成 `end = node.t + 10 if node.t > 0 else 0`：/clock 第一条还没到时
    # end=0，循环条件当场不成立，一次都没 spin 就误报"订阅没通"。DDS 发现本来就要一两秒，
    # 所以这里用墙钟兜底。
    wall = time.monotonic() + 20.0
    while rclpy.ok() and node.pose is None and time.monotonic() < wall:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.pose is None:
        print('!! 20 s 内没拿到 %s 的位姿（订阅没通），本轮作废' % a.car)
        return 1

    print('起始 t=%.2f 位=(%.2f,%.2f) z=%.2f yaw=%+.2f' % node.pose)
    same = 0
    for i in range(int(a.rounds)):
        t0, x0, y0, z0, yaw = node.pose
        dx = -a.step * math.cos(yaw)
        dy = -a.step * math.sin(yaw)
        ok1 = node.put(x0 + dx, y0 + dy, z0, yaw)
        t1 = node.t
        # 摆退之后要让中间那位活过至少一帧位姿流（~10 ms），否则 97 Hz 根本采不到，
        # 0 ms 间隔的实测就是"三次全没采到"。40 ms 足够，又仍在同一个 100 ms 时钟格里。
        wall = time.monotonic() + a.hold
        f1 = node.frames
        while time.monotonic() < wall:
            rclpy.spin_once(node, timeout_sec=0.005)
        ok2 = node.put(x0, y0, z0, yaw)
        t2 = node.t
        grid = round(t1 * 10) == round(t2 * 10)
        same += 1 if grid else 0
        print('轮%d 摆退 %s(t=%.3f) 中间收到 %d 帧位姿 -> 摆回 %s(t=%.3f) %s（两次相隔 %.0f ms）'
              % (i + 1, 'OK' if ok1 else '失败', t1, node.frames - f1,
                 'OK' if ok2 else '失败', t2,
                 '同一时钟格' if grid else '跨格', (t2 - t1) * 1000.0))
        if not (ok1 and ok2):
            print('!! 摆位服务失败，本轮作废')
            return 1
        end = node.t + 1.5
        while rclpy.ok() and node.t < end:
            rclpy.spin_once(node, timeout_sec=0.02)
    print('完成 %d 轮，其中 %d 轮两次摆位落在同一个 100 ms 时钟格里' % (int(a.rounds), same))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
