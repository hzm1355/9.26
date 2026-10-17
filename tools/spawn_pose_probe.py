#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""受控复现 #141/#142 的"摆位没生效"：spawn 之后每 2 s 打一次位姿，看时间线。

为什么要这个：respawn 脚本只在 spawn 后 sleep 4 s 读一次，读到的是 (12.00,-12.82)
= world 里的出生位姿，而它打印的 XML 明明写着 6.80 —— 单次采样分不清两种完全不同的病因：
  A) gzserver 压根不认 XML 里的 model 级 <pose>（那要换路子摆位，比如 set_model_position）；
  B) 它先按 6.80 生出来，随后被某个还在跑的节点搬回入口（那要揪出那个节点）。
时间线能一眼分开这两种：A 是全程 6.80 都不出现，B 是先 6.80 再跳走。

不删别的模型；car_6 只挂 diff_drive、没有 camera sensor，所以 /delete_entity 对它安全
（memory「绝不对挂了 camera sensor + SensorPlugin 的模型调 delete_entity」）。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetModelPosition

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from respawn_car6_136 import WORLD, MODEL, model_xml, yaw_q   # noqa: E402

TX, TY = 6.80, -12.80
TZ = 6.70
TYAW = 0.0


class Probe(Node):
    def __init__(self):
        super().__init__('spawn_pose_probe')
        self.last = None
        self.frames = 0
        self.create_subscription(ModelStates, '/model_states', self.cb,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.smp_cli = self.create_client(SetModelPosition, '/set_model_position')
        self.cmd = self.create_publisher(Twist, '/%s/cmd_vel' % MODEL, 10)

    def cb(self, m):
        self.last = m
        self.frames += 1

    def pose(self):
        m = self.last
        if m is None or MODEL not in m.name:
            return None
        p = m.pose[m.name.index(MODEL)]
        o = p.orientation
        return (p.position.x, p.position.y, p.position.z,
                math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)))

    def wait(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, cli, req, secs=25.0):
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > secs:
                raise RuntimeError('服务调用超时')
        return fut.result()


def main():
    rclpy.init()
    n = Probe()
    n.wait(3.0)
    print('起步：/model_states 已收 %d 帧，car_6 %s' % (
        n.frames, '在' if n.pose() else '不在'))
    for cli, nm in ((n.del_cli, 'delete_entity'), (n.spawn_cli, 'spawn_entity'),
                    (n.smp_cli, 'set_model_position')):
        if not cli.wait_for_service(15.0):
            print('!! 服务 %s 不可用' % nm)
            return 1

    xml = model_xml(WORLD, MODEL, '%.2f %.2f %.2f 0 0 %.4f' % (TX, TY, TZ, TYAW))
    print('XML 里的 model 级 pose：%s' % xml.split('<pose>')[1].split('</pose>')[0])

    print('== 1) 先连发 2 s 零速，让轮子别带着旧目标 ==')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        rclpy.spin_once(n, timeout_sec=0.02)
        n.cmd.publish(Twist())
        time.sleep(0.05)

    print('== 2) /delete_entity car_6 ==')
    r = n.call(n.del_cli, DeleteEntity.Request(name=MODEL))
    print('  success=%s %s' % (r.success, r.status_message.strip()))
    t0 = time.monotonic()
    while n.pose() is not None and time.monotonic() - t0 < 12.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('  等 %.1f s 后 %s（期间又收 %d 帧）' % (
        time.monotonic() - t0, '已消失' if n.pose() is None else '还在', n.frames))

    print('== 3) /spawn_entity（XML 已写 6.80）+ 时间线 ==')
    req = SpawnEntity.Request()
    req.name = MODEL
    req.xml = xml
    req.robot_namespace = ''
    req.reference_frame = 'world'
    p = Pose()
    p.position.x, p.position.y, p.position.z = TX, TY, TZ
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = yaw_q(TYAW)
    req.initial_pose = p
    r = n.call(n.spawn_cli, req, secs=40.0)
    print('  success=%s %s' % (r.success, r.status_message.strip()))
    for k in range(12):
        n.wait(2.0)
        q = n.pose()
        print('  t+%2d s  %s' % ((k + 1) * 2, '还没出现' if q is None else
                                 '(%.2f, %.2f, %.2f) yaw=%+.3f' % q))

    print('== 4) 再试另一条摆位路：/set_model_position ==')
    if n.pose():
        rq = SetModelPosition.Request()
        rq.model_name = MODEL
        rq.pose = p
        rq.reference_frame = 'world'
        try:
            rr = n.call(n.smp_cli, rq)
            print('  success=%s %s' % (rr.success, rr.status_message.strip()))
        except Exception as e:
            print('  调用异常：%s' % e)
        for k in range(4):
            n.wait(2.0)
            q = n.pose()
            print('  t+%2d s  %s' % ((k + 1) * 2, '读不到' if q is None else
                                     '(%.2f, %.2f, %.2f) yaw=%+.3f' % q))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
