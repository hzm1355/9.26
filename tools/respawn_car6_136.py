#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复位 car_6 的驱动通道：删掉运行期实例再从 world 里的原始 XML 重新 spawn 一份。

为什么要这么干（#135/#136 的实测结论）：
  * car_6 现在以 ω≈-2.30 rad/s 稳态自转，420 s / 14 个窗口无衰减；
  * /car_6/cmd_vel 上真有 rclpy 20 Hz 发布者、QoS 配对成功、换 UDPv4-only 也一样，ω 不变；
  * 同一个 gzserver 里 barrier 插件的 ROS 输入路实测是通的（手动抬落杆 angle 跟着走）；
  ⇒ 死的是 car_6 那一个 diff_drive 插件实例的回调，而它最后一次 SetVelocity 是持久的
    ODE 铰参数，所以摆位/零速都钉不住。**只有新建插件实例才会把缓存的轮速目标带回 0**，
    reset_world 只是重放物理状态，插件每帧还会把缓存目标重新 ApplyCommand 上去。

代价与补偿：重新 spawn 会重建整棵 visual 树，运行时 SetMaterial 贴上去的车牌材质随之丢失
（memory「运行时换车牌纹理机制」）。所以脚本最后必须重跑一次 /plate/set_plate，把 car_6 的
front/rear 换回 沪A12345 —— 材质载体 plate_carrier_* 是独立模型，没被删，重贴是幂等的。

不动别的车、不动 world 文件、不发行驶指令之外的任何东西。
"""
import math
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from v2x_msgs.srv import SetPlate

WORLD = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world'
MODEL = 'car_6'
# world 里 car_6 的出生位姿：12.00 -12.80 7.00 0 0 1.5708（等待位，车头朝北）
SPAWN_X, SPAWN_Y, SPAWN_YAW = 12.00, -12.80, 1.5708
# 摆位高度一律 6.70（= 静止路面），不用 world 里的 7.00：那 0.3 m 自由落体带 2.4 m/s
# 垂直分量，WorldLinearVel 会把它算进"实测速度"，拍出来就是一张 1.6 m/s 的运动模糊帧。
SPAWN_Z = 6.70
PLATE = '沪A12345'


def opt(name, default):
    """--at=x,y,yaw 这类带值的选项。负数必须走 --opt=value 形式（argparse 那套
    "--at -5,-14" 会被当成缺参数，这里同理）。"""
    for a in sys.argv[1:]:
        if a.startswith('--' + name + '='):
            return a.split('=', 1)[1]
    return default


_at = opt('at', '%.2f,%.2f,%.4f' % (SPAWN_X, SPAWN_Y, SPAWN_YAW))
SPAWN_X, SPAWN_Y, SPAWN_YAW = [float(v) for v in _at.split(',')]


def model_xml(path, name, pose=None):
    txt = open(path, encoding='utf-8', errors='replace').read()
    a = txt.index('<model name="%s">' % name)
    b = txt.index('\n    </model>', a) + len('\n    </model>')
    block = txt[a:b]
    if block.count('<model') != 1:
        raise RuntimeError('car_6 块里嵌了子 model，截取边界要重写')
    if pose is not None:
        # 为什么要改 XML 而不是只传 initial_pose（#141 实测）：块里第一个 <pose> 是
        # world 给 car_6 写的出生位姿 12.00 -12.80 7.00 0 0 1.5708，/spawn_entity 的
        # initial_pose 盖不住它 —— 传 --at=6.80,-12.80,0.0 却仍然生回 (12.00,-12.82)
        # yaw=+1.57，车因此没走复位带、gate_in 越不过 far=2.5、整轮零抓拍。
        # 块首就是 model 级 pose（link/inertial 的 <pose> 都在它后面），所以 count=1
        # 只会命中要改的那一行；命中数不为 1 说明 world 结构变了，宁可直接炸。
        block, n = re.subn(r'<pose[^>]*>[^<]*</pose>',
                           '<pose>%s</pose>' % pose, block, count=1)
        if n != 1:
            raise RuntimeError('car_6 块开头没找到 model 级 <pose>，摆位改不了')
    return '<sdf version="1.6">%s</sdf>' % block


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Respawn(Node):
    def __init__(self):
        super().__init__('respawn_car6_136')
        self.beam = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.last = None
        self.create_subscription(ModelStates, '/model_states',
                                 lambda m: setattr(self, 'last', m), self.beam)
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.plate_cli = self.create_client(SetPlate, '/plate/set_plate')
        self.cmd_pub = self.create_publisher(Twist, '/%s/cmd_vel' % MODEL, 10)

    def wait_srv(self):
        for cli, n in ((self.del_cli, 'delete_entity'), (self.spawn_cli, 'spawn_entity'),
                       (self.plate_cli, 'set_plate')):
            if not cli.wait_for_service(15.0):
                raise RuntimeError('服务 /%s 不可用' % n)

    def pose(self):
        m = self.last
        if m is None or MODEL not in m.name:
            return None
        p = m.pose[m.name.index(MODEL)]
        o = p.orientation
        return (p.position.x, p.position.y, p.position.z,
                math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                           1.0 - 2.0 * (o.y * o.y + o.z * o.z)))

    def spin_for(self, secs):
        """返回 (帧数, 平均 ω, 车心净位移)：ω 是判"自转有没有被复位掉"的主证据。"""
        t0 = time.monotonic()
        tot, prev, first, last, n = 0.0, None, None, None, 0
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue
            n += 1
            if prev is not None:
                d = p[3] - prev
                while d > math.pi:
                    d -= 2.0 * math.pi
                while d < -math.pi:
                    d += 2.0 * math.pi
                tot += d
            if first is None:
                first = p
            prev, last = p[3], p
        span = time.monotonic() - t0
        moved = math.hypot(last[0] - first[0], last[1] - first[1]) if first else 0.0
        return n, (tot / span if span else 0.0), moved

    def call(self, cli, req, secs=20.0):
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > secs:
                raise RuntimeError('服务调用超时')
        return fut.result()

    def drive(self, vx, secs):
        """按 20 Hz 发一段 vx：这是复位后唯一认的驱动通道（#135 已证摆位钉不住）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)
            m = Twist()
            m.linear.x = vx
            self.cmd_pub.publish(m)
            time.sleep(0.05)


def main():
    dry = '--dry-run' in sys.argv
    want = '%.2f %.2f %.2f 0 0 %.4f' % (SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_YAW)
    xml = model_xml(WORLD, MODEL, want)
    got = re.search(r'<pose>([^<]*)</pose>', xml).group(1)
    print('== 0) 目标摆位 %s｜XML 里写进去的 %s｜块 %d 字节，plugin=%s ==' % (
        want, got, len(xml), 'diff_drive' if 'diff_drive' in xml else '缺失!!'))
    if got != want:
        print('  !! pose 没替换成功，别往下跑（会生回 world 的出生位姿）')
        return 2
    if dry:
        print('  （--dry-run：只截取不执行）')
        return 0

    rclpy.init()
    node = Respawn()
    node.wait_srv()

    n0, w0, mv0 = node.spin_for(8.0)
    p0 = node.pose()
    print('== 1) 复位前：位姿 (%.2f,%.2f,%.2f) yaw=%+.2f｜%d 帧 ω=%+.3f rad/s ==' % (
        p0[0], p0[1], p0[2], p0[3], n0, w0))

    print('== 2) /delete_entity car_6 ==')
    r = node.call(node.del_cli, DeleteEntity.Request(name=MODEL))
    print('  success=%s message=%s' % (r.success, r.status_message.strip()))
    if not r.success:
        return 1
    t0 = time.monotonic()
    while node.pose() is not None and time.monotonic() - t0 < 12.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('  等 %5.1f s 后 model_states 里 car_6 %s' % (
        time.monotonic() - t0, '已消失' if node.pose() is None else '还在'))

    print('== 3) /spawn_entity car_6（回到 world 里的出生位姿）==')
    req = SpawnEntity.Request()
    req.name = MODEL
    req.xml = xml
    req.robot_namespace = ''
    req.reference_frame = 'world'
    req.initial_pose = Pose()
    req.initial_pose.position.x = SPAWN_X
    req.initial_pose.position.y = SPAWN_Y
    req.initial_pose.position.z = SPAWN_Z
    req.initial_pose.orientation.x, req.initial_pose.orientation.y, \
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = yaw_q(SPAWN_YAW)
    r = node.call(node.spawn_cli, req, secs=40.0)
    print('  success=%s message=%s' % (r.success, r.status_message.strip()))
    if not r.success:
        print('  !! 删除成功但没生回来 —— 立刻整栈重启恢复 car_6，别继续往下跑')
        return 1
    t0 = time.monotonic()
    while node.pose() is None and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(4.0)                     # 让它落地稳定再测
    rclpy.spin_once(node, timeout_sec=0.1)
    p1 = node.pose()
    print('  现位姿 (%.2f,%.2f,%.2f) yaw=%+.2f' % (p1[0], p1[1], p1[2], p1[3]))

    print('== 4) 静默 8 s：ω 应归零（这是复位成功的判据）==')
    n1, w1, mv1 = node.spin_for(8.0)
    print('  %d 帧 ω=%+.4f rad/s｜净位移 %.2f m' % (n1, w1, mv1))

    print('== 5) 前向 0.25 m/s 跑 4 s，再零速 3 s：cmd_vel 通道是否复活 ==')
    node.drive(0.25, 4.0)
    node.drive(0.0, 3.0)
    n2, w2, mv2 = node.spin_for(4.0)
    p2 = node.pose()
    print('  行驶段后位姿 (%.2f,%.2f)｜静默段 ω=%+.4f、净位移 %.2f m' % (
        p2[0], p2[1], w2, mv2))
    d_run = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    print('  4 s 油门应推动 ≈0.9 m，实走 %.2f m' % d_run)

    print('== 6) 重贴车牌（新 visual 树没有运行时材质）==')
    for slot in ('front', 'rear'):
        req = SetPlate.Request()
        req.vehicle_id = MODEL
        req.slot = slot
        req.number = PLATE
        try:
            rr = node.call(node.plate_cli, req, secs=30.0)
            print('  %-5s success=%s seq=%s msg=%s' % (
                slot, rr.success, rr.seq, rr.message.strip()))
        except Exception as e:
            print('  %-5s 调用异常：%s（材质没贴 => manager 认不出白名单，必须补）' % (slot, e))

    ok = abs(w1) < 0.10 and d_run > 0.30
    print('== 判读 ==')
    print('  复位成功判据：静默 ω 归零(%s) 且油门能推动(%s) → %s' % (
        abs(w1) < 0.10, d_run > 0.30, '通过' if ok else '不通过'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
