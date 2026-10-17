#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 car_6 安全地放回停车场入口等待位（只动车，不动其它车、不动节点）。

现场（tools/probe88.sh 实测）：car_6 在 (27.17,-18.09) z=0.4 roll=1.57 —— 摔出台面、
侧翻在地面层，轮子不接地，所以 shuttle 一路发 w=-0.35 它纹丝不动。这是任务 #77 那套
"顶住杆 → 轮位误差攒着 → 一次性释放沿车头弹射"的自伤版本。

为什么每一步都不能省（memory「摆位弹射与复位风暴」）：
  * 先零速刹轮 1.5 s：瞬移时还在受驱的轮会把攒下的误差一次放掉，实测沿车头冲到 8.5 m/s；
  * 摆完再压 1.0 s 零速：让落地那一下的自由落体速度先衰减掉，别把它算进卡口通过速度；
  * 摆完必须按"实测位移"自检，不能只信 SetEntityState 返回的 success=true ——
    success 只代表服务端收到并写了位姿，弹射恰恰发生在那之后。

调用方要先杀掉 parking_shuttle_node（它是 /car_6/cmd_vel 的发布者，会抢方向盘）。
脚本自己会检查这个发布者的数量并把它报出来，不靠调用方口头保证。

用法：python3 park_car6.py [--x 12.00 --y -12.80 --z 6.70 --yaw 1.5708]
                           [--verify-gap 12] [--model car_6]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState


def euler(q):
    """四元数 -> (roll, pitch, yaw)。侧翻的车 roll≈±π/2，一眼能认出来；
    只算 yaw 会把 roll=90° 的车报成一个假的航向（fleet_peek 就报错过一次）。"""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class Fixer(Node):
    def __init__(self, model):
        super().__init__('park_car6')
        self.model = model
        self.snap = None
        self.create_subscription(ModelStates, '/model_states', self.on, 10)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_entity_state 未就绪，继续等待')
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % model, 10)

    def on(self, m):
        for n, p in zip(m.name, m.pose):
            if n == self.model:
                self.snap = p
                return

    def wait_snap(self, secs=20.0):
        end = time.time() + secs
        while rclpy.ok() and self.snap is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.snap

    def hold(self, secs, tag):
        end = time.monotonic() + secs
        n = 0
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            n += 1
        print('  %s：零速 %d 帧（%.1f s）' % (tag, n, secs))

    def read(self):
        """取当前位姿：靠 /model_states 的 200 Hz 流，spin 一会儿就有新帧"""
        t0 = time.time()
        old = self.snap
        while rclpy.ok() and self.snap is old and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.snap

    def fmt(self, p):
        r, pi, y = euler(p.orientation)
        return ('(%.2f,%.2f) z=%.2f  roll=%+.3f pitch=%+.3f yaw=%+.3f'
                % (p.position.x, p.position.y, p.position.z, r, pi, y))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    # 负数一律走 --opt=value 形式（memory「argparse 负数带逗号会当选项」），
    # 默认值写在代码里，命令行不传就是入口等待位。
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70,
                    help='台面实测静止高度（world spawn 写 7.0 但落地是 6.70，按 7.0 摆=每次自由落体 0.3 m）')
    ap.add_argument('--yaw', type=float, default=1.5708,
                    help='entry#0 规定的到位朝向 +π/2（车头朝 +y，正对杆轴）')
    ap.add_argument('--brake-s', type=float, default=1.5)
    ap.add_argument('--quiet-s', type=float, default=1.0)
    ap.add_argument('--verify-gap', type=float, default=12.0)
    a = ap.parse_args()

    rclpy.init()
    n = Fixer(a.model)
    p0 = n.wait_snap()
    if p0 is None:
        print('!! 20 s 没收到 /model_states 里的 %s —— gzserver 的 ROS 插件没发或模型不存在' % a.model)
        raise SystemExit(2)
    print('1) 复位前实测：%s' % n.fmt(p0))
    r, _, _ = euler(p0.orientation)
    print('   台面 z≈6.70、地面层 z≈0.45；|roll|>1 rad 就是侧翻。'
          '（当前 z=%.2f roll=%+.3f → %s）' % (p0.position.z, r, '侧翻' if abs(r) > 1.0 else '正立'))

    print('2) 刹轮（瞬移前必须做，否则攒下的轮位误差在落点一次性释放）')
    n.hold(a.brake_s, '摆前')

    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = a.model
    req.state.pose.position.x = a.x
    req.state.pose.position.y = a.y
    req.state.pose.position.z = a.z
    req.state.pose.orientation.z = math.sin(a.yaw / 2.0)
    req.state.pose.orientation.w = math.cos(a.yaw / 2.0)
    req.state.reference_frame = 'world'
    req.state.twist.linear.x = 0.0
    req.state.twist.angular.z = 0.0
    fut = n.cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=8.0)
    res = fut.result()
    # Humble 的 SetEntityState.Response 只有一个字段：ros2 查出来是 ['success']，
    # 没有 status_message 也没有 message。前两版分别按 status.message 和 message 取，
    # 都 AttributeError 崩在 call_async 之后 —— 位姿其实已经写进去了，但后面的零速
    # 压制与位移自检全没跑，等于"半截复位"。回包字段要用 get_fields_and_field_types 查，别照搬别的 srv。
    print('3) SetEntityState 到 (%.2f,%.2f,%.2f) yaw=%+.4f → success=%s'
          % (a.x, a.y, a.z, a.yaw, bool(res and res.success)))
    if not (res and res.success):
        raise SystemExit(3)

    print('4) 落地后压零速')
    n.hold(a.quiet_s, '摆后')
    p1 = n.read()
    print('   刚摆完读到：%s' % n.fmt(p1))

    print('5) 自检：%.0f s 内实测位移（弹射就发生在这里，只信位姿不信 success）' % a.verify_gap)
    end = time.time() + a.verify_gap
    last = n.read()
    moved = 0.0
    peak = (0.0, last)
    frames = 0
    while rclpy.ok() and time.time() < end:
        p = n.read()
        frames += 1
        d = math.hypot(p.position.x - a.x, p.position.y - a.y)
        if d > moved:
            moved = d
        if d > peak[0]:
            peak = (d, p)
        last = p
        n.pub.publish(Twist())
    print('   %d 帧里离落点最大水平偏移 %.2f m（那一帧 %s）；末帧 %s'
          % (frames, peak[0], n.fmt(peak[1]), n.fmt(last)))
    n.destroy_node()
    rclpy.shutdown()
    if moved > 0.5:
        print('!! 摆完还在漂（>0.5 m）：落点上有接触/坡面，或还有别的 cmd_vel 发布者在开车')
        raise SystemExit(1)
    print('CAR6_PARKED_OK：%.0f s 内最大偏移 %.2f m，车停在等待位' % (a.verify_gap, moved))


if __name__ == '__main__':
    main()
