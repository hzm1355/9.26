#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 "摆不住"的判别探针：三段各 8 s —— 不发指令 / 连发零速 / 再不发指令。

要劈开的两种解释：
  A) 插件里留着最后一次角速度指令（杀 shuttle 前那条 `ros2 topic pub --once` 零速
     可能因为 DDS 发现竞态根本没送达），车其实一直在按 w=0.35 自己转 ——
     那表现就是"摆位后 yaw 继续单调走、车心被甩弧"，与实测的 0.28→1.05→3.38 m 残差同形；
  B) 外部在推（别的车撞、穿透求解）—— 那连发零速那一段不会有任何区别。
判据：yaw 在"连发零速"段停住、在两段"不发"里继续走 ⇒ A；三段一样 ⇒ B。
顺带把其它五辆车的位置一起打出来，B 里若是被撞，能看见谁在旁边。
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('spin_probe')
        self.snap = {}
        self.create_subscription(ModelStates, '/model_states', self.on, 10)
        self.pub = self.create_publisher(Twist, '/car_6/cmd_vel', 10)

    def on(self, m):
        for i, n in enumerate(m.name):
            if n in CARS:
                p = m.pose[i]
                self.snap[n] = (p.position.x, p.position.y, yaw_of(p.orientation))

    def phase(self, secs, w):
        print('--- 段：w=%s，%.0f s ---' % (w, secs))
        end = time.time() + secs
        last = None
        while time.time() < end:
            if w is not None:
                t = Twist()
                t.angular.z = w
                self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
            if 'car_6' not in self.snap:
                continue
            x, y, h = self.snap['car_6']
            line = '  %6.1f s  car_6=(%7.2f,%7.2f) yaw=%+.3f' % (time.time() % 1000, x, y, h)
            if last:
                # last 存的是上一拍的 (x,y,yaw)，这里两个差都要算：只看位移会把
                # "原地被拧着转"看成"没动"，而 yaw 自己会跨 ±π 跳变，必须 wrap。
                dh = h - last[2]
                while dh > math.pi:
                    dh -= 2 * math.pi
                while dh < -math.pi:
                    dh += 2 * math.pi
                line += '  Δ位=%.2f Δyaw=%+.3f' % (math.hypot(x - last[0], y - last[1]), dh)
            near = min(((math.hypot(px - x, py - y), n) for n, (px, py, _) in
                        self.snap.items() if n != 'car_6'), default=(99, '-'))
            line += '   最近他车 %s@%.2f m' % (near[1], near[0])
            print(line)
            last = (x, y, h)
            time.sleep(0.9)


def main():
    rclpy.init()
    n = Probe()
    end = time.time() + 12
    while time.time() < end and 'car_6' not in n.snap:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'car_6' not in n.snap:
        print('读不到 car_6 位姿')
        return 2
    n.phase(8, None)
    n.phase(8, 0.0)
    n.phase(8, None)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
