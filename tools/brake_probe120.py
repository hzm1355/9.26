#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#120 —— car_6 幽灵自转的能量源归因（纯指令侧，不摆位、不重启、不杀进程）。

已知现场（_gh118 / _wt115）：/car_6/cmd_vel 上 Publisher count=0、70 s 内到达 0 条，
但 car_6 以 ~2.7 rad/s 连续自转、累计路径 95 m；轮角速率左右大小相等符号相反，
与"纯原地转 ω=−2.7"的预测 (+9.45/−9.45) 实测 (+10.6/−8.4) 吻合。
所以要么 (a) 插件内部目标轮速被永久卡住（cmd_vel_timeout 不触发），
要么 (b) 插件根本没在收消息，车是被动被物理推着转。

五段判别，每段前后各读一次 gz 轮角，段内逐帧展开累加车体 yaw/路径：
  A 静默   —— 一条 cmd_vel 都不发（本段若有消息到达 = 外来者）
  B 零速   —— 20 Hz 连发 (0,0)。轮速归零 ⇒ 插件在收，卡的是 timeout 不清目标
  C 静默   —— 再停发。B 归零后 C 又自己转 ⇒ 源不在插件目标，在物理侧
  D 新指令 —— 发 (0,+1.0)。轮速跟着翻符号 ⇒ 插件响应新鲜指令
  E 静默   —— 收尾状态

时长写死在 SEGS 里，不走命令行（argparse 传负数带逗号会被当选项）。
"""
import math
import re
import subprocess
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import BEST, SKIP, wrap, yaw_of   # noqa: E402
import rclpy                                        # noqa: E402
from rclpy.node import Node                          # noqa: E402
from geometry_msgs.msg import Twist                  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock                  # noqa: E402
from gazebo_msgs.msg import ModelStates              # noqa: E402

CAR = 'car_6'
SEP, RAD = 0.70, 0.10
NUM = r'[-+0-9.eE]+'

# (tag, 发不发, v, w, 目标秒数)
SEGS = [('A 静默', False, 0.0, 0.0, 8.0),
        ('B 零速', True, 0.0, 0.0, 10.0),
        ('C 静默', False, 0.0, 0.0, 8.0),
        ('D 新指令', True, 0.0, 1.0, 8.0),
        ('E 静默', False, 0.0, 0.0, 8.0)]


class Watch(Node):
    def __init__(self, car):
        super().__init__('brake_probe120')
        self.car = car
        self.clock = 0.0
        self.pose = None
        self.prev = None
        self.yaw_acc = 0.0
        self.path_acc = 0.0
        self.near = ('-', 1e9)
        self.n_ms = 0
        self.n_cmd = 0
        self.z_max = 0.0
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        # BEST_EFFORT 订阅可同时收 RELIABLE / BEST_EFFORT 发布者，用它当探针
        self.create_subscription(Twist, '/%s/cmd_vel' % car, self.on_cmd, BEST)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % car, rel)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.n_cmd += 1

    def on_s(self, m):
        if self.car not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(self.car)
        p = m.pose[i].position
        self.z_max = max(self.z_max, p.z)
        cur = (self.clock, p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
        best = ('-', 1e9)
        for j, nm in enumerate(m.name):
            if nm == self.car or nm in SKIP:
                continue
            q = m.pose[j].position
            d = math.hypot(q.x - cur[1], q.y - cur[2])
            if d < best[1]:
                best = (nm, d)
        self.near = best
        if self.prev is None:
            self.prev = cur
        elif cur[0] - self.prev[0] > 1e-9:
            self.yaw_acc += wrap(cur[4] - self.prev[4])
            self.path_acc += math.hypot(cur[1] - self.prev[1], cur[2] - cur[2])
            self.prev = cur
        self.pose = cur

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def send(self, v, w):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)


def gz_read(car):
    p = subprocess.run(['gz', 'model', '-m', car, '-i'],
                       capture_output=True, text=True, timeout=25)
    txt = p.stdout
    if 'left_wheel_joint' not in txt:
        raise RuntimeError('gz 输出里没有轮关节：\n%s' % txt[:400])

    def ang(j):
        m = re.search(r'name: "%s::%s".{0,400}?angle: (%s)' % (car, j, NUM), txt, re.S)
        return float(m.group(1)) if m else float('nan')

    m = re.search(r'^pose \{\s*position \{\s*x: (%s)\s+y: (%s)\s+z: (%s)\s*\}\s*'
                  r'orientation \{\s*x: %s\s+y: %s\s+z: (%s)\s+w: (%s)'
                  % (NUM, NUM, NUM, NUM, NUM, NUM, NUM), txt, re.M)
    if not m:
        raise RuntimeError('gz 输出里没有 model pose：\n%s' % txt[:400])
    qz, qw = float(m.group(4)), float(m.group(5))
    return (ang('left_wheel_joint'), ang('right_wheel_joint'),
            float(m.group(1)), float(m.group(2)), float(m.group(3)),
            math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz)))


def snap(n):
    """gz 轮角 + 紧贴其后的展开累积量，返回 (8+3) 元组。"""
    r = gz_read(n.car)
    c = n.clock
    t0 = time.time()
    while n.clock == c and time.time() - t0 < 3.0:
        n.tick()
        time.sleep(0.01)
    return r + (c, n.yaw_acc, n.path_acc, time.time())


def fmt(r, head):
    return ('  %-8s 轮角 左%9.3f 右%9.3f 位(%7.3f,%7.3f) z=%6.4f yaw=%+.4f '
            '钟%9.2f 累计yaw%+9.3f 累计路%7.3f') % ((head,) + tuple(r[:9]))


def run_seg(n, tag, publish, v, w, secs):
    pre = snap(n)
    nc0 = n.n_cmd
    ya, pa = n.yaw_acc, n.path_acc
    last = 0.0
    sent_cnt = 0
    end = time.time() + secs
    while time.time() < end:
        if publish:
            n.send(v, w)
            sent_cnt += 1
        n.tick()
        time.sleep(0.05)
        n.tick()
        if time.time() - last >= 1.0 and n.pose:
            last = time.time()
            dy, dp = n.yaw_acc - ya, n.path_acc - pa
            print('    %s t=%9.2f (%7.3f,%7.3f) 本秒 ω=%+.3f |v|=%+.3f 最近 %s@%4.2f m  到达=%d 发%d' % (
                tag, n.pose[0], n.pose[1], n.pose[2], dy, dp,
                n.near[0], n.near[1], n.n_cmd - nc0, sent_cnt), flush=True)
            ya, pa = n.yaw_acc, n.path_acc
    post = snap(n)
    dur = post[6] - pre[6]
    dl, dr = post[0] - pre[0], post[1] - pre[1]
    dyaw, dpath = post[7] - pre[7], post[8] - pre[8]
    print('\n--- %s  publish=%s v=%+.2f w=%+.2f  窗口 %.2f s（发出 %d 条，收到 %d 条）---' % (
        tag, 'ON ' if publish else 'off', v, w, dur, sent_cnt, n.n_cmd - nc0), flush=True)
    print(fmt(pre, '段前'))
    print(fmt(post, '段后'))
    if dur > 0:
        print('  轮速 左 %+7.3f  右 %+7.3f rad/s   两轮差反推 ω=%+.3f rad/s' % (
            dl / dur, dr / dur, ((dr - dl) * RAD) / SEP / dur))
        print('  车体真值 ω=%+.3f rad/s（与轮差差 %+.3f）  |v|=%+.3f m/s  路径 %.3f m  位移 %.3f m' % (
            dyaw / dur,
            dyaw / dur - ((dr - dl) * RAD) / SEP / dur,
            dpath / dur, dpath,
            math.hypot(post[2] - pre[2], post[3] - pre[3])))
    return post


def main():
    rclpy.init()
    n = Watch(CAR)
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 20:
        n.tick()
        time.sleep(0.02)
    if n.pose is None:
        print('!! 20 s 内没拿到 /model_states，gzserver 可能已经不在')
        return 1
    print('=== 起点 ===')
    print(fmt(snap(n), '起点'), flush=True)
    for spec in SEGS:
        run_seg(n, *spec)
    print('\n=== 终点 ===')
    print(fmt(snap(n), '终点'))
    print('全程 model_states %d 帧，cmd_vel 到达 %d 条，z 最高 %.4f' % (
        n.n_ms, n.n_cmd, n.z_max))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
