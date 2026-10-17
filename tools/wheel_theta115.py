#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#85 收官：给 car_6 发一段指令，用**前后两次 `gz model -i` 的轮关节累计角 Δθ** 判插件到底算了什么。

前面几轮把 v 通道、地面摩擦、承载高度、运行态副本、文件差异全排除了，剩下的唯一分歧是
"指令进去以后两轮各自转了多少"。这个话题侧读不到（/joint_states 在本 world 不存在），
但 gzserver 进程内的 `gz model -m car_6 -i` 会吐 joint{ name angle }，angle 是**物理关节
自己的累计角**，不是插件上报的 odom —— 所以它是一次直读轮编码器，能一锤定音：

  A 两轮同号且等量     -> 插件把 ω 算成了共模（差速解算 bug），车"前冲不转"由此来
  B 两轮都几乎不动     -> 力矩根本没下发（话题没进插件 / 插件没接到）
  C 两轮 ±对称大增而车不转 -> 插件算对了，是物理侧被顶住/打滑

预测用与 cmd_pivot102 同一套公式（左 = (v − ω·0.35)/0.10，右 = (v + ω·0.35)/0.10）。
时长用 /clock 首末差（8 Hz、步进 0.1000 s），位姿用 gz 自带的 model pose，
避免 /model_states 80 Hz ÷ /clock 10 Hz 那种逐拍差分把速度缩 8~10 倍的坑。

只发 cmd_vel，不摆位、不改 world、不重启。
"""
import argparse
import math
import re
import subprocess
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import BEST, wrap, yaw_of      # noqa: E402
import rclpy                                      # noqa: E402
from rclpy.node import Node                         # noqa: E402
from geometry_msgs.msg import Twist                 # noqa: E402
from rosgraph_msgs.msg import Clock                 # noqa: E402

SEP, RAD = 0.70, 0.10
NUM = r'[-+0-9.eE]+'


class Read(Node):
    def __init__(self, car):
        super().__init__('wheel_theta115')
        self.car = car
        self.clock = 0.0
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % car, 10)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def send(self, v, w):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)


def gz_read(car):
    """返回 (左轮角, 右轮角, x, y, z, yaw, 钟)；gz 侧读取与 /clock 读取贴在一起。"""
    p = subprocess.run(['gz', 'model', '-m', car, '-i'],
                       capture_output=True, text=True, timeout=25)
    txt = p.stdout
    if 'left_wheel_joint' not in txt:
        raise RuntimeError('gz 输出里没有轮关节，gzserver 可能不应答：\n%s' % txt[:400])

    def ang(j):
        m = re.search(r'name: "%s::%s".{0,400}?angle: (%s)' % (car, j, NUM), txt, re.S)
        return float(m.group(1)) if m else float('nan')

    m = re.search(r'^pose \{\s*position \{\s*x: (%s)\s+y: (%s)\s+z: (%s)\s*\}\s*'
                  r'orientation \{\s*x: %s\s+y: %s\s+z: (%s)\s+w: (%s)' % (NUM, NUM, NUM, NUM, NUM, NUM, NUM),
                  txt, re.M)
    if not m:
        raise RuntimeError('gz 输出里没有 model pose：\n%s' % txt[:400])
    qz, qw = float(m.group(4)), float(m.group(5))
    yaw = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    return (ang('left_wheel_joint'), ang('right_wheel_joint'),
            float(m.group(1)), float(m.group(2)), float(m.group(3)), yaw)


def snap(n):
    """贴钟读取：先收到一个新鲜 /clock，再读 gz，读完立刻再取钟。"""
    t0 = time.time()
    while n.clock <= 0 and time.time() - t0 < 10:
        n.tick()
        time.sleep(0.02)
    c_a = n.clock
    r = gz_read(n.car)
    t1 = time.time()
    while n.clock == c_a and time.time() - t1 < 3.0:
        n.tick()
        time.sleep(0.01)
    return r + (c_a, n.clock)


def fmt(r, head):
    """(左轮角 右轮角 x y z yaw 起钟 止钟) -> 一行。"""
    return ('  %-4s 轮角 左%9.3f 右%9.3f  位(%7.3f,%7.3f) z=%6.4f yaw=%+.4f  钟%.2f→%.2f'
            % ((head,) + tuple(r)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--t', type=float, default=3.0, help='每段仿真时长目标（秒）')
    ap.add_argument('--gap', type=float, default=2.5, help='段间零速间隔（秒）')
    a = ap.parse_args()

    rclpy.init()
    n = Read(a.model)

    def tl_r(v, w):
        return (v - w * SEP / 2.0) / RAD

    def tr_r(v, w):
        return (v + w * SEP / 2.0) / RAD

    segs = [(0.00, 0.00, '负对照：不发任何轮速'),
            (0.00, -0.35, '纯 ω 右'),
            (0.00, +0.35, '纯 ω 左'),
            (0.31, -0.45, '车队同款弧线'),
            (0.31, 0.00, '纯 v 直行')]

    print('=== 起点 ===')
    print(fmt(snap(n), '起点'))

    for (v, w, tag) in segs:
        pre = snap(n)
        end = time.time() + a.t
        while time.time() < end:
            n.send(v, w)
            n.tick()
            time.sleep(0.05)
            n.tick()
        n.send(0.0, 0.0)
        post = snap(n)
        dur = 0.5 * (post[6] + post[7]) - 0.5 * (pre[6] + pre[7])
        dl, dr = post[0] - pre[0], post[1] - pre[1]
        pl, pr = tl_r(v, w) * dur, tr_r(v, w) * dur
        dpos = math.hypot(post[3] - pre[3], post[4] - pre[4])
        dyaw = wrap(post[5] - pre[5])
        print('\n--- %s  指令 v=%+.2f w=%+.2f（目标 %d s，实测 %.2f s）---' % (tag, v, w, int(a.t), dur))
        print(fmt(pre, '段前'))
        print(fmt(post, '段后'))
        if dur > 0:
            print('  轮角增量   左 %+9.3f（预测 %+9.3f）  右 %+9.3f（预测 %+9.3f）' % (dl, pl, dr, pr))
            print('  兑现轮速   左 %+7.3f  右 %+7.3f rad/s   （指令 %+.3f / %+.3f）' % (
                dl / dur, dr / dur, tl_r(v, w), tr_r(v, w)))
            print('  车体真值   位移 %6.3f m（|v| %+.3f）  转角 %+7.4f rad（ω %+.3f）' % (
                dpos, dpos / dur, dyaw, dyaw / dur))
        if v == 0.0 and w == 0.0:
            if abs(dl) > 0.5 or abs(dr) > 0.5:
                print('  >> 负对照失守：零指令下轮角仍在累计（左 %+.2f 右 %+.2f）—— 有别的发布者或物理在转轮' % (dl, dr))
            else:
                print('  >> 负对照通过：零指令两轮不动，读数可信')
            continue
        if dur <= 0 or abs(pl) + abs(pr) < 0.2:
            continue
        opp = pl * pr < 0            # 这一发指令本来要求两轮反号
        if abs(dl) < 0.25 and abs(dr) < 0.25:
            print('  >> 判读 B：两轮都没被驱动，力矩没下发（话题没进插件 / 插件没生效）')
        elif opp and dl * dr > 0:
            print('  >> 判读 A：要求反号却出了同号 —— 插件把 ω 算成了共模')
        elif opp and abs(dl) > 0.5 * abs(pl) and abs(dr) > 0.5 * abs(pr) \
                and abs(dyaw) < 0.3 * abs(w) * dur:
            print('  >> 判读 C：两轮确实按差速转了，车体却没转起来 —— 物理侧被顶住/打滑')
        else:
            print('  >> 判读 D：与 A/B/C 都不完全吻合，按上面原始数读')

        end_gap = time.time() + a.gap
        while time.time() < end_gap:
            n.send(0.0, 0.0)
            n.tick()
            time.sleep(0.05)

    print('\n=== 终点 ===')
    print(fmt(snap(n), '终点'))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
