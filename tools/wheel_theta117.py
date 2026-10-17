#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#85 收官 v2：轮关节 Δθ + **逐帧展开 yaw 累积** + 每秒最近他车（指认顶推者）。

v1（wheel_theta115.py）里我犯了两个错，这一版都修：
  1) 绕卷混淆：段前后各只读一次 gz，3 s 的窗口里若车以 ~2.7 rad/s 连续转，
     atan2 出来的 yaw 差会被 wrap 成 −1.6 rad，看着像"没转够"。
     实测 −1.597/−1.378/−1.736 各减 2π 得 −7.88/−7.66/−8.02 rad，
     除以 2.9/2.9/3.05 s 正好 −2.7 rad/s，与轮速反推的 (−0.84−1.08)/0.70=−2.74 吻合
     —— 所以第 3/4/5 段车在被**外力连续拧转**，不是我的指令生效。
  2) 判读 A/B/C 的前提是"这段没人碰车"。v1 没有旁证，所以三段全判成 D。
本版按 /model_states 80 Hz 逐帧 wrap 累加 yaw 与路径（锚在钟跳变那帧，避开
逐拍差分把速度缩 8~10 倍的坑），并每秒打"最近他车中心距"，谁顶的一眼看得见。

段序与 v1 一致，便于直接对照 _wt115.txt。只发 cmd_vel，不摆位、不重启。
"""
import argparse
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
from rosgraph_msgs.msg import Clock                  # noqa: E402
from gazebo_msgs.msg import ModelStates              # noqa: E402

SEP, RAD = 0.70, 0.10
NUM = r'[-+0-9.eE]+'


class Watch(Node):
    def __init__(self, car):
        super().__init__('wheel_theta117')
        self.car = car
        self.clock = 0.0
        self.pose = None        # (clock, x, y, z, yaw)
        self.prev = None
        self.yaw_acc = 0.0      # 逐帧 wrap 累加，不绕卷
        self.path_acc = 0.0
        self.near = ('-', 1e9)
        self.n_ms = 0
        self.n_cmd = 0
        self.n_stray = 0
        self.stray_last = None
        self.sent = (0.0, 0.0)
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        # BEST_EFFORT 订阅能同时收到 RELIABLE / BEST_EFFORT 发布者，用它当探针
        self.create_subscription(Twist, '/%s/cmd_vel' % car, self.on_cmd, BEST)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % car, 10)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.n_cmd += 1
        if abs(m.linear.x - self.sent[0]) > 0.01 or abs(m.angular.z - self.sent[1]) > 0.01:
            self.n_stray += 1
            self.stray_last = (round(self.clock, 2), round(m.linear.x, 2), round(m.angular.z, 2))

    def on_s(self, m):
        if self.car not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(self.car)
        p = m.pose[i].position
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
        elif cur[0] - self.prev[0] > 1e-9:      # 只在钟跳变那帧差分，且跳变才更新 prev
            self.yaw_acc += wrap(cur[4] - self.prev[4])
            self.path_acc += math.hypot(cur[1] - self.prev[1], cur[2] - self.prev[2])
            self.prev = cur
        self.pose = cur

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def send(self, v, w):
        self.sent = (v, w)
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
    """gz 读取 + 紧贴其后的展开累积量。返回 9 元组。"""
    r = gz_read(n.car)
    c = n.clock
    t0 = time.time()
    while n.clock == c and time.time() - t0 < 3.0:
        n.tick()
        time.sleep(0.01)
    return r + (c, n.yaw_acc, n.path_acc)


def fmt(r, head):
    return ('  %-4s 轮角 左%9.3f 右%9.3f 位(%7.3f,%7.3f) z=%6.4f yaw=%+.4f '
            '钟%9.2f 累计yaw%+9.3f 累计路%7.3f'
            % ((head,) + tuple(r)))


def hold(n, secs, v, w, tag):
    """发指令 secs 秒，每秒打一行旁证。"""
    end = time.time() + secs
    last = 0.0
    ya, pa = n.yaw_acc, n.path_acc
    while time.time() < end:
        n.send(v, w)
        n.tick()
        time.sleep(0.05)
        n.tick()
        if time.time() - last >= 1.0 and n.pose:
            last = time.time()
            dy, dp = n.yaw_acc - ya, n.path_acc - pa
            print('    %s t=%9.2f (%6.2f,%6.2f) 本秒 ω=%+.3f |v|=%+.3f 最近=%s@%4.2f m  非己指令=%d' % (
                tag, n.pose[0], n.pose[1], n.pose[2], dy, dp / 1.0,
                n.near[0], n.near[1], n.n_stray))
            ya, pa = n.yaw_acc, n.path_acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--t', type=float, default=3.0)
    ap.add_argument('--gap', type=float, default=2.5)
    a = ap.parse_args()

    rclpy.init()
    n = Watch(a.model)
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 15:
        n.tick()
        time.sleep(0.02)

    def pl_p(v, w):
        return (v - w * SEP / 2.0) / RAD

    def pr_p(v, w):
        return (v + w * SEP / 2.0) / RAD

    segs = [(0.00, 0.00, '负对照'),
            (0.00, -0.35, '纯ω右'),
            (0.00, +0.35, '纯ω左'),
            (0.31, -0.45, '弧线'),
            (0.31, 0.00, '纯v')]

    print('=== 起点 ===')
    print(fmt(snap(n), '起点'))
    for (v, w, tag) in segs:
        pre = snap(n)
        hold(n, a.t, v, w, tag)
        n.send(0.0, 0.0)
        post = snap(n)
        dur = post[6] - pre[6]
        dl, dr = post[0] - pre[0], post[1] - pre[1]
        dyaw, dpath = post[7] - pre[7], post[8] - pre[8]
        pl, pr = pl_p(v, w) * dur, pr_p(v, w) * dur
        print('\n--- %s  v=%+.2f w=%+.2f（%.2f s）---' % (tag, v, w, dur))
        print(fmt(pre, '段前'))
        print(fmt(post, '段后'))
        if dur > 0:
            print('  轮角增量 左 %+9.3f（预测 %+7.3f，兑现 %.2f）  右 %+9.3f（预测 %+7.3f，兑现 %.2f）' % (
                dl, pl, dl / pl if abs(pl) > 0.2 else float('nan'),
                dr, pr, dr / pr if abs(pr) > 0.2 else float('nan')))
            print('  展开真值 ω=%+.3f rad/s（指令 %+.3f，兑现 %s）  |v|=%+.3f m/s（指令 %+.3f）  路径 %.3f m' % (
                dyaw / dur, w,
                '%.2f' % (dyaw / dur / w) if abs(w) > 1e-6 else '—',
                dpath / dur, v, dpath))
            wr = ((dl - dr) * RAD) / SEP / dur          # 两轮差速反推的车体角速度
            if abs(wr) > 1e-9 and abs(dyaw / dur) > 1e-3:
                print('  轮差反推 ω=%+.3f vs 展开真值 ω=%+.3f  ->  %s' % (
                    wr, dyaw / dur,
                    '吻合（轮在滚，没打滑）' if abs(wr - dyaw / dur) < 0.3 * max(abs(wr), 0.3)
                    else '不吻合（轮在打滑或读数是假的）'))
        if abs(v) < 1e-9 and abs(w) < 1e-9:
            print('  >> 零指令段：%s（非己指令到达 %d 条，最近 %s@%.2f m）' % (
                '两轮仍在动，外力或在发指令' if (abs(dl) > 0.5 or abs(dr) > 0.5) else '两轮不动',
                n.n_stray, n.near[0], n.near[1]))
        hold(n, a.gap, 0.0, 0.0, tag + '-空档')
    print('\n=== 终点 ===')
    print(fmt(snap(n), '终点'))
    print('  全程 cmd 到达 %d 条，其中非己指令 %d 条，最后一条非己 (t, v, w)=%s' % (
        n.n_cmd, n.n_stray, n.stray_last))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
