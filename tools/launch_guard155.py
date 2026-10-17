#!/usr/bin/env python3
"""摆位之后的"起步实测速度闸"：先刹轮，再用一小脚试探油门，看车**实际**跑多快。

为什么 1.5 s 零速刹轮不够（155 轮实测）：
  * place_car6_144.sh 摆完 4 s 位移 0.00 m、体检通过，`gz model` 读数干净；
  * 摆渡车节点 14:51:58 接手，14:52:08 读到 (12.00,-12.51)（北行 1.44 m，巡航速度量级，
    看着完全正常）；
  * 14:52:22 已经在 (8.92,-2.60)、roll=180°、z=7.15 —— 14 s 内被甩出 10.4 m，
    平均 0.74 m/s；入口闸 P0005 的"到场→离场"只用了 2.74 s 跨过 3.20 m 占用框
    （1.17 m/s），而同一方向上一轮 P0002 用了 17.86 s（0.25 m/s，正常）。
  合起来就是 memory「摆位弹射」的第三种释放时机 —— **驱动节点接手并开始给油那一下**，
  楔住期间攒下的轮位目标误差一次性放出来。静止 4 s 的体检只能证明"没在动"，证明不了
  "误差已经排空"，因为它没有被激励。

所以这里补的正是"加激励再复核效果"（memory「生效要用效果复核」）：给 0.10 m/s 四秒，
理论位移 ~0.4 m、峰值 ~0.12 m/s；实测峰值超过 cap（默认 0.40 m/s，即巡航的 1.6 倍）
就判"残余误差还在"，立刻连发零速并返回 2，调用方延长刹轮后重试，**不要**起摆渡车。

速度差分必须用 /clock（memory「仿真速度差分要 /clock 驱动」）：本项目 RTF 实测约 0.63，
拿墙钟算会把速度低估三成，正好把 0.4 的阈值钝化成一个永远不会触发的数。model_states
要按 sim 时刻去重（同一钟刻的重复帧会把分母抹成 0）。
"""
import collections
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

NAME = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
BRAKE_S = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
PROBE_V = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
PROBE_S = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
CAP = float(sys.argv[5]) if len(sys.argv) > 5 else 0.40


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class Guard:
    def __init__(self):
        rclpy.init()
        self.nd = Node('launch_guard155')
        self.bq = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.pose = None
        self.clock = None
        self.hist = collections.deque()
        self.nd.create_subscription(ModelStates, '/model_states', self.on_states, self.bq)
        self.nd.create_subscription(Clock, '/clock', self.on_clock, self.bq)
        # 发布端必须 RELIABLE（drive_probe153.py 里记着：BEST_EFFORT 的 publisher 和
        # gz diff_drive 的默认可靠订阅不兼容，DDS 一条都不投，车不动会被误读成卡死）。
        self.pub = self.nd.create_publisher(
            Twist, '/' + NAME + '/cmd_vel', QoSProfile(depth=10))

    def on_states(self, m):
        for i, n in enumerate(m.name):
            if n == NAME:
                p = m.pose[i].position
                t = self.clock if self.clock is not None else time.time()
                self.pose = (p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
                if self.pose[2] < 6.5:
                    print('  !! z=%.3f 掉出台面' % self.pose[2])
                if self.hist and t - self.hist[-1][0] > 1e-6:
                    self.hist.append((t, p.x, p.y))
                    if len(self.hist) > 400:
                        self.hist.popleft()
                elif not self.hist:
                    self.hist.append((t, p.x, p.y))

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.nd, timeout_sec=0.02)

    def send(self, v, secs):
        t0 = time.time()
        n = 0
        while time.time() - t0 < secs:
            m = Twist()
            m.linear.x = v
            self.pub.publish(m)
            n += 1
            rclpy.spin_once(self.nd, timeout_sec=0.01)
        return n

    def peak_speed(self, win=0.5):
        """滑窗实测峰值速度（m/s，仿真时钟）。窗口不足时返回 None。"""
        best = None
        h = list(self.hist)
        for i in range(len(h)):
            for j in range(i + 1, len(h)):
                dt = h[j][0] - h[i][0]
                if dt < win:
                    continue
                if dt > win + 1.0:
                    break
                v = math.hypot(h[j][1] - h[i][1], h[j][2] - h[i][2]) / dt
                if best is None or v > best:
                    best = v
                break
        return best


def main():
    g = Guard()
    g.spin(3.0)
    if g.pose is None:
        print('FAIL 3 s 内没收到 %s 位姿' % NAME)
        rclpy.shutdown()
        return 1
    print('  起始 (%.3f, %.3f) z=%.3f roll 未查 yaw=%+.3f'
          % (g.pose[0], g.pose[1], g.pose[2], g.pose[3]))
    try:
        nsub = g.pub.get_subscription_count()
    except Exception:
        nsub = -1
    print('  /%s/cmd_vel 订阅者数 = %d（0 说明没有驱动者在线，试探油门会毫无反应，'
          '此时"没动"不能当成"误差已排空"）' % (NAME, nsub))
    if nsub == 0:
        print('FAIL 没有 cmd_vel 订阅者，无法做激励复核')
        rclpy.shutdown()
        return 1
    if g.clock is None:
        print('  注意：3 s 内没收到 /clock，速度将按墙钟差分（RTF<1 时会低估）')

    n = g.send(0.0, BRAKE_S)
    print('  刹轮 %+.1f s 发了 %d 帧零速，末位 (%.3f, %.3f)'
          % (BRAKE_S, n, g.pose[0], g.pose[1]))

    g.hist.clear()
    # 位移基准必须取**试探油门开始的那一刻**，不是刹轮之前：拿刹轮前的点位算，会把刹轮
    # 期间的漂移也算进"油门造成的位移"里，155 轮就是这么把一个顶死的车读成"倒得动"的。
    xp, yp = g.pose[0], g.pose[1]
    n = g.send(PROBE_V, PROBE_S)
    dd = math.hypot(g.pose[0] - xp, g.pose[1] - yp)
    peak = g.peak_speed()
    theory = PROBE_V * PROBE_S
    print('  试探油门 v=%.2f 发 %d 帧 / %.1f s：位移 %.3f m（理论约 %.2f m），'
          '峰值实测速度 %s' % (PROBE_V, n, PROBE_S, dd, theory,
                              ('%.3f m/s' % peak) if peak is not None else '样本不足'))
    g.send(0.0, 3.0)
    print('  收尾 (%.3f, %.3f) z=%.3f yaw=%+.3f'
          % (g.pose[0], g.pose[1], g.pose[2], g.pose[3]))
    rclpy.shutdown()

    if peak is None:
        print('FAIL 位姿样本不足，判不了峰值 —— 别起摆渡车')
        return 1
    if peak > CAP:
        print('FAIL 峰值 %.3f m/s 超过上限 %.2f m/s：残余轮位误差还在往外放，'
              '继续刹轮后重测，此时起摆渡车就会重演 155 轮的弹射' % (peak, CAP))
        return 2
    # 下限判据按**位移占指令理论值的比例**，不按瞬时峰值：155 轮实测 v=-0.10 × 6 s 只挪了
    # 0.031 m（理论 0.60 m，5%），可峰值却有 0.062 m/s —— 那是顶住之前的一小段滑动，
    # 只查 "峰值 > 0.02" 会把它放成 PASS（本轮就是这么误判过一次）。峰值会因一次瞬滑达标，
    # 位移不会：真的在走才会累积出与指令同量级的行程。
    if dd < 0.5 * abs(theory):
        print('FAIL 只走 %.3f m，指令理论 %.2f m 的 %.0f%%：给油不响应（被顶住或插件实例'
              '没出力），先查驱动通路再起车' % (
                  dd, abs(theory), 100.0 * dd / max(abs(theory), 1e-9)))
        return 4
    print('PASS 位移 %.3f m / 峰值 %.3f m/s 都在指令 v=%.2f 的量级内，驱动通路是活的'
          % (dd, peak, PROBE_V))
    return 0


if __name__ == '__main__':
    sys.exit(main())
