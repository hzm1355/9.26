#!/usr/bin/env python3
# car_3 不动的三态定性探针：没送到 / 没出力 / 被顶住。
# 跑之前必须先把 parking_shuttle_node 杀掉（本脚本是 /car_3/cmd_vel 的唯一发布者），
# 否则两边的指令会互相盖掉，读数说明不了任何事。
#
# 判据口径（都是这轮之前踩过的坑）：
#   * 静止复检不算证据 —— 每一段都给一小脚激励，看**实测**变化，并先取一段静止基线；
#   * diff_drive 的 odom 是底盘转发，判轮子真转没转要看 link_states 里的轮关节点速度；
#   * 位姿差分按 /clock 驱动，别用墙钟；
#   * 峰值速度要单独记，只看末态会把"弹一下又停住"读成没动；
#   * Δz 也记 —— 车掉出沙盘（路面 z≈6.7，世界地面 z=0）时水平读数会全零。
#
# 采样为什么改成后台 spin 线程：主线程用 Executor.spin_once(timeout) 采样时，实测整轮
# 跑不满三个 6 s 段就能耗到外层 timeout 的 240 s 被 SIGTERM 打断，而 rclpy 收到 TERM 会
# 关掉 context，抛出来的是 ExternalShutdownException —— 看着像 DDS/上下文坏了，其实是
# 采样方式太贵（每次调用重建 WaitSet、只取一个任务，而 /model_states+/link_states 是 ~100 Hz）。
# 用法：python3 tools/pk_probe308.py [模型名]
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, ShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock

MODEL = sys.argv[1] if len(sys.argv) > 1 else 'car_3'
FLEET = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']


class Probe(Node):
    def __init__(self):
        super().__init__('pk_probe308')
        self.pose = None
        self.twist = None
        self.wheels = {}
        self.clock = None
        self.near = {}
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % MODEL, 10)
        # 话题名点名过：这套 gz 桥发的是 /model_states、/link_states、/clock（没有 /gazebo 前缀）
        self.create_subscription(ModelStates, '/model_states', self.on_ms, 10)
        self.create_subscription(LinkStates, '/link_states', self.on_ls,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(Clock, '/clock', self.on_clk,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_clk(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_ms(self, m):
        if MODEL not in m.name:
            return
        i = m.name.index(MODEL)
        p = m.pose[i]
        t = m.twist[i]
        q = p.orientation
        # 完整 ZYX 解 roll/pitch/yaw —— 只取 z,w 会把侧躺的车读成航向正常
        sinr = 2 * (q.w * q.x + q.y * q.z)
        cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yawn = 2 * (q.w * q.z + q.x * q.y)
        yawd = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose = (p.position.x, p.position.y, p.position.z,
                     math.degrees(math.atan2(sinr, cosr)), math.degrees(pitch),
                     math.degrees(math.atan2(yawn, yawd)))
        self.twist = (t.linear.x, t.linear.y, t.angular.z)
        near = {}
        for n in FLEET:
            if n == MODEL or n not in m.name:
                continue
            j = m.name.index(n)
            near[n] = math.hypot(m.pose[j].position.x - p.position.x,
                                 m.pose[j].position.y - p.position.y)
        self.near = near

    def on_ls(self, m):
        # 连杆全名在这套 gz 桥里是 `car_3::left_wheel`（双冒号），不是 `car_3/left_wheel`。
        # 用 startswith(MODEL + '/') 过滤会一帧都匹配不上，然后读成"轮子不转"，
        # 其实是过滤式写错（tools/pk135_wheelcheck248.py 早就点过这个坑）。
        w = {}
        for i, nm in enumerate(m.name):
            owner, _, base = (nm.partition('::') if '::' in nm else nm.partition('/'))
            if owner == MODEL and any(k in base for k in ('wheel', 'caster', 'joint')):
                w[base] = m.twist[i].angular.z
        self.wheels = w


def yaw_err(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def main():
    rclpy.init()
    # 执行器必须在 rclpy.init() 之后构造：它 __init__ 里就建 GuardCondition，
    # 而 GuardCondition 要用已初始化的 context.handle（放模块级 import 时就炸）。
    ex = SingleThreadedExecutor()
    nd = Probe()
    ex.add_node(nd)

    def pump():
        try:
            ex.spin()
        except (ShutdownException, ExternalShutdownException):
            pass

    threading.Thread(target=pump, daemon=True).start()

    t0 = time.time()
    while (nd.pose is None or nd.clock is None) and time.time() - t0 < 20:
        time.sleep(0.1)
    if nd.pose is None:
        print('读不到 %s 的位姿（/model_states 没数据，或模型不在运行期 world 里）' % MODEL)
        return 1
    if nd.clock is None:
        print('!! /clock 没来 —— 下面"仿真时长"会是 nan，位姿差分只能按墙钟粗看')
    print('起始位姿 x=%.3f y=%.3f z=%.3f roll=%.2f pitch=%.2f yaw=%.2f' % nd.pose)
    print('邻车距离：%s' % {k: round(v, 2) for k, v in sorted(nd.near.items(), key=lambda x: x[1])[:3]})
    print('轮关节点（收到 %d 个 link 名匹配）：%s' % (len(nd.wheels),
          {k: round(v, 3) for k, v in sorted(nd.wheels.items())}))

    def drive(vx, wz, secs, tag):
        p0 = nd.pose
        c0 = nd.clock
        peak = 0.0
        wheel_peak = 0.0
        t_end = time.time() + secs
        while time.time() < t_end:
            tw = Twist()
            tw.linear.x = vx
            tw.angular.z = wz
            nd.pub.publish(tw)
            time.sleep(0.05)
            if nd.twist:
                peak = max(peak, math.hypot(nd.twist[0], nd.twist[1]))
            for v in nd.wheels.values():
                wheel_peak = max(wheel_peak, abs(v))
        stop = Twist()
        for _ in range(10):
            nd.pub.publish(stop)
            time.sleep(0.05)
        dt = (nd.clock - c0) if (nd.clock and c0) else float('nan')
        d = math.hypot(nd.pose[0] - p0[0], nd.pose[1] - p0[1])
        print('  [%s] 指令 v=%.2f ω=%.2f 持续 %.1f s（仿真 %.2f s）-> 位移 %.3f m、Δz %+.3f m、'
              'Δyaw %+.2f°、实测峰值线速 %.3f m/s、轮关节点峰值 %.3f rad/s'
              % (tag, vx, wz, secs, dt, d, nd.pose[2] - p0[2],
                 yaw_err(nd.pose[5], p0[5]), peak, wheel_peak))

    print('===== 0) 静止基线 3 s（不发指令，确认可读的起点是稳的）=====')
    p0 = nd.pose
    time.sleep(3)
    print('  基线位移 %.3f m、Δyaw %+.2f°'
          % (math.hypot(nd.pose[0] - p0[0], nd.pose[1] - p0[1]), yaw_err(nd.pose[5], p0[5])))
    print('===== 1) 纯转向（ω=+0.6，v=0）=====')
    drive(0.0, 0.6, 6.0, 'spin')
    print('===== 2) 纯油门（v=+0.35，ω=0）=====')
    drive(0.35, 0.0, 6.0, 'throttle')
    print('===== 3) 反向转向（ω=-0.6）=====')
    drive(0.0, -0.6, 6.0, 'spin_back')
    print('===== 4) 静默 4 s（cmd_vel_timeout 不生效，静默=保持上一条，能自走就露出来）=====')
    p0 = nd.pose
    time.sleep(4)
    print('  静默后位移 %.3f m、Δyaw %+.2f°（应≈0，否则底盘里有缓存轮速=单实例卡死）'
          % (math.hypot(nd.pose[0] - p0[0], nd.pose[1] - p0[1]), yaw_err(nd.pose[5], p0[5])))
    print('===== 收尾位姿 =====')
    print('  x=%.3f y=%.3f z=%.3f roll=%.2f pitch=%.2f yaw=%.2f' % nd.pose)
    print('  邻车距离：%s' % {k: round(v, 2) for k, v in sorted(nd.near.items(), key=lambda x: x[1])[:3]})
    print('PROBE308DONE')
    ex.shutdown()
    nd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
