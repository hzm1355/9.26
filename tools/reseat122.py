#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#122 —— 把翻出沙盘的 car_6 刹轮回正摆回等待位，然后在"轮子确实接地"的前提下重做 ω 审计。

背景（#120/#121 实测，_bp120.txt / _pf121.txt）：
  · car_6 位于 (30.876,−18.515) z=0.4000、roll=−90.0° —— 已冲出 room_33 地板东边缘
    （footprint x∈[−25,25]），从 6.3 m 高的板边翻下去侧躺在底层地面上。
  · 该车 ω=+1.0 指令下右轮 3.285 rad/s（指令 3.5 的 94%）而车体 ω=0.000
    ⇒ 轮子空转不接地。此前 #85 的"纯 ω 失效""发零速也自走"全是拿翻车在读。
  · 附带结论：E 段 7.3 s 一条 cmd_vel 都不发，右轮仍按 D 段的 ω=+1.0 转
    ⇒ **diff_drive 的 cmd_vel_timeout=0.3 不生效，静默不等于刹车**，
       停车必须显式连发零速。所以本脚本段间一律显式刹 1.5 s。

顺序：刹轮 2 s → SetEntityState 到 (12.000,−12.800,6.720) yaw=+1.5708 → 再刹 2.5 s
      → 体检（roll/pitch/z/离摆位距离）→ 负对照 → 纯ω右 → 纯ω左 → 纯v → 收尾体检。
只发 cmd_vel 和摆位，不重启、不杀进程、不删实体。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import BEST, SEP, RADIUS, wrap, yaw_of   # noqa: E402
from pose_full121 import gz_read                           # noqa: E402
import rclpy                                               # noqa: E402
from rclpy.node import Node                                 # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy         # noqa: E402
from geometry_msgs.msg import Twist                         # noqa: E402
from rosgraph_msgs.msg import Clock                         # noqa: E402
from gazebo_msgs.msg import EntityState, ModelStates        # noqa: E402
from gazebo_msgs.srv import SetEntityState                  # noqa: E402

CAR = 'car_6'
PLACE = (12.000, -12.800, 6.720, 1.5708)     # world 2903 行等待位，z 抬到板顶之上防穿透
RATE_HZ = 20.0


class Hub(Node):
    def __init__(self):
        super().__init__('reseat122')
        self.clock = 0.0
        self.pose = None        # (钟, x, y, z, yaw)
        self.prev = None
        self.yaw_acc = 0.0
        self.path_acc = 0.0
        self.n_ms = 0
        self.n_cmd = 0
        self.sent = (0.0, 0.0)
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        self.create_subscription(Twist, '/%s/cmd_vel' % CAR, self.on_cmd, BEST)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, rel)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.n_cmd += 1

    def on_s(self, m):
        if CAR not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(CAR)
        p = m.pose[i].position
        cur = (self.clock, p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
        if self.prev is None:
            self.prev = cur
        elif cur[0] - self.prev[0] > 1e-9:
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

    def wait_pose(self, secs=20.0):
        end = time.time() + secs
        while self.pose is None and time.time() < end:
            self.tick()
            time.sleep(0.02)
        return self.pose

    def brake(self, secs):
        """显式连发零速 —— 静默不刹车，只有真消息能把目标轮速清掉。"""
        self.drive(secs, 0.0, 0.0, '刹轮')

    def drive(self, secs, v, w, tag, quiet=False):
        end = time.time() + secs
        last = 0.0
        ya, pa = self.yaw_acc, self.path_acc
        n = 0
        while time.time() < end:
            self.send(v, w)
            n += 1
            self.tick()
            time.sleep(1.0 / RATE_HZ)
            self.tick()
            if time.time() - last >= 1.0 and self.pose:
                last = time.time()
                if not quiet:
                    print('    %-8s t=%9.2f (%7.3f,%7.3f) z=%6.4f 本秒 ω=%+.3f |v|=%+.3f' % (
                        tag, self.pose[0], self.pose[1], self.pose[2], self.pose[3],
                        self.yaw_acc - ya, self.path_acc - pa), flush=True)
                ya, pa = self.yaw_acc, self.path_acc
        return n

    def place(self, x, y, z, yaw):
        r = SetEntityState.Request()
        r.state = EntityState()
        r.state.name = CAR
        r.state.pose.position.x = x
        r.state.pose.position.y = y
        r.state.pose.position.z = z
        r.state.pose.orientation.z = math.sin(yaw / 2.0)
        r.state.pose.orientation.w = math.cos(yaw / 2.0)
        r.state.reference_frame = 'world'
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        res = f.result()
        if res is None:
            print('  !! SetEntityState 10 s 无回包（服务超时；gzserver 是否卡在物理步进？）')
            return False
        if not res.success:
            print('  !! SetEntityState 被拒：status=%r' % (res.status_message or ''))
        return bool(res.success)


def body(h, tag):
    """gz 轮角 + 完整 rpy + 该时刻的展开累积量。"""
    g = gz_read(CAR)
    c = h.clock
    t0 = time.time()
    while h.clock == c and time.time() - t0 < 3.0:
        h.tick()
        time.sleep(0.01)
    print('  %-6s 轮角 左%9.3f 右%9.3f 位(%7.3f,%7.3f) z=%6.4f '
          'roll=%+6.1f° pitch=%+6.1f° yaw=%+.4f 钟%9.2f 累计yaw%+8.3f 累计路%7.3f' % (
              tag, g[8], g[9], g[0], g[1], g[2], g[6], g[7], g[5], c,
              h.yaw_acc, h.path_acc), flush=True)
    return g + (c, h.yaw_acc, h.path_acc)


def seg(h, tag, v, w, secs):
    print('\n--- %s  指令 v=%+.2f w=%+.2f（%.1f s）---' % (tag, v, w, secs), flush=True)
    pre = body(h, '段前')
    h.drive(secs, v, w, tag)
    post = body(h, '段后')
    dur = post[10] - pre[10]
    if dur <= 0:
        print('  !! 窗口为 0，跳过')
        return
    dl, dr = post[8] - pre[8], post[9] - pre[9]
    dyaw, dpath = post[11] - pre[11], post[12] - pre[12]
    pl = (v - w * SEP / 2.0) / RADIUS * dur
    pr = (v + w * SEP / 2.0) / RADIUS * dur
    wl, wr_ = dl / dur, dr / dur
    wheel_w = (wr_ - wl) * RADIUS / SEP
    print('  轮角增量 左%+8.3f（预测%+7.3f 兑现%.2f）  右%+8.3f（预测%+7.3f 兑现%.2f）' % (
        dl, pl, dl / pl if abs(pl) > 0.2 else float('nan'),
        dr, pr, dr / pr if abs(pr) > 0.2 else float('nan')))
    print('  轮速 左%+7.3f 右%+7.3f rad/s  轮差反推 ω=%+.3f' % (wl, wr_, wheel_w))
    print('  车体真值 ω=%+.3f rad/s（指令 %+.3f 兑现 %s）  |v|=%+.3f（指令 %+.2f）  路径 %.3f m' % (
        dyaw / dur, w, '%.2f' % (dyaw / dur / w) if abs(w) > 1e-6 else '—',
        dpath / dur, v, dpath))
    print('  判读：%s' % ('轮速与车体吻合（接地在滚）'
                         if abs(wheel_w - dyaw / dur) < 0.3 * max(abs(wheel_w), 0.3)
                         else '轮速与车体不吻合（打滑 / 没接地 / 被顶住）'))


def main():
    rclpy.init()
    h = Hub()
    if h.wait_pose() is None:
        print('!! 没拿到 /model_states')
        return 1
    print('=== 翻车现场体检 ===')
    body(h, '现场')
    print('\n=== 1) 刹轮 2 s（把 D 段残留的 ω=+1.0 目标清掉）===')
    h.brake(2.0)
    # 摆位弹射护栏：第一次摆位会把楔住时攒的轮位误差一次性释放（本轮实测 |v|=19.9 m/s、
    # 1 s 内冲 20.9 m）。等它把能量跑完再摆第二次，连续两次都落点一致才算就位。
    chk = None
    for k in range(1, 4):
        print('\n=== 2-%d) 第 %d 次摆位到等待位 (%.3f, %.3f, %.3f) yaw=%+.4f ===' % (
            k, k, PLACE[0], PLACE[1], PLACE[2], PLACE[3]))
        if not h.place(*PLACE):
            print('  SetEntityState 失败')
            return 1
        print('  摆位后再刹 3 s 让弹射能量跑完')
        h.brake(3.0)
        chk = body(h, '体检%d' % k)
        d = math.hypot(chk[0] - PLACE[0], chk[1] - PLACE[1])
        good = abs(chk[6]) < 15 and abs(chk[7]) < 15 and 6.5 < chk[2] < 6.9 and d < 0.6
        print('  roll=%+.1f° pitch=%+.1f° z=%6.4f 离摆位 %.3f m yaw=%+.4f -> %s' % (
            chk[6], chk[7], chk[2], d, chk[5], '就位' if good else '弹开了，再来一次'))
        if good:
            break
        h.brake(2.0)
    if chk is None or not (abs(chk[6]) < 15 and abs(chk[7]) < 15
                           and 6.5 < chk[2] < 6.9
                           and math.hypot(chk[0] - PLACE[0], chk[1] - PLACE[1]) < 0.6):
        print('!! 三次摆位都没坐住，停在这里，别继续测')
        return 1
    for spec in [('负对照 静默=指令零', 0.0, 0.0, 3.0),
                 ('纯ω右', 0.0, -0.35, 3.0),
                 ('纯ω左', 0.0, 0.35, 3.0),
                 ('纯v直行', 0.31, 0.0, 3.0)]:
        seg(h, *spec)
        print('  段间显式刹 1.5 s')
        h.brake(1.5)
    print('\n=== 收尾体检 ===')
    body(h, '终点')
    print('全程 model_states %d 帧，cmd_vel 到达 %d 条' % (h.n_ms, h.n_cmd))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
