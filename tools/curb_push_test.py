#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""顶推测试：证明 B 类设施等效盒真的"挡得住"，而不只是"在物理端注册了"。

为什么还要这一发：16 分钟车队回归里车离盒最近还有 1.27 m（控制器本来就把它压在
横偏 0.83 m 内），那道物理兜底从头到尾没被用到。所以"零侵入"只证明了没副作用，
没证明正向能力。这里绕开控制器直接喂 cmd_vel，把车怼向盒面。

两条判据缺一不可（都用同一个正号指令，只是车头朝向相反）：
  1) 空跑对照：车头朝西（背对盒）发 +0.7 m/s，必须真的跑起来 —— 位移 <= -2 m、
     窗口峰值 >= 0.5 m/s、全程无单帧跳变。这条证明"指令能驱动车"，于是顶推阶段
     的"停住"就不可能是链路断了或差速插件没动。
  2) 顶住：车头朝东对准盒面发同一条 +0.7 m/s，先跑起来（位移 >= 1 m），然后停在
     盒面前（带符号间隙 >= -0.05 m）、末段 2 s 速度均值 < 0.15 m/s，而指令全程没撤。

为什么不用"反向指令"做对照（上一版的设计）：那两次 -0.7 m/s 实测都发生在车头已经
顶住盒面的状态下（车先被摆位弹射甩到墙边，再收负速，纹丝不动），根本分不开
"插件不认负速"和"顶住之后倒不出来"。再加上 set_entity_state 带着空转轮子摆位会
瞬移好几米（tools/cmd_sign_probe.py 实测一次 9.21 m），"倒车对照"量到的永远是
被弹出去的距离。同一个正号指令 + 两个朝向，才把"能不能跑"和"跑起来被什么挡住"
这两件事真正分开 —— 也不依赖任何关于负速能否驱动的未证实结论。

前提：先杀掉 signal_drive_node，否则控制器每 1 s 覆盖一次 cmd_vel，两边抢方向盘。

用法（WSL，需先 source /opt/ros/humble 与 ~/v2x_ws/install/setup.bash）：
  python3 tools/curb_push_test.py [车名] [顶推秒数]
  python3 tools/curb_push_test.py car_4 12
"""
import csv
import json
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

CAR = sys.argv[1] if len(sys.argv) > 1 else 'car_4'
PUSH_SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
FREE_SEC = 4.0
CMD_V = 0.7
Z = 6.70
CAR_HALF_LEN = 0.8           # chassis_collision 1.6×0.8×0.4，沿车体 +x 半长 0.8
CAR_Z0, CAR_Z1 = 7.00, 7.50  # 轮底 7.00 到底盘顶 7.45，盒必须压住这段才挡得住
STAGE_BACK = 2.5             # 出发点离盒面 2.5 m：够跑出速度，又不至于撞歪
FREE_CLEAR = 5.0             # 空跑对照往西要 4 s x 0.7 m/s ~ 2.8 m，留 5 m 净空
JUMP = 0.5                   # 单帧位移超过它就是瞬移/弹射，不是行驶
BOX_JSON = '/tmp/world_collisions.json'
CSV_OUT = '/tmp/curb_push_%s.csv' % CAR

# gazebo_ros 的 /clock 是 best_effort 发的，用默认 reliable 订阅会被 DDS 直接判
# 不兼容（"offering incompatible QoS ... Last incompatible policy: RELIABILITY"），
# 一条都收不到，仿真时间恒 0，差分全废。sensor_data 预设就是 best_effort+volatile。
CLOCK_QOS = qos_profile_sensor_data


class PushTest(Node):

    def __init__(self):
        super().__init__('curb_push_test')
        self.samples = []          # (sim_t, x, y)
        self.sim_t = 0.0
        self.latest = {}
        self.create_subscription(ModelStates, '/model_states', self._on_states,
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self._on_clock, CLOCK_QOS)
        self.cmd = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, 10)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

    def _on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_states(self, msg):
        if CAR in msg.name:
            p = msg.pose[msg.name.index(CAR)].position
            self.latest = {'x': p.x, 'y': p.y, 'z': p.z}
            self.samples.append((self.sim_t, p.x, p.y))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, vx):
        t = Twist()
        t.linear.x = vx
        self.cmd.publish(t)

    def place(self, x, y, yaw):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = CAR
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = Z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)


def z_overlaps_car_band(aabb):
    return aabb[4] < CAR_Z1 and aabb[5] > CAR_Z0


def corridor_clash(boxes, cand, lo, hi, cy):
    """走廊 [lo, hi] x y=cy±0.45 内是否存在 cand 之外的盒（等效盒也算）。"""
    return [o for o in boxes
            if o is not cand and z_overlaps_car_band(o['aabb'])
            and o['aabb'][0] < hi and o['aabb'][1] > lo
            and o['aabb'][2] < cy + 0.45 and o['aabb'][3] > cy - 0.45]


def pick_blocker(node):
    """挑一个能当靶子的等效盒：南北够宽，且从"空跑最西点"一直到盒面整条走廊净空。

    走廊必须把其他等效盒也算进去。更早一版只排非 fixtures 的盒，结果靶子选了
    c35（盒面 x=20.00），可车实际被走廊里漏算的 c31（盒面 x=19.00）顶在
    x=18.20，前后两块盒一夹，位移 0.00 m —— 看着像 cmd 链路坏了，其实是布点
    自己造出来的夹缝。
    """
    with open(BOX_JSON, encoding='utf-8') as fh:
        boxes = json.load(fh)['boxes']
    here = node.latest
    best, diag = None, []
    for b in boxes:
        if b['name'] != 'sandbox_fixtures_coll' or not z_overlaps_car_band(b['aabb']):
            continue
        if b['aabb'][3] - b['aabb'][2] < 0.9:
            continue
        x0, cy = b['aabb'][0], 0.5 * (b['aabb'][2] + b['aabb'][3])
        sx = x0 - STAGE_BACK
        clash = corridor_clash(boxes, b, sx - FREE_CLEAR, x0 - 0.05, cy)
        d = math.hypot(sx - here['x'], cy - here['y'])
        if not clash:
            if best is None or d < best[0]:
                best = (d, b, sx, cy)
        else:
            diag.append((d, b['coll'], ['%s@x%.2f' % (o['coll'], o['aabb'][0])
                                        for o in clash[:3]]))
    if best is None and diag:
        diag.sort()
        print('!! 没有净空靶子，最近 5 个候选被这些盒挡住：')
        for d, coll, hits in diag[:5]:
            print('   %s 距 %.1f m  <- %s' % (coll, d, ', '.join(hits)))
    return best


def speed_series(samples, win=0.2):
    """按仿真时钟做窗口差分（一个窗口跨多帧，把整段位移除以整段时长）。

    必须用 /clock 而不是墙钟，而且必须按窗口累积：/clock 是 0.1 s 一跳，
    /model_states 却按物理步 ~90 Hz 发，拿"相邻两帧位移 / 相邻两帧时钟差"差分，
    分子是 1/90 s 的路程、分母是 1/10 s 的时钟步，结果整体缩到约 1/10 ——
    0.7 m/s 会被算成 0.07，"停住"和"没动"就分不开了。
    """
    out = []
    for i in range(1, len(samples)):
        j = i - 1
        while j > 0 and samples[i][0] - samples[j][0] < win:
            j -= 1
        dt = samples[i][0] - samples[j][0]
        if dt > 1e-6:
            out.append((samples[i][0], samples[i][1], samples[i][2],
                        math.hypot(samples[i][1] - samples[j][1],
                                   samples[i][2] - samples[j][2]) / dt))
    return out


def count_jumps(samples):
    """相邻两帧的纯位移（不看时长）超过 JUMP 的次数 —— 瞬移/弹射计数。"""
    return sum(1 for i in range(1, len(samples))
               if math.hypot(samples[i][1] - samples[i - 1][1],
                             samples[i][2] - samples[i - 1][2]) > JUMP)


def settle(node, tag):
    """等车真的停住再摆位。轮子还在空转时 set_entity_state 只挪根 link、不清关节
    角速度，落地就是个几米的弹射（实测 9.21 m），会把后面所有判据污染掉。"""
    node.send(0.0)
    for i in range(10):
        a = dict(node.latest)
        node.spin_for(1.0)
        b = node.latest
        if math.hypot(b['x'] - a['x'], b['y'] - a['y']) < 0.05:
            print('  %s 已停稳（第 %d s，位移 %.2f m）' % (tag, i + 1,
                                                          math.hypot(b['x'] - a['x'],
                                                                     b['y'] - a['y'])))
            return True
    print('!! %s 10 s 没停稳，本轮作废（脏态，先整栈重启再测）' % tag)
    return False


def stage(node, sx, sy, yaw):
    """停稳 -> 瞬移到出发点并验回弹。"""
    if not settle(node, '摆位前'):
        return False
    drift = 9.9
    for attempt in (1, 2):
        node.send(0.0)
        if not node.place(sx, sy, yaw):
            print('  set_entity_state 失败')
            return False
        node.spin_for(2.5)
        drift = math.hypot(node.latest['x'] - sx, node.latest['y'] - sy)
        print('  第 %d 次摆位（yaw=%+.2f）后漂移 %.2f m' % (attempt, yaw, drift))
        if drift <= 0.40:
            return True
    print('!! 两次摆位漂移都 >0.4 m（脏态回弹），本轮作废，先整栈重启再测')
    return False


def drive(node, sec, vx, tag):
    node.samples.clear()
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < sec and rclpy.ok():
        node.send(vx)
        node.spin_for(0.05)
        if time.time() - last > 2.0:
            last = time.time()
            p = node.latest
            print('   %s t+%4.1fs  x=%6.2f y=%6.2f  车头 x=%6.2f' % (
                tag, time.time() - t0, p['x'], p['y'], p['x'] + CAR_HALF_LEN))
    node.send(0.0)
    node.spin_for(1.0)
    ser = speed_series(node.samples)
    jumps = count_jumps(node.samples)
    print('   %s 采样 %d 条 -> 窗口差分 %d 点，瞬移 %d 处' % (
        tag, len(node.samples), len(ser), jumps))
    if not ser:
        print('  !! %s 阶段零采样' % tag)
    return ser, jumps


def main():
    rclpy.init()
    node = PushTest()
    print('== 等 /model_states 与 /clock ==')
    for _ in range(30):
        node.spin_for(1.0)
        if node.latest and node.sim_t > 0.0:
            break
    if not node.latest:
        print('!! 30 s 没收到 /model_states')
        return 1
    if node.sim_t <= 0.0:
        print('!! /clock 没给到有效仿真时间，差分必然全废，先修链路再测')
        return 1
    print('  %s 当前 (%.2f, %.2f, %.2f)  sim_t=%.1f  采样 %d 条' % (
        CAR, node.latest['x'], node.latest['y'], node.latest['z'],
        node.sim_t, len(node.samples)))
    if not node.cli.wait_for_service(timeout_sec=5.0):
        print('!! /set_entity_state 不可用')
        return 1

    picked = pick_blocker(node)
    if not picked:
        print('!! %s 里没有可用靶子盒（要求：车带高度内、南北宽 >=0.9 m、'
              '西向 %.1f m 到盒面全程净空）' % (BOX_JSON, STAGE_BACK + FREE_CLEAR))
        return 1
    dist, box, sx, sy = picked
    x0 = box['aabb'][0]
    print('== 靶子 ==')
    print('  %s 盒面 x=%.2f  y %.2f..%.2f  z %.2f..%.2f  尺寸 %.2f x %.2f x %.2f' % (
        box['coll'], x0, box['aabb'][2], box['aabb'][3], box['aabb'][4], box['aabb'][5],
        box['size'][0], box['size'][1], box['size'][2]))
    print('  出发点 (%.2f, %.2f)  距当前 %.1f m  顶住时车头应停在 x≈%.2f' % (
        sx, sy, dist, x0))

    print('== 判据一：空跑对照 %.0f s（车头朝西 +%.2f m/s，背对一切盒）==' % (
        FREE_SEC, CMD_V))
    if not stage(node, sx, sy, math.pi):
        return 1
    free, free_jumps = drive(node, FREE_SEC, CMD_V, 'free')
    if not free:
        return 1
    fx = free[-1][1] - free[0][1]
    fv = max(s[3] for s in free)
    tail = [s[3] for s in free if s[0] > free[-1][0] - 1.0]
    print('  位移 dx=%+.2f m（朝西应为负）  |v| 峰值 %.2f m/s  末段均值 %.2f m/s' % (
        fx, fv, sum(tail) / len(tail)))
    is_free = fv >= 0.5 and fx <= -2.0 and free_jumps == 0

    print('== 判据二预备：重新摆位（车头朝东对准盒面）==')
    if not stage(node, sx, sy, 0.0):
        return 1

    print('== 判据二：顶推 %.0f s（同一条 +%.2f m/s 指令不撤）==' % (PUSH_SEC, CMD_V))
    push, push_jumps = drive(node, PUSH_SEC, CMD_V, 'push')
    if not push:
        return 1
    xf, yf = push[-1][1], push[-1][2]
    vmax = max(s[3] for s in push)
    moved = push[-1][1] - push[0][1]
    tail = [s[3] for s in push if s[0] > push[-1][0] - 2.0]
    v_tail = sum(tail) / len(tail)
    gap = x0 - (xf + CAR_HALF_LEN)
    print('  终点 (%.2f, %.2f)  车头 x=%.2f  盒面 x=%.2f  带符号间隙 %+.3f m' % (
        xf, yf, xf + CAR_HALF_LEN, x0, gap))
    print('  段内位移 %+.2f m  |v| 峰值 %.2f m/s  末段(2 s)均值 %.2f m/s  瞬移 %d 处' % (
        moved, vmax, v_tail, push_jumps))
    held = (gap >= -0.05 and v_tail < 0.15 and vmax > 0.3
            and moved >= 1.0 and push_jumps == 0)

    with open(CSV_OUT, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['phase', 't_sim', 'x', 'y', 'v'])
        for s in free:
            w.writerow(['free'] + ['%.3f' % v for v in s])
        for s in push:
            w.writerow(['push'] + ['%.3f' % v for v in s])
    print('  轨迹已写 %s' % CSV_OUT)

    print('== 结论 ==')
    print('  空跑对照（同一正号指令能跑）  : %s' % ('PASS' if is_free else 'FAIL'))
    print('  顶住（被等效盒挡住）          : %s' % ('PASS' if held else 'FAIL'))
    print('  → 同一条指令在净空里跑出 %.1f m，怼向盒面时停在盒面 %+.3f m 处且速度归零，'
          '等效盒确实在物理上挡住了车' % (abs(fx), gap) if (held and is_free)
          else '  → 判据未同时成立，别下结论')
    return 0 if (held and is_free) else 2


if __name__ == '__main__':
    sys.exit(main())
