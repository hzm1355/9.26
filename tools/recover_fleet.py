#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把五辆车逐个摆回各自车道起点：摆之前先清出净空，摆之后用"摆位之后的位移"验停稳。

为什么不用现成的 place_cars.py / reset_fleet.sh：它们按配置算出目标点就直接摆，
**不看那个点上现在有没有别的车**。本轮取证连失败 5 次的原因就是这个 —— 摆下去的车
与正在经过的车体重叠，接触求解器把两辆车沿车头方向弹射出去（why_move_probe.py 实测
8 m/s、z 全程不变、cmd_vel 发布者数=0、odom 与位姿微分一致），被弹出去的车又落到
下一个摆位点上，形成级联。place_cars.py 的 GAP 只约束"它自己摆的这几辆"互相间隔，
管不了现场正在乱跑的那几辆。

两处判据修正（第 1 次实测踩到的坑）：
  A) 停稳位移必须以 **place() 返回之后**的位姿为零点。先前在摆位前取 p0，
     将传送距离算进了"4 s 位移"里：car_1 明明第 1 次就摆准了（落点偏差 0.00 m），
     却被报成 7.76 m 位移、判成"被弹走"。
  B) 目标点被一辆**静止的**（驱动已停）车占住时，等它"走开"是等不来的。
     实测 `目标点被占：car_3 3.4 m` 连打 12 次、零进展。改成递归：
     先把堵路车摆到它自己的目标点，再回来摆这一辆。
     （五个目标点两两相距 ≥12 m，所以全都摆准之后净空条件自然成立。）

净空半径取 3.0 m 而不是 3.5 m：弹射要求车体**真的重叠**（底盘 1.6×0.8 m），
3.4 m 的间距并不重叠，用 3.5 m 会把大量安全落点误判成"被占"。
"""
import argparse
import math
import os
import time

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

_T0 = time.monotonic()
CLEAR_R = 3.0          # 净空半径（见上面注释 B 的说明）
SETTLE_S = 4.0         # 摆位后观察停稳的时长
DRIFT_MAX = 0.6        # 停稳判据：观察窗内位置增量上限
STILL_M = 0.30         # 全场静止判据：3 s 窗口内位置增量上限
SANITY_R = 26.0        # 场界：|x| 或 |y| 超过它说明车已经飞出沙盘，空中无摩擦会越落越快


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recover(Node):
    def __init__(self, z):
        super().__init__('recover_fleet')
        self.z = z
        self.latest = {}               # name -> (x, y, yaw, z)
        self.hold = {}
        for n in ('car_1', 'car_2', 'car_3', 'car_4', 'car_5'):
            self.hold[n] = self.create_publisher(Twist, '/%s/cmd_vel' % n, 10)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_timer(0.05, self.on_hold)
        self.cli_put = self.create_client(SetEntityState, '/set_entity_state')

    def on_states(self, msg):
        for n, p in zip(msg.name, msg.pose):
            if n.startswith('car_'):
                self.latest[n] = (p.position.x, p.position.y,
                                  yaw_of(p.orientation), p.position.z)

    def on_hold(self):
        for pub in self.hold.values():
            pub.publish(Twist())

    def spin_wait(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_service(self, cli, who):
        t0 = time.monotonic()
        while not cli.wait_for_service(timeout_sec=3.0):
            if time.monotonic() - t0 > 60:
                raise SystemExit('!! %s 60 s 内没发现服务' % who)

    def place(self, name, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = self.z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        self.wait_service(self.cli_put, '/set_entity_state')
        fut = self.cli_put.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return bool(fut.result() and fut.result().success)

    def others_near(self, x, y, skip, radius=CLEAR_R):
        out = []
        for n, p in self.latest.items():
            if n == skip or not n.startswith('car_'):
                continue
            d = math.hypot(p[0] - x, p[1] - y)
            if d < radius:
                out.append((n, d))
        return sorted(out, key=lambda t: t[1])

    def wait_still(self, name, budget=45.0):
        """等这辆车自己停下来（判据只用位置增量，/model_states 的 twist 列不可信）。"""
        t0 = time.monotonic()
        last = self.latest.get(name)
        while time.monotonic() - t0 < budget:
            self.spin_wait(2.0)
            now = self.latest.get(name)
            if now and last:
                if math.hypot(now[0] - last[0], now[1] - last[1]) < 0.15:
                    return True
            last = now
        return False

    # ------------------------------------------------------------------ 两道前置闸
    def sane(self, name):
        """这辆车还在沙盘里、还在地面上吗。twist 列不可信，只看位姿本身。"""
        p = self.latest.get(name)
        if p is None:
            return False
        return (abs(p[0]) <= SANITY_R and abs(p[1]) <= SANITY_R
                and abs(p[3] - self.z) <= 1.2)

    def rescue(self, tgmap, done):
        """把飞出沙盘 / 掉穿地面的车先拉回各自目标点。

        这种车不该留给驱动节点的越界看门狗去复位：`reset_car` 是一次
        /set_entity_state 瞬移，如果它发生在测速跑动的半途，抓拍到的就是米级跳帧，
        跳变护栏会把这一趟整趟作废 —— 现场证据直接没了。所以越界必须在开跑前清干净。
        """
        bad = sorted(n for n in tgmap if n not in done and not self.sane(n))
        if not bad:
            return
        print('  越界/掉落地：%s —— 先拉回各自目标点' % ', '.join(bad))
        for n in bad:
            x, y, yaw = tgmap[n]
            p = self.latest.get(n)
            if p:
                print('    %s 现在 (%.1f,%.1f) z=%.1f' % (n, p[0], p[1], p[3]))
            for _ in range(2):
                self.place(n, x, y, yaw)
                self.spin_wait(2.0)
                if self.sane(n):
                    break
            if not self.sane(n):
                print('    !! %s 拉不回来（话题没它？），后面按缺席处理' % n)

    def calm(self, budget=90.0, thresh=STILL_M):
        """等全场静止：3 s 窗口内每辆车位移都 < thresh 才算稳。

        弹射速度还没衰减完就摆下一辆，正在滑的车会正好滑进新的落点，又是一次重叠弹射
        —— 上一轮连失败 5 次的级联就是这么滚起来的。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < budget:
            self.spin_wait(1.5)
            snap = dict(self.latest)
            self.spin_wait(3.0)
            moving = []
            for n, p in self.latest.items():
                q = snap.get(n)
                if not q:
                    continue
                d = math.hypot(p[0] - q[0], p[1] - q[1])
                if d > thresh:
                    moving.append('%s %.1f m' % (n, d))
            if not moving:
                print('  全场静止（3 s 窗内位移均 < %.2f m）' % thresh)
                return True
            print('  仍在滑动：%s —— 继续等' % ', '.join(moving))
        print('  !! %.0f s 内没停干净，带着滑动继续（靠摆后停稳判定重试兜底）' % budget)
        return False

    # ------------------------------------------------------------------ 单车恢复
    def recover(self, name, x, y, yaw, tgmap, done, chain=(), depth=0):
        """把 name 摆到 (x,y,yaw)。堵在落点上的别车如果还没摆过，先递归摆它。"""
        if name in done:
            return True
        blockers = [n for n, _ in self.others_near(x, y, name) if n not in chain]
        for b in blockers:
            if depth >= 3:
                print('    递归深度到顶，不再挪 %s（硬摆，靠停稳判定兜底）' % b)
                continue
            bx, by, byaw = tgmap[b]
            print('    落点被未就位的 %s 占着 —— 先把它摆到 (%.2f,%.2f)' % (b, bx, by))
            self.recover(b, bx, by, byaw, tgmap, done, chain + (name,), depth + 1)
        # 递归回来后仍被占，只能等（多半是已就位的车，正常不会发生）
        for _ in range(3):
            near = self.others_near(x, y, name)
            if not near:
                break
            print('    落点仍有 %s —— 等 6 s' % ', '.join(
                '%s %.1f m' % t for t in near))
            self.spin_wait(6.0)

        for i in range(3):
            ok = self.place(name, x, y, yaw)
            self.spin_wait(0.5)
            p0 = self.latest.get(name)          # 必须在 place 之后取，见注释 A
            self.spin_wait(SETTLE_S)
            p1 = self.latest.get(name)
            if not (ok and p0 and p1):
                print('    第 %d 次：回执 %s，位姿读数 %s —— 重试' % (
                    i + 1, ok, bool(p1)))
                continue
            drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            err = math.hypot(p1[0] - x, p1[1] - y)
            print('    第 %d 次：落点偏差 %.2f m，摆后 %.1f s 位移 %.2f m' % (
                i + 1, err, SETTLE_S, drift))
            if drift < DRIFT_MAX and err < 1.5:
                done.add(name)
                return True
            print('    被弹走/没落位 —— 等它停下再重来')
            if self.sane(name):
                self.wait_still(name)
            # 已经越界/掉落地的不用等：空中没有摩擦，它不会自己停，
            # 下一轮的 place() 就是绝对位姿，直接把它拉回来。
        return False


def targets():
    cfg = yaml.safe_load(open(os.path.expanduser(
        '~/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml'),
        encoding='utf-8'))['signal_drive']['cars']
    GAP, placed = 5.0, []
    out = []
    for name in sorted(cfg):
        route = [tuple(p) for p in cfg[name]['route']]
        idx = int(cfg[name].get('start_idx', 0)) % len(route)
        px, py = route[(idx - 1) % len(route)]
        tx, ty = route[idx]
        dx, dy = tx - px, ty - py
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        yaw = math.atan2(uy, ux)
        back = 3.0
        while any(math.hypot(tx - ux * back - qx, ty - uy * back - qy) < GAP
                  for qx, qy in placed) and back < L - 3.0:
            back += GAP
        pos = (round(tx - ux * back, 2), round(ty - uy * back, 2))
        placed.append(pos)
        out.append((name, pos[0], pos[1], yaw))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--z', type=float, default=6.70)
    a = ap.parse_args()
    tg = targets()
    tgmap = {n: (x, y, yaw) for n, x, y, yaw in tg}
    print('摆位目标（与 place_cars.py 同一套算法）：')
    for n, x, y, yaw in tg:
        print('    %-6s (%6.2f,%6.2f) yaw=%+.2f' % (n, x, y, yaw))

    rclpy.init()
    nd = Recover(a.z)
    while not nd.latest and time.monotonic() - _T0 < 30:
        rclpy.spin_once(nd, timeout_sec=0.2)
    print('现场车辆 %d 辆：%s' % (len(nd.latest), ', '.join(
        '%s(%.1f,%.1f,%.1f)' % (n, p[0], p[1], p[3]) for n, p in sorted(nd.latest.items()))))
    nd.spin_wait(2.0)          # 攒够 /model_states 再开始判净空

    done, bad = set(), []
    nd.rescue(tgmap, done)     # 闸 1：越界/掉落地的先拉回来，别留给看门狗半途瞬移
    nd.calm()                  # 闸 2：全场停稳了才开始逐个摆，避免滑进新落点二次弹射
    for n, x, y, yaw in tg:
        print('  --- %s → (%.2f,%.2f)' % (n, x, y))
        if not nd.recover(n, x, y, yaw, tgmap, done):
            bad.append(n)
            print('    !! %s 三试未停稳，记为异常' % n)

    print('\n--- 收尾回读（落点偏差应为 0，摆后不应再漂移） ---')
    for n in sorted(nd.latest):
        p = nd.latest[n]
        g = tgmap.get(n)
        s = '偏差 %.2f m' % math.hypot(p[0] - g[0], p[1] - g[1]) if g else '-'
        if not nd.sane(n):
            s += '  [越界/离地]'
        print('    %-6s (%7.2f,%7.2f) yaw=%+6.1f°  %s' % (
            n, p[0], p[1], math.degrees(p[2]), s))
    nd.destroy_node()
    rclpy.shutdown()
    if bad:
        print('\n!! 未停稳：%s' % ', '.join(bad))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
