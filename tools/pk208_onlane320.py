#!/usr/bin/env python3
# 车队"在不在自己那条车道走廊里"的独立复核（不读驱动节点的判据，自己按 yaml 算）。
#
# 为什么要第三方量一遍：signal_drive_node 的"出界"判据是 |x|>oob_x 或 |y|>oob_y 这种
# **世界坐标方框**，而它自己的入道判据是 lane_half=2.4 的**横向容差** —— 两条口径不
# 一致（西车道中心 x=-21.8，方框只到 22.0，等于只给 0.2 m；南车道 y=-16.8，方框到 19.0，
# 给 2.2 m）。本轮复位风暴就是从这条缝里起来的。所以这里按航段折线量**真实横向距离**，
# 回答两个问题：车是否还在自己的走廊里（<=lane_half），以及它落在第几段、沿线走了多远。
# 用法：python3 tools/pk208_onlane320.py [配置yaml] [秒数] [只查这几台...]
import math
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

CFG = sys.argv[1] if len(sys.argv) > 1 else (
    '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/signal_drive_params.yaml')
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
ONLY = sys.argv[3:]

with open(CFG, encoding='utf-8') as fh:
    CARS = yaml.safe_load(fh)['signal_drive']['cars']


class Sampler(Node):

    def __init__(self, names):
        super().__init__('pk208_onlane')
        self.pose = {}
        self.names = names
        self.create_subscription(ModelStates, '/model_states', self.on_ms, 10)

    def on_ms(self, msg):
        for name in self.names:
            if name in msg.name:
                i = msg.name.index(name)
                p, o = msg.pose[i].position, msg.pose[i].orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                self.pose[name] = (p.x, p.y, p.z, yaw)


def nearest(px, py, route, loop):
    """点到航段折线的最近距离与投影位置；loop=True 时把末点->首点那段也算上"""
    segs = []
    n = len(route)
    for i in range(n - 1 if not loop else n):
        a = route[i]
        b = route[(i + 1) % n]
        segs.append((a, b))
    best = None
    for (ax, ay), (bx, by) in segs:
        dx, dy = bx - ax, by - ay
        L = math.hypot(dx, dy) or 1.0
        t = ((px - ax) * dx + (py - ay) * dy) / (L * L)
        t = max(0.0, min(1.0, t))
        cx, cy = ax + dx * t, ay + dy * t
        d = math.hypot(px - cx, py - cy)
        cand = (d, ax, ay, bx, by, t * L, L)
        if best is None or cand[0] < best[0]:
            best = cand
    return best


def main():
    names = ONLY or sorted(CARS)
    rclpy.init()
    node = Sampler(names)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(DUR)
    ex.shutdown()

    print('  %-7s %-18s %-9s %-9s %-22s %s' % (
        '车', '位置(x,y)', '离中心线', 'z', '最近航段', '判读'))
    for name in names:
        p = node.pose.get(name)
        if p is None:
            print('  %-7s /model_states 里读不到' % name)
            continue
        cfg = CARS[name]
        route = [tuple(x) for x in cfg['route']]
        loop = str(cfg.get('mode', 'loop')) == 'loop'
        d, ax, ay, bx, by, along, L = nearest(p[0], p[1], route, loop)
        half = 2.4
        for ob in cfg.get('obey') or []:
            half = max(half, float(ob.get('lane_half', 2.4)))
        if not cfg.get('drive', True):
            verdict = 'drive:false（不走这条路线，由摆渡节点负责）'
        elif d <= 0.6:
            verdict = '贴着中心线'
        elif d <= half:
            verdict = '走廊内（避障横偏）'
        else:
            verdict = '出走廊 %.1f m' % (d - half)
        print('  %-7s (%7.2f,%7.2f)   %7.2f m  %6.2f  (%.1f,%.1f)-(%.1f,%.1f) 走了 %.1f/%.1f  %s' % (
            name, p[0], p[1], d, p[2], ax, ay, bx, by, along, L, verdict))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
