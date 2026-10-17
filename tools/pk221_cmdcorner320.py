#!/usr/bin/env python3
# 进角限速的效果复核：把控制器**自己发出去的指令速度**和"这台车离最近角点多近"配成对。
# 为什么看 cmd 而不是实测速度：实测会被复位弹射污染（pk206 读到 car_4 实测 4.34 m/s 而
# 指令为 0），那样读不出控制律本身。控制律改对没改对，指令是直接证据。
# 判据：d<2.2 m 的采样里必须 v <= max(0.12, 0.45*d)+0.05（0.05 是给 10 Hz 采样与位姿
#       差分余量，不是把阈值卡成等式）。越界记 1 次违例。
#       另外单独统计"刚换段、在新段上 0~1.2 m"这一小窗的违例数 —— 旧公式正是在这里
#       把 to_corner 读成 leg_len+1.2 从而静默失效，那才是冲过角点的直接原因。
# 用法：python3 tools/pk221_cmdcorner320.py <秒> <config yaml> [car_1 car_2 ...]
import math
import sys
import time

import rclpy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

SECS = float(sys.argv[1])
CFG = sys.argv[2]
NAMES = sys.argv[3:] or None

c = yaml.safe_load(open(CFG, encoding='utf-8'))
cars = c['cars'] if 'cars' in c else c['signal_drive']['cars']
if NAMES:
    cars = {k: v for k, v in cars.items() if k in NAMES}
ROUTES = {k: [tuple(p) for p in v.get('route', [])] for k, v in cars.items()}
print('  监控 %s，%.0f s' % (','.join(cars), SECS))

WIN, MAX_ANG, LEAD = 2.2, 0.45, 1.2


def nearest_vertex(r, x, y):
    """到路线包络里最近角点顶点的欧氏距离，以及车在该顶点之后（已换段）的段内里程。"""
    best = (1e9, 0.0, 0)
    for j in range(len(r)):
        d = math.hypot(x - r[j][0], y - r[j][1])
        if d < best[0]:
            best = (d, 0.0, j)
    d, _, j = best
    # 车在顶点 j 之后 = 沿 j 出发的下一条边的投影为正
    nx, ny = r[(j + 1) % len(r)]
    ex, ey = nx - r[j][0], ny - r[j][1]
    L = math.hypot(ex, ey) or 1.0
    along = ((x - r[j][0]) * ex + (y - r[j][1]) * ey) / L
    return d, along


class Mon:
    def __init__(self):
        self.n = rclpy.create_node('pk221_mon320')
        self.last = {}          # name -> (pose, v)
        self.rows = {k: [] for k in cars}
        self.missing = set(cars)
        self.n.create_subscription(ModelStates, '/model_states', self.on_ms, 10)
        for k in cars:
            pub = self.n.create_subscription(Twist, '/%s/cmd_vel' % k,
                                             self.mk(k), 10)
            del pub

    def mk(self, name):
        def cb(m):
            self.last.setdefault(name, [None, None])[1] = m.linear.x
        return cb

    def on_ms(self, ms):
        now = time.time()
        for i, mn in enumerate(ms.name):
            if mn in cars:
                p = ms.pose[i].position
                self.missing.discard(mn)
                self.last.setdefault(mn, [None, None])[0] = (p.x, p.y, now)

    def sample(self):
        # 位姿与指令各自异步到达，按"当前位姿 × 最近一次指令"配对；两侧都超 0.5 s 就丢弃
        for k, (pv, vv) in list(self.last.items()):
            if pv is None or vv is None:
                continue
            x, y, t = pv
            if time.time() - t > 0.5:
                continue
            d, along = nearest_vertex(ROUTES[k], x, y)
            self.rows[k].append((d, along, vv))


rclpy.init()
mon = Mon()
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(mon.n, timeout_sec=0.1)
    mon.sample()
mon.n.destroy_node()
rclpy.shutdown()

viol_all = viol_sw = tot = 0
for k in cars:
    rows = mon.rows[k]
    near = [(d, a, v) for (d, a, v) in rows if d < WIN]
    vv = [(d, a, v) for (d, a, v) in near
          if v > max(0.12, MAX_ANG * max(d, 0.25)) + 0.05]
    sw = [(d, a, v) for (d, a, v) in near if 0.0 <= a <= LEAD]
    swv = [(d, a, v) for (d, a, v) in sw
           if v > max(0.12, MAX_ANG * max(d, 0.25)) + 0.05]
    vmax = max((v for (_, _, v) in near), default=float('nan'))
    print('  %-6s 采样%d 角点2.2m内%d 违例%d(换段窗%d/%d) 窗内最大指令%.2f m/s'
          % (k, len(rows), len(near), len(vv), len(swv), len(sw), vmax))
    if near:
        b = sorted(near)[-4:]
        print('         最贴角的几对 (d,段内里程,指令)：%s'
              % ' '.join('(%.2f,%.2f,%.2f)' % t for t in b))
    viol_all += len(vv)
    viol_sw += len(swv)
    tot += len(rows)
print('  合计 采样%d 进角违例%d 其中换段窗%d' % (tot, viol_all, viol_sw))
if tot == 0:
    print('PK221NODATA 一个采样都没配上的原因：位姿或指令任一侧没到（查 cmd_vel 是否还在发）')
    sys.exit(4)
print('PK221' + ('PASS' if viol_all == 0 else 'VIOL'))
