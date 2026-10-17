#!/usr/bin/env python3
# 效果复核用的"每台车 40 s 累计路程"计数器。
#
# 为什么不用两次位姿相减（pk200 那一版的做法）：车在**等红灯**时两次相减也是 0，
# 读起来跟"卡死"一模一样 —— 这一版量的是累计路程（沿轨迹把每 0.1 s 的位移累加），
# 等红灯的车照样有非零路程（它之前走过），真卡死的才是 0。两个数一起打，等红灯和
# 卡死就分得开：累计路程≈0 且末态离摆放点≈0 才是没走过。
#
# 采样口径沿用 tools/pk_probe308.py 踩过的两个坑：
#   * 执行器在 rclpy.init() 之后建，并放后台线程自旋 —— 主线程 spin_once(timeout)
#     采 ~100 Hz 的 /model_states 会贵到跑不满窗口就被外层看门狗 SIGTERM；
#   * /model_states 是可靠话题，BestEffort 只给 /clock 和 /link_states 用。
# 用法：python3 tools/pk203_track320.py [秒数] [模型名...]
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
MODELS = sys.argv[2:] or ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']


class Tracker(Node):

    def __init__(self):
        super().__init__('pk203_track')
        self.samples = {m: [] for m in MODELS}
        self.create_subscription(ModelStates, '/model_states', self.on_ms, 10)

    def on_ms(self, m):
        now = time.monotonic()
        for name in MODELS:
            if name in m.name:
                p = m.pose[m.name.index(name)].position
                o = m.pose[m.name.index(name)].orientation
                self.samples[name].append(
                    (now, p.x, p.y, p.z, math.atan2(2 * (o.w * o.z + o.x * o.y),
                                                    1 - 2 * (o.y * o.y + o.z * o.z))))


def main():
    rclpy.init()
    node = Tracker()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    end = time.monotonic() + DUR
    while time.monotonic() < end:
        time.sleep(0.2)
    ex.shutdown()

    print('  %-7s %-9s %-9s %-9s %-8s %-8s %s' % (
        '车', '累计路程', '离起点', '末态z', '末yaw', '首末Δyaw', '判读'))
    bad = []
    for m in MODELS:
        s = node.samples[m]
        if len(s) < 5:
            print('  %-7s 采样 %d 帧 —— 这个话题上读不到这台车（模型没上场？）' % (m, len(s)))
            bad.append(m)
            continue
        path = sum(math.hypot(s[i][1] - s[i - 1][1], s[i][2] - s[i - 1][2])
                   for i in range(1, len(s)))
        net = math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2])
        dyaw = (s[-1][4] - s[0][4] + math.pi) % (2 * math.pi) - math.pi
        # 阈值口径：0.85 m/s 巡航 × 40 s ≈ 34 m；红灯周期最长一段 ~20 s 也还剩 ~17 m。
        # 取 1.5 m 当"走过"的下限，比一次摆位回弹（>4 m）小、比纯噪声（<0.05 m）大两个量级。
        if path > 1.5:
            verdict = '在走'
        elif path > 0.15:
            verdict = '只挪了一点点（可疑）'
            bad.append(m)
        else:
            verdict = '没走过'
            bad.append(m)
        print('  %-7s %8.2f m  %8.2f m  %8.3f  %+7.3f  %+7.3f  %s' % (
            m, path, net, s[-1][3], s[-1][4], dyaw, verdict))
    rclpy.shutdown()
    print('  读数帧数（每车）：%s' % {m: len(node.samples[m]) for m in MODELS})
    if bad:
        print('TRACK320 待定：%s' % ' '.join(bad))
        return 1
    print('TRACK320 PASS：全部有车在走')
    return 0


if __name__ == '__main__':
    sys.exit(main())
