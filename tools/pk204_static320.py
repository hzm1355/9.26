#!/usr/bin/env python3
# 摆位之后的"钉得住"闸：一台车都不许动，且必须站在水平台面上、姿态是正的。
#
# 为什么单列一道闸而不是摆完就重起驱动：本轮要摆的是**翻倒 + 掉出台面**的 car_3
# （实测 (26.99,-12.48) z=0.45 roll=3.14159）和停在南口格外的 car_2。memory
# 「摆位弹射与复位风暴」记的第一类能量源就是摆位本身 —— 零速没灌够、或者落点跟
# 别的件互穿，摆完那一下就会回弹几米，事后 drive 节点一接手就把它当成"跑偏了"，
# 整轮又得重来。所以这里量的是**摆完之后**这 10 s：
#   * 累计路程 < 0.15 m —— 没回弹（0.15 m 比摆位回弹的 >4 m 小两个量级，比纯噪声大）；
#   * z 落在 6.55..6.78 —— 还在沙盘台面上（台面 z≈6.7，台外地面 z=0，掉出去一眼就露）；
#   * |roll|、|pitch| < 0.25 rad —— 是正着站的（翻车那一例 roll=3.14159）。
# 用法：python3 tools/pk204_static320.py [秒数] [模型名...]
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
MODELS = sys.argv[2:] or ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']
MAX_PATH = 0.15
Z_LO, Z_HI = 6.55, 6.78
MAX_TILT = 0.25


class Sampler(Node):

    def __init__(self):
        super().__init__('pk204_static')
        self.samples = {m: [] for m in MODELS}
        self.create_subscription(ModelStates, '/model_states', self.on_ms, 10)

    def on_ms(self, msg):
        now = time.monotonic()
        for name in MODELS:
            if name in msg.name:
                i = msg.name.index(name)
                p, o = msg.pose[i].position, msg.pose[i].orientation
                roll = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x * o.x + o.y * o.y))
                pitch = math.asin(max(-1.0, min(1.0, 2 * (o.w * o.y - o.z * o.x))))
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                self.samples[name].append((now, p.x, p.y, p.z, roll, pitch, yaw))


def main():
    rclpy.init()
    node = Sampler()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(DUR)
    ex.shutdown()

    print('  %-7s %-9s %-9s %-8s %-8s %-8s %-16s %s' % (
        '车', '累计路程', '离起点', 'z', 'roll', 'pitch', '末态(x,y)', '判读'))
    bad = []
    for m in MODELS:
        s = node.samples[m]
        if len(s) < 5:
            print('  %-7s 只采样到 %d 帧 —— /model_states 里没有这台车' % (m, len(s)))
            bad.append(m)
            continue
        path = sum(math.hypot(s[i][1] - s[i - 1][1], s[i][2] - s[i - 1][2])
                   for i in range(1, len(s)))
        net = math.hypot(s[-1][1] - s[0][1], s[-1][2] - s[0][2])
        why = []
        if path >= MAX_PATH or net >= MAX_PATH:
            why.append('动了')
        if not (Z_LO < s[-1][3] < Z_HI):
            why.append('不在台面')
        if abs(s[-1][4]) > MAX_TILT or abs(s[-1][5]) > MAX_TILT:
            why.append('姿态不正')
        print('  %-7s %8.3f m  %8.3f m  %7.3f  %+7.3f  %+7.3f  (%6.2f,%6.2f)  %s' % (
            m, path, net, s[-1][3], s[-1][4], s[-1][5], s[-1][1], s[-1][2],
            '钉住了' if not why else '、'.join(why)))
        if why:
            bad.append(m)
    rclpy.shutdown()
    if bad:
        print('STATIC320 FAIL：%s' % ' '.join(bad))
        return 1
    print('STATIC320 PASS：%d 台全部钉在住、在台面上、姿态正' % len(MODELS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
