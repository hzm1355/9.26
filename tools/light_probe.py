#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""光照预设采样：切一档、验若干帧、打一行，全程不等车。

第二版改了两处，因为第一版的数不可信（同一档重测能给出互相矛盾的结果）：

  · 切换后先**丢弃** warm 秒内的帧，再开始采样。~/scene 与 ~/light/modify 的消息
    是排进队列、等 Scene::PreRender()（相机出帧前）才落到 OGRE 的，而
    /gate/lighting/current 的回读是在插件的 ROS 回调里发的，比渲染侧真正消化完
    **更早**。第一版拿 ack 当生效点立刻取帧，取到的很可能还是上一档的图。

  · 一个窗口里统计"出的帧数"和"不同的帧数"，不再只报最新一帧的均值。
    某一档三个样本给出一模一样的数字时，"这盏灯对这块相机没影响"和
    "这台相机根本没再出新帧"是两件完全不同的事，只看均值分不开；
    帧内容哈希能把它们分开。

用法：
  python3 light_probe.py [day backlit lowlight night]
  python3 light_probe.py --warm 1.0 --settle 1.5 custom:0.2,0.3,0.3,0,0 custom:0.2,0.3,0.3,0,1
把两个 custom 串交替多写几遍就是 A/B（顺序反转也一样写），用来判断结果是否与顺序有关。
"""
import hashlib
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import Image

TOPICS = ['/gate_j3_north/gate_cam/image_raw', '/gate_j2_south/gate_cam/image_raw']
ORDER = ['day', 'backlit', 'lowlight', 'night']
CN = {'day': '白天', 'backlit': '逆光', 'lowlight': '弱光', 'night': '夜间'}


class Probe(Node):
    def __init__(self, presets, warm, settle):
        super().__init__('light_probe')
        self.frames = {}
        self.current = None
        self.warm = warm
        self.settle = settle
        self.pub = self.create_publisher(String, '/gate/lighting', 10)
        self.create_subscription(String, '/gate/lighting/current',
                                 lambda m: setattr(self, 'current', m.data), 10)
        for t in TOPICS:
            self.create_subscription(
                Image, t, lambda m, k=t: self.on_frame(k, m), 10)

    def on_frame(self, topic, m):
        a = np.frombuffer(m.data, dtype=np.uint8)
        if a.size != m.height * m.width * 3:
            return
        g = a.reshape(m.height, m.width, 3).mean(axis=2).astype(np.uint8)
        self.frames.setdefault(topic, []).append((
            hashlib.sha1(a).hexdigest()[:8],
            (float(g.mean()), float(g.std()), len(np.unique(g >> 3)),
             int(g.max()), int(g.min()))))

    def spin_to(self, dur):
        end = time.time() + dur
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sample(self, p):
        """切到 p，等渲染侧消化，返回每话题的 (帧数, 唯一帧数, 末帧统计)。"""
        self.current = None
        for _ in range(5):
            self.pub.publish(String(data=p))
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        t0 = time.time()
        while rclpy.ok() and self.current != p and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        acked = self.current == p

        self.frames.clear()
        self.spin_to(self.warm)          # 旧光照的残帧全部丢掉
        self.frames.clear()
        self.spin_to(self.settle)

        out = {}
        for t in TOPICS:
            fr = self.frames.get(t, [])
            uniq = len({h for h, _ in fr})
            out[t] = (len(fr), uniq, fr[-1][1] if fr else None)
        return acked, out


def main():
    # --warm/--settle 支持 "--warm 1.0" 的写法，所以取参数时要连着跳过它的值，
    # 否则 "1.0" 会被当成一个预设名发给插件（第一版就这么把两行垃圾发出去了）。
    args, keys = [], ('--warm', '--settle')
    skip = False
    for a in sys.argv[1:]:
        if skip:
            skip = False
            continue
        if a in keys:
            skip = True
            continue
        if a.startswith('--'):
            continue
        args.append(a)

    def opt(name, default):
        v = [a for a in sys.argv[1:]
             if a.startswith('--%s' % name) and '=' in a]
        if v:
            return float(v[0].split('=', 1)[1])
        w = sys.argv[1:]
        i = w.index(name) if name in w else -1
        return float(w[i + 1]) if i >= 0 and i + 1 < len(w) else default
    warm, settle = opt('--warm', 1.0), opt('--settle', 1.5)

    rclpy.init()
    node = Probe(args or ORDER, warm, settle)
    t0 = time.time()
    while rclpy.ok() and node.pub.get_subscription_count() == 0 \
            and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pub.get_subscription_count() == 0:
        print('!! /gate/lighting 无订阅者，v2x_lighting 插件未加载')
        rclpy.shutdown()
        return 1

    print('%-24s %-16s %5s %5s %8s %7s %5s %5s %5s' % (
        '预设', '卡口', '帧数', '唯一', '均值', '标准差', '灰度', '最亮', '最暗'))
    for p in args or ORDER:
        acked, res = node.sample(p)
        for t in TOPICS:
            n, u, s = res[t]
            short = t.split('/')[1]
            if s is None:
                print('%-24s %-16s  窗口内没出帧' % (p, short))
            else:
                print('%-24s %-16s %5d %5d %8.1f %7.1f %5d %5d %5d%s' % (
                    p + ('' if acked else '(无回读)'), short, n, u, *s,
                    '  <-- 冻结' if u == 0 or n == 0 else
                    ('  末帧=首帧' if u == 1 and n > 1 else '')))
        print('   (%s)' % CN.get(p, ''))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
