#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""流式采 /parking/shuttle_status，每行前缀墙钟秒，供 pk190_flipanalyze316.py 逐拍判读。

为什么不用 `ros2 topic echo`：echo 走 yaml.dump，默认 width=80 会把我们这条 ~200 字
节、中间带 ASCII 空格的状态行**折成多行**，正则就只吃到前半截（"位置="之后的字段全丢），
读出来是"行数太少"而不是"车没动"。订阅端自己写行就没这个坑。

用法（WSL）：python3 tools/pk190_statusdump316.py <秒> <输出文件>
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/pk190_trace.txt'


class Dump(Node):
    def __init__(self, fh):
        super().__init__('pk190_statusdump')
        self.fh = fh
        self.n = 0
        self.first = None
        # 发布端是 RELIABLE depth=10；这里 BEST_EFFORT + 深队列，掉几拍不阻塞发布端。
        self.create_subscription(String, '/parking/shuttle_status', self.cb,
                                 QoSProfile(depth=200,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, m):
        now = time.time()
        if self.first is None:
            self.first = now
        self.fh.write('%.3f %s\n' % (now, m.data))
        self.fh.flush()
        self.n += 1


def main():
    rclpy.init()
    t0 = time.time()
    with open(OUT, 'w', encoding='utf-8') as fh:
        node = Dump(fh)
        while time.time() - t0 < SECS:
            # 状态行 20 Hz；spin_once 一次只取一条，所以用 0.02 s 的小步长紧转，
            # 别用大 timeout —— 那会让回调攒在队列里被 KEEP_LAST 挤掉。
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        print('采到 %d 行 / %.1f s，首行 t=%.3f，写入 %s' % (
            node.n, time.time() - t0, node.first or 0.0, OUT))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
