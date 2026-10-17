#!/usr/bin/env python3
"""读一帧 /model_states，把指定模型的 x y 打到 stdout（一行，只有两个数）。

存在的理由是 stuck_look152.sh 要把"车此刻在哪"喂给 nearby_colliders.py。输出严格
只有一行两个数，别的诊断信息一律走 stderr，这样 `POS=$(... | tr -d '\\r')` 和
`set -- $POS` 不会被多行输出打乱。

话题是 /model_states 不是 /gazebo/model_states —— world 行 3046 的
libgazebo_ros_state 插件没有 remap，节点自报发布的就是 /model_states。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    rclpy.init()
    nd = Node('probe_xy')
    q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST)
    got = {}

    def on_states(m):
        for i, n in enumerate(m.name):
            if n == want:
                got['v'] = (m.pose[i].position.x, m.pose[i].position.y)

    nd.create_subscription(ModelStates, '/model_states', on_states, q)
    t0 = time.time()
    while time.time() - t0 < secs and 'v' not in got:
        rclpy.spin_once(nd, timeout_sec=0.2)
    rclpy.shutdown()
    if 'v' not in got:
        sys.stderr.write('FAIL %.1f s 内没收到 %s 的位姿\n' % (secs, want))
        return 1
    print('%.4f %.4f' % got['v'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
