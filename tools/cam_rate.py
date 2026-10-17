#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量卡口相机的真实出帧间隔与"渲染时刻 vs 仿真时刻"的滞后。

为什么要单独量这个：v2x_gate 的抓拍取的是 `CameraSensor::ImageData()`，也就是
**渲染线程最近写完的那一帧**，而位姿是抓拍那一刻现读的。两者相差一个渲染周期，
画面里的车就比记录位姿落后 `v × τ`。0.7 m/s 时 τ 多大都无所谓（0.1 s 只差 7 cm，
已经被 focus 的 1.81 m 提前量吃掉），2.8 m/s 时同一个 τ 就是 0.8 m——
高速档牌面掉出画面底沿、识别返回空号，根子在这个数上，不在识别器。

用法：python3 cam_rate.py [秒数]        默认 15 s
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

TOPICS = ['/gate_j3_north/gate_cam/image_raw', '/gate_j2_south/gate_cam/image_raw',
          # 停车场两道闸的相机（任务五）。命名规则是 <namespace>/<camera_name>/image_raw，
          # camera_name 就是 world 里那个 sensor 的名字（gate_in / gate_out）。
          # 早先 yaml 把 camera.gate_id 写成 parking_gate_in、namespace 又写成
          # /parking/gate_in，两个片段拼进了同一个话题名（实测多出一层
          # /parking/gate_in/parking_gate_in/image_raw）；现在 camera.gate_id 与闸机 id
          # 合成同一个值，命名空间只剩 /parking，由 check_parking_layout.py 锁死。
          '/parking/gate_in/image_raw',
          '/parking/gate_out/image_raw']


class Rate(Node):
    def __init__(self, secs):
        # use_sim_time 必须在构造时给：帧戳是仿真时间，节点钟也必须是仿真时间，
        # 否则"滞后"量出来是 1791352xxx s 这种墙上时间差。
        super().__init__('cam_rate',
                         parameter_overrides=[Parameter('use_sim_time',
                                                        Parameter.Type.BOOL, True)])
        self.secs = secs
        self.last = {}
        self.gap = {t: [] for t in TOPICS}
        self.skew = {t: [] for t in TOPICS}
        # 与 cam_probe 一致用默认可靠 QoS：gazebo_ros_camera 的发端是 RELIABLE，
        # 这里换成 BEST_EFFORT 实测一帧都收不到（fastrtps 的这条组合不通）。
        qos = QoSProfile(depth=10)
        for t in TOPICS:
            self.create_subscription(Image, t, lambda m, k=t: self.on_img(k, m), qos)

    def on_img(self, key, m):
        now = time.time()
        if key in self.last:
            self.gap[key].append(now - self.last[key])
        self.last[key] = now
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        sim = self.get_clock().now().nanoseconds * 1e-9
        if stamp > 0:
            self.skew[key].append(sim - stamp)

    def run(self):
        t0 = time.time()
        while time.time() - t0 < self.secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        for t in TOPICS:
            g, s = self.gap[t], self.skew[t]
            if not g:
                print('%-40s 没收到帧' % t)
                continue
            g2 = sorted(g)
            print('%-46s %3d 帧  间隔中位 %.3f s  90分位 %.3f s  最大 %.3f s  '
                  '帧戳落后仿真钟中位 %.3f s' % (
                      t, len(g) + 1, g2[len(g2) // 2], g2[int(len(g2) * 0.9)], g2[-1],
                      sorted(s)[len(s) // 2] if s else float('nan')))
        return 0


def main():
    rclpy.init()
    node = Rate(int(sys.argv[1]) if len(sys.argv) > 1 else 15)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
