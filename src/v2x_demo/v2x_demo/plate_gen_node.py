#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三（三）虚拟车牌生成节点。

职责：把"车牌号码"这个业务输入变成 Gazebo 能加载的纹理资源，并通知 v2x_plate 插件换图。

  输入  /plate/request   String  "car_1|rear|沪A12345"          （"-" 表示还原占位牌）
  输入  /plate/set_plate v2x_msgs/SetPlate                      同一入口的服务形态
  输出  /plate/set       String  "car_1|rear|p00007"             给贴牌插件
  输出  /plate/number    String  "car_1|rear|p00007|沪A12345"    识别率统计用的真值

每张车牌落到独立目录 /tmp/v2x_plates/<seq>/{tex.png,<seq>.material}。
必须独立目录：OGRE 只在目录首次注册时扫描文件，同目录新增 .material 会静默失效。
"""
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from plate_layout import render_plate
from v2x_msgs.srv import SetPlate


class PlateGenNode(Node):
    def __init__(self):
        super().__init__('plate_gen_node')
        self.declare_parameter('plate_dir', '/tmp/v2x_plates')
        # lit:=true 让牌面走"受光"材质（lighting on），用于条目 3 的对照实验：
        # 默认 lighting off 是把牌当爆闪补光下的反光牌，亮度与场景光照解耦，
        # 逆光/弱光/夜间三档的识别率因此量不出差异。要看光照对识别的影响，
        # 就得有一组"牌面跟着场景一起暗下去"的车，这就是那个开关。
        self.declare_parameter('lit', False)
        self.lit = bool(self.get_parameter('lit').value)
        self.dir = self.get_parameter('plate_dir').value
        os.makedirs(self.dir, exist_ok=True)
        self.lock = threading.Lock()
        # 序号必须跨重启单调递增：OGRE 按材质名缓存纹理，重名会命中上一次的旧图，
        # 换牌指令"成功"、车上却还是上一张牌，是最难查的一种假阳性。
        self.seq = max([int(n[1:]) for n in os.listdir(self.dir)
                        if n[:1] == 'p' and n[1:].isdigit()] or [0])

        self.pub_set = self.create_publisher(String, '/plate/set', 10)
        self.pub_num = self.create_publisher(String, '/plate/number', 10)
        self.create_subscription(String, '/plate/request', self.on_request, 10)
        self.create_service(SetPlate, '/plate/set_plate', self.on_set_plate)
        self.get_logger().info('[plate_gen] 就绪，纹理目录 %s，牌面材质 lighting %s'
                              % (self.dir, 'on（受光对照）' if self.lit else 'off（自发光）'))

    def on_request(self, msg):
        parts = msg.data.split('|')
        if len(parts) != 3:
            self.get_logger().warn('载荷应为 模型|位置|车牌号，收到: %s' % msg.data)
            return
        self.apply(parts[0], parts[1], parts[2])

    def on_set_plate(self, req, resp):
        seq = self.apply(req.vehicle_id, req.slot, req.number)
        resp.success = seq is not None
        resp.seq = seq or ''
        resp.message = ('已生成 %s（牌面 lighting %s）' % (seq, 'on' if self.lit else 'off')
                        if seq else 'vehicle_id/slot/number 不能为空')
        return resp

    def apply(self, model, slot, number):
        """渲染纹理 + 写材质 + 通知插件。返回序号，"-" 表示还原占位牌，None 表示入参不合法。"""
        if not model or not slot:
            return None
        if number == '-' or not number:
            seq = '-'
        else:
            with self.lock:
                self.seq += 1
                seq = 'p%05d' % self.seq
            d = os.path.join(self.dir, seq)
            os.makedirs(d, exist_ok=True)          # 关键：每张牌一个新目录
            render_plate(number, os.path.join(d, 'tex.png'))
            # lighting off + diffuse 纯白：等效现实卡口的爆闪/频闪补光。
            # 实测车尾牌正对的那一面完全背阴，lighting on 时蓝牌被压成棕红色，
            # HSV 定位（色相 100~130）直接找不到牌；补光后牌面亮度与场景光照解耦，
            # #33 的逆光/弱光/夜间预设才不会把识别率打成零。
            with open(os.path.join(d, seq + '.material'), 'w') as f:
                f.write(
                    'material v2x/%s\n{\n  technique\n  {\n    pass\n    {\n'
                    '      lighting %s\n      diffuse 1 1 1 1\n      texture_unit\n      {\n'
                    '        texture %s/tex.png\n      }\n    }\n  }\n}\n'
                    % (seq, 'on' if self.lit else 'off', d))

        self.pub_set.publish(String(data='%s|%s|%s' % (model, slot, seq)))
        self.pub_num.publish(String(data='%s|%s|%s|%s' % (model, slot, seq, number)))
        self.get_logger().info('[plate_gen] %s %s -> %s (%s)' % (model, slot, seq, number))
        return seq


def main():
    rclpy.init()
    node = PlateGenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
