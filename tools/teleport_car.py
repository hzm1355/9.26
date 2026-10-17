#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把车辆瞬移到卡口相机正前方并摆好朝向，用于确定性地验证抓拍画面。
走 /set_model_state 服务（place_cars.py 用的就是它），不依赖 gz CLI。
用法：python3 teleport_car.py <模型名> <x> <y> <z> <yaw弧度>"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node


def main():
    name, x, y, z, yaw = sys.argv[1], *[float(v) for v in sys.argv[2:6]]
    rclpy.init()
    node = Node('teleport_car')
    cli = node.create_client(SetEntityState, '/set_entity_state')
    t0 = time.time()
    while not cli.wait_for_service(timeout_sec=1.0) and time.time() - t0 < 15:
        pass
    if not cli.service_is_ready():
        print('!! /set_entity_state 服务不可用')
        return

    st = EntityState()
    st.name = name
    st.pose = Pose()
    st.pose.position.x, st.pose.position.y, st.pose.position.z = x, y, z
    st.pose.orientation.z = math.sin(yaw / 2.0)
    st.pose.orientation.w = math.cos(yaw / 2.0)
    st.reference_frame = 'world'

    req = SetEntityState.Request()
    req.state = st
    future = cli.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not future.done() and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if future.done():
        print('%s -> (%.2f, %.2f, %.2f) yaw=%.0f° 已提交' % (
            name, x, y, z, math.degrees(yaw)))
    else:
        print('!! 瞬移超时')


if __name__ == '__main__':
    main()
