#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 PlateCapture 里的车辆位姿投回卡口相机像素系，判断车牌到底落在画面哪里。

实机抓拍照里只见车顶不见蓝牌，光看图猜不出是"牌没换上"还是"相机没拍到"。
相机世界位姿是从 world 里 model pose + sensor pose 相加得到的（gate_j3_north：
模型 -7.0 8.0 6.7，传感器局部 0 -3.4 2.5 0 0.55 0 -> 相机 -7.0 4.6 9.2 俯角 0.55），
车牌相对底盘中心固定 0.81 m 在车尾，所以有车位姿 + 朝向就能算出牌心的像素位置。
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from v2x_msgs.msg import PlateCapture

CAM = {
    'gate_j3_north': dict(pos=np.array([-7.0, 4.6, 9.2]), pitch=0.55, yaw=0.0),
    'gate_j2_south': dict(pos=np.array([14.833, -14.6, 9.2]), pitch=0.55, yaw=np.pi),
}
W_IMG, H_IMG, FOV_H = 1920, 1080, 0.6
F_PX = (W_IMG / 2.0) / np.tan(FOV_H / 2.0)
PLATE_BEHIND = 0.81          # 车尾牌离底盘中心的距离（world 里 plate_rear 的 x）


def rot(pitch, yaw):
    """Gazebo 固定轴角 RPY(0, pitch, yaw)：机体系 -> 世界系，R = Rz(yaw) @ Ry(pitch)。
    俯仰是绕 **Y** 轴，不是绕 X——上一版写成 Rx 会把"上"分量算成 -2.00（真值 0），
    于是把明明落在画面正中的车牌投影成画面外。"""
    c, s = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry


def project(cam, world_pt):
    """世界点 -> 相机像素。相机系 x 光轴、y 向上、z 向左（光学系），
    这里直接用 Gazebo 机体旋转变量：R 把机体系旋到世界系，取逆即世界->机体，
    再按 (x_fwd, y_left, z_up) -> (u 右, v 下) 映射。"""
    R = rot(cam['pitch'], cam['yaw'])
    d = np.array(world_pt) - cam['pos']
    b = R.T @ d                              # 机体系：x 前、y 左、z 上
    xf, yl, zu = b
    if xf <= 0.05:
        return None, b
    u = F_PX * yl / xf * -1.0 + W_IMG / 2.0   # 相机看向 +x，画面右 = -y
    v = H_IMG / 2.0 - F_PX * zu / xf          # 画面向下 = -z
    return (u, v), b


def plate_center(msg):
    cam = CAM.get(msg.gate_id)
    if cam is None:
        return None, None, None
    q = msg.pose.orientation
    h = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    p = msg.pose.position
    rear_x = p.x - PLATE_BEHIND * np.cos(h)
    rear_y = p.y - PLATE_BEHIND * np.sin(h)
    return cam, (rear_x, rear_y, 6.85), h


class Wait(Node):
    def __init__(self):
        super().__init__('gate_geom_check')
        self.got = []
        self.create_subscription(PlateCapture, '/plate/capture', self.on, 50)
        print('等待一条 /plate/capture ...')

    def on(self, msg):
        self.got.append(msg)


def main():
    rclpy.init()
    node = Wait()
    t0 = time.time()
    while not node.got and time.time() - t0 < 200:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.got:
        print('!! 200 s 内没有任何抓拍')
        return 1
    for msg in node.got:
        cam, pt, h = plate_center(msg)
        if cam is None:
            print('%s 未知卡口 %s' % (msg.capture_id, msg.gate_id))
            continue
        px, body = project(cam, pt)
        print('%-20s 车 %s 朝向%.0f° -> 车尾牌世界 (%.2f,%.2f,6.85) 机体(前%.2f 左%.2f 上%.2f) -> 像素 %s'
              % (msg.capture_id, msg.vehicle_id, np.degrees(h),
                 pt[0], pt[1], body[0], body[1], body[2],
                 ('%.0f,%.0f' % px) if px else '画面外'))
        img = cv2.imread(msg.image_path)
        if img is None:
            continue
        if px:
            cv2.circle(img, (int(px[0]), int(px[1])), 8, (0, 0, 255), 2)
            cv2.rectangle(img, (int(px[0]) - 152, int(px[1]) - 41),
                          (int(px[0]) + 152, int(px[1]) + 41), (0, 255, 0), 2)
        out = '/mnt/d/desktop1/9.26.1/geom_%s.png' % msg.capture_id
        cv2.imwrite(out, img)
        print('   标注图 ->', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
