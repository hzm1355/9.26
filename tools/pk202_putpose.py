#!/usr/bin/env python3
# 把一台车按**完整 rpy**钉到指定格位（place_cars.py 只会给 yaw，翻车复位用不了）。
#
# 为什么单独写一片而不是给 place_cars.py 加开关：place_cars 的语义是"按 yaml 的
# start_idx 把车流车摆回路线起点"，它显式跳过 drive:false 的 car_3（那段注释就是
# 本轮踩过的坑——照摆会把车从停车场瞬移回环路）。这里要的正好是另一半：把停车场
# 那台**翻倒 + 掉出台面**的车扶正放回它的等待位，摆完还得站得住。
#
# 三条实测来的规矩（沿用 place_cars.py 的口径，不重新发明）：
#   1) 摆之前先按 ~20 Hz 灌 1.5 s 零速：轮子还受驱时摆位，差速控制器攒下的轮位误差
#      会一次性释放，实测沿车头方向冲到 4~8.5 m/s 并飞出路网（tools/place_eject_probe.py）。
#   2) 摆之后再压 1.0 s 零速，同一落点实测 6 s 内 mv 恒 0.00。
#   3) z 用 6.70 不用 world 里写的 7.00：按 7.00 摆等于每次摆完自由落体 0.3 m，
#      落地瞬间 2.4 m/s 的垂直速度会被卡口当成"通过速度"记进抓拍账。
# 用法：python3 tools/pk202_putpose.py 名字 x y z roll pitch yaw
# 负数直接当位置参数传（不走 argparse，免得撞上"负数带逗号被当选项"那一坑）。
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node

BRAKE_S = 1.5
QUIET_S = 1.0


def rpy_to_quat(roll, pitch, yaw):
    """与 tf.transformations.euler_quaternion 同式（内旋 Z-Y-X，单位弧度）"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


class Putter(Node):

    def __init__(self, name):
        super().__init__('pk202_putpose')
        self.name = name
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_entity_state 未就绪，继续等待')
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % name, 10)

    def hold(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def put(self, x, y, z, roll, pitch, yaw):
        qw, qx, qy, qz = rpy_to_quat(roll, pitch, yaw)
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.name
        p = req.state.pose
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w, p.orientation.x = qw, qx
        p.orientation.y, p.orientation.z = qy, qz
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = req.state.twist.linear.y = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        ok = bool(res and res.success)
        print('  %s -> (%.2f,%.2f,%.2f) rpy=(%.3f,%.3f,%.3f)  %s%s' % (
            self.name, x, y, z, roll, pitch, yaw, 'OK' if ok else '失败',
            '' if ok else '：%s' % (res.status_message if res else 'srv 无回执')))
        return ok


def main():
    if len(sys.argv) != 8:
        print('用法：pk202_putpose.py 名字 x y z roll pitch yaw')
        return 2
    name = sys.argv[1]
    vals = [float(v) for v in sys.argv[2:]]
    rclpy.init()
    node = Putter(name)
    node.hold(BRAKE_S)
    ok = node.put(*vals)
    node.hold(QUIET_S)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
