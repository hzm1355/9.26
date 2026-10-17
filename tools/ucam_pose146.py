#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成一条 gz_msgs.Pose，让 Gazebo 的 user camera 站在给定位置看向给定的点。

为什么要有这个文件：Gazebo 11 的 `gz camera -c gzclient_camera -f <模型>` 在这套环境里是
**空操作**（对 car_1 和 car_6 各发一次，前后截图除状态栏外逐像素相同），CLI 的 follow 走
不通；而 `/gazebo/v2x_city_scene/user_camera/pose` 这个话题在 gz topic -l 里是存在的，
所以退一步自己算相机位姿发过去。

Gazebo 的相机看向自己的局部 +X，所以四元数就是 R_z(yaw)·R_y(pitch)：
    dir = look - cam 归一化，yaw = atan2(dy, dx)，pitch = asin(dz)
用法：ucam_pose146.py CX CY CZ LX LY LZ 输出文件
"""
import math
import sys


def main():
    cx, cy, cz, lx, ly, lz = [float(v) for v in sys.argv[1:7]]
    out = sys.argv[7]
    dx, dy, dz = lx - cx, ly - cy, lz - cz
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    if n < 1e-6:
        sys.stderr.write('相机点和目标点重合\n')
        return 2
    dx, dy, dz = dx / n, dy / n, dz / n
    yaw = math.atan2(dy, dx)
    pitch = math.asin(max(-1.0, min(1.0, dz)))
    # R = Rz(yaw) * Ry(pitch)，roll=0
    qy = (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))
    qz = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    # 四元数乘积 (x,y,z,w)
    ax, ay, az, aw = qz
    bx, by, bz, bw = qy
    qw = aw * bw - ax * bx - ay * by - az * bz
    qx = aw * bx + ax * bw + ay * bz - az * by
    qy_ = aw * by - ax * bz + ay * bw + az * bx
    qz_ = aw * bz + ax * by - ay * bx + az * bw
    txt = ('position { x: %.3f y: %.3f z: %.3f }\n'
           'orientation { x: %.6f y: %.6f z: %.6f w: %.6f }\n'
           % (cx, cy, cz, qx, qy_, qz_, qw))
    open(out, 'w').write(txt)
    print('相机 (%.2f,%.2f,%.2f) → 目标 (%.2f,%.2f,%.2f)｜yaw=%+.1f° pitch=%+.1f°'
          % (cx, cy, cz, lx, ly, lz, math.degrees(yaw), math.degrees(pitch)))
    print('  消息已写 %s：' % out)
    for line in txt.strip().split('\n'):
        print('    %s' % line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
