#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#127 —— 摆位之后分两相监测漂移，把"插件自持"和"接触挤出"分开。

为什么要分相（#126 实测）：把 car_6 摆到等待位 (12.00,−12.80) 四次全不坐住，落点沿车头
冲出 0.9~2.2 m、车头偏 56~77°，而且**停止发布之后又自己走了 3.6 m**。两种成因在日志里
长得一模一样，判法却完全不同：
  · 插件自持（#120 已证 cmd_vel_timeout 不生效，进程一退插件保持最后一条指令）
    → 只会在"停止发布"那一相里动，且方向沿车头、速度等于最后一条指令；
  · 接触挤出（等待位离入口杆轴只有 1.75 m，杆放平在 z 6.970~7.030，正夹在车身碰撞盒
    6.75~7.15 之间）→ 在"连发零速"那一相里就开始动，且会带着 yaw 一起翻。
所以每一相都逐秒记位姿，两相各自算"相对本相起点"的位移和末速。

用法：python3 tools/place_and_watch127.py [x,y,yaw度] [静默观察秒数]
默认摆到通道带净空地 (10.00,−9.55) 车头朝西 —— 离两道杆轴 y=−11.05 都是 1.5 m，
离龙门架立杆 y=−7.05 有 2.5 m，摆位瞬间不可能撞上任何 B 类设施。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from reseat122 import CAR, Hub, body   # noqa: E402
import rclpy                            # noqa: E402


def main():
    xy = (sys.argv[1] if len(sys.argv) > 1 else '10.00,-9.55,180')
    x, y, ydeg = [float(v) for v in xy.split(',')]
    yaw = math.radians(ydeg)
    sil = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    rclpy.init()
    h = Hub()
    if h.wait_pose() is None:
        print('!! 没拿到 /model_states')
        return 1
    print('=== 现场 ===')
    body(h, '现场')
    print('\n=== 1) 摆到 (%.2f, %.2f, 6.720) yaw=%+.1f°（净空地，四周 1.5 m 内无杆件）===' % (
        x, y, ydeg))
    if not h.place(x, y, 6.720, yaw):
        print('!! SetEntityState 失败')
        return 1
    print('\n=== 2) 连发零速 5 s，逐秒读（这一相动车 = 接触挤出）===')
    p0 = None
    h.send(0.0, 0.0)
    end = time.time() + 5.0
    last = 0.0
    while time.time() < end:
        h.send(0.0, 0.0)
        h.tick()
        time.sleep(0.05)
        h.tick()
        if time.time() - last >= 1.0 and h.pose:
            last = time.time()
            c = h.pose                      # (钟, x, y, z, yaw)
            if p0 is None:
                p0 = (c[1], c[2])
            print('    零速 t=%9.2f (%7.3f,%7.3f) z=%6.4f 相对本相起点 %.3f m' % (
                c[0], c[1], c[2], c[3], math.hypot(c[1] - p0[0], c[2] - p0[1])), flush=True)
    g = body(h, '零相末')
    d_brake = math.hypot(g[0] - x, g[1] - y)

    print('\n=== 3) 停止发布 %.0f s，逐秒读（这一相动车 = 插件自持最后一条指令）===' % sil)
    end = time.time() + sil
    last = 0.0
    q0 = (g[0], g[1])
    while time.time() < end:
        h.tick()
        time.sleep(0.2)
        if time.time() - last >= 1.0 and h.pose:
            last = time.time()
            c = h.pose
            print('    静默 t=%9.2f (%7.3f,%7.3f) z=%6.4f 相对静默起点 %.3f m' % (
                c[0], c[1], c[2], c[3], math.hypot(c[1] - q0[0], c[2] - q0[1])), flush=True)
            q0 = (c[1], c[2])
    g2 = body(h, '静默末')
    d_sil = math.hypot(g2[0] - g[0], g2[1] - g[1])

    print('\n=== 判读 ===')
    print('  连发零速 5 s 相：摆位点→本相末 %.3f m' % d_brake)
    print('  停止发布 %.0f s 相：本相末速上限 %.3f m/s、相内位移 %.3f m' % (
        sil, d_sil / sil, d_sil))
    a = []
    if d_brake > 0.30:
        a.append('零速相就在动 ⇒ 有实体在顶它（接触挤出/楔在设施上），不是指令问题')
    if d_sil > 0.30:
        a.append('静默相还在走 ⇒ 插件保持最后一条指令（cmd_vel_timeout 不生效），'
                 '或接触力仍在推')
    if not a:
        a.append('两相都不动 ⇒ 这一摆是干净的，摆位点四周净空足够')
    for s in a:
        print('  · %s' % s)
    print('  末态 roll=%+.1f° pitch=%+.1f° z=%6.4f yaw=%+.4f 位(%.3f,%.3f)' % (
        g2[6], g2[7], g2[2], g2[5], g2[0], g2[1]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
