#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#126/#128 —— 把 car_6 摆回进场等待位，并证明它"真的坐住了"。

#127 实测把这条根因钉死了（也是 #84"发零速也自走"、#85"纯ω失效"、#86"被顶走 6.1 m"、
本轮 12.9 m 侧翻的共同解释）：
  · 摆到净空地 (10.00,−9.55) 车头朝西，**连发零速** 5 s 内车自己走了 10.6 m、
    翻成 roll=−180° 顶朝天；
  · 摆到等待位 (12.00,−12.80) 之后零速相走出 1.04 m、车头被转掉 146°；
  · 但**停止发布之后 8 s 位移 0.002 m** —— 所以既不是别的车在顶，也不是接触挤出，
    更不是"插件保持最后一条指令"：是 **diff_drive 把轮关节位置当目标在追**。
      摆位（SetEntityState）只挪动车体、不动轮关节，于是车体与轮位的关系被重置，
      插件立刻用"轮子真转"把这份误差补回来 —— 表现出来就是"发了零速车还自己走"。
      误差被追平之后它就真的停了（静默相 0.002 m 就是证据）。

所以正确做法不是"摆一次读一次"，而是**迭代收敛**：摆到位 → 连发零速等它把误差追平
（位置连续 1.5 s 不再变）→ 量残差 → 再摆一次。每轮残差应当显著缩小；缩小不成立
（顶到设施/姿态坏了）就报 FAIL 停手，别把 shuttle 拉起来顶着同一根杆。

就位判据五条同时成立：|roll|<15°、|pitch|<15°、6.5<z<6.9（侧躺 z=7.100=板顶 6.700+
半宽 0.400，顶朝天 z=7.150）、离目标 <0.35 m、|wrap(yaw−目标)|<12°。

用法：python3 tools/seat_guard126.py [x,y,z,yaw度]（默认进场等待位）
只发 cmd_vel 和摆位，不重启、不杀进程、不删实体。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import wrap                        # noqa: E402
from reseat122 import CAR, Hub, body                 # noqa: E402
import rclpy                                          # noqa: E402

DEF_TARGET = '12.00,-12.80,6.720,90'
MAX_TRIES = 6
CONV_S = 1.5          # 连续这么久不动算追平
CONV_M = 0.02
CONV_MAX = 10.0       # 单轮最多等这么多秒


def wait_converged(h):
    """连发零速直到位置连续 CONV_S 不再变化（或超时）。返回 (耗时, 末位姿)。"""
    t0 = time.time()
    ref = None
    ref_at = None
    while time.time() - t0 < CONV_MAX:
        h.send(0.0, 0.0)
        h.tick()
        time.sleep(0.05)
        h.tick()
        if not h.pose:
            continue
        cur = (h.pose[1], h.pose[2])
        if ref is None or math.hypot(cur[0] - ref[0], cur[1] - ref[1]) > CONV_M:
            ref, ref_at = cur, time.time()
        elif time.time() - ref_at >= CONV_S:
            return time.time() - t0, cur
    return time.time() - t0, ref


def verdict(c, tgt):
    d = math.hypot(c[0] - tgt[0], c[1] - tgt[1])
    dy = abs(wrap(c[5] - tgt[3]))
    bad = []
    if abs(c[6]) >= 15.0:
        bad.append('roll=%+.1f°' % c[6])
    if abs(c[7]) >= 15.0:
        bad.append('pitch=%+.1f°' % c[7])
    if not 6.5 < c[2] < 6.9:
        bad.append('z=%.4f' % c[2])
    if d >= 0.35:
        bad.append('离目标 %.3f m' % d)
    if math.degrees(dy) >= 12.0:
        bad.append('车头偏 %.1f°' % math.degrees(dy))
    return d, math.degrees(dy), bad


def main():
    f = [float(v) for v in (sys.argv[1] if len(sys.argv) > 1 else DEF_TARGET).split(',')]
    tgt = (f[0], f[1], f[2], math.radians(f[3]))
    rclpy.init()
    h = Hub()
    if h.wait_pose() is None:
        print('!! 没拿到 /model_states（gzserver 没起？）')
        return 1
    print('目标位 (%.2f, %.2f, %.3f) yaw=%+.1f°' % (
        tgt[0], tgt[1], tgt[2], math.degrees(tgt[3])))
    print('=== 现场 ===')
    c = body(h, '现场')
    d, dy, bad = verdict(c, tgt)
    print('  roll=%+.1f° pitch=%+.1f° z=%6.4f 位(%.3f,%.3f) yaw=%+.1f° -> %s' % (
        c[6], c[7], c[2], c[0], c[1], math.degrees(c[5]),
        '就位' if not bad else '不合格：%s' % '、'.join(bad)))

    prev_d = None
    for k in range(1, MAX_TRIES + 1):
        print('\n=== 第 %d 轮：摆位 → 连发零速等轮位误差追平 ===' % k, flush=True)
        if not h.place(*tgt):
            print('  !! SetEntityState 失败（服务没起？实体名不对？）')
            return 1
        spent, _ = wait_converged(h)
        c = body(h, '体检%d' % k)
        d, dy, bad = verdict(c, tgt)
        print('  追平耗时 %.1f s；roll=%+.1f° pitch=%+.1f° z=%6.4f 残差 %.3f m 车头偏 %.1f° -> %s' % (
            spent, c[6], c[7], c[2], d, dy, '就位' if not bad else '不合格：%s' % '、'.join(bad)),
            flush=True)
        if not bad:
            print('\nOK：car_6 已坐住 (%.3f, %.3f) yaw=%+.1f°，残差 %.3f m' % (
                c[0], c[1], math.degrees(c[5]), d))
            return 0
        if prev_d is not None and d > prev_d * 0.8:
            print('!! 残差没有继续缩小（%.3f → %.3f m）：多半顶在设施上或姿态已坏，'
                  '停手；继续摆只会把同一份轮位误差换成新的自走。' % (prev_d, d))
            return 1
        prev_d = d
    print('!! %d 轮都没坐住，停在这里，别拉 shuttle（它会顶着同一根杆再翻一次）' % MAX_TRIES)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
