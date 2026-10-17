#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#131 —— 验证"摆位坐不住"的真正机制：插件在追**自己里程计估计的那个位姿**。

#127/#130 已经把范围收到这里：
  · 静默相 8 s 位移 0.002 m ⇒ 不是"保持最后一条指令"；
  · /car_6/cmd_vel 发布者计数 0、全场只有 diff_drive 一个订阅者 ⇒ 没有别的东西在发指令；
  · 连发零速却走 1.04 m / 车头被转 150°，且**摆到任意点都以类似幅度跑掉** ⇒
    它追的是一个"与摆位无关的目标"，而这个目标正好由轮关节位置承载。
diff_drive 的里程计是**用轮子转过的角度推出来的**：轮位 P 与插件目标 S 一旦脱节
（车顶杆/翘起空转/侧躺被拖着滚都会造成脱节），插件就会把车"开回它以为自己在的地方"。
所以本脚本的判据很干脆：

  把车 SetEntityState 到 /car_6/odom 报的那个位姿（x/y/yaw 取 odom，z 固定 6.720），
  连发零速 5 s：
    · 残差 <0.15 m ⇒ 证实"追的是 odom 位姿"。推论很硬：**摆位永远钉不住这台车**，
      想让它在哪儿停住只能靠指令把它开过去（或重启 gzserver 复位插件）；
    · 残差仍然 >0.3 m ⇒ 这条也被否定，剩下的唯一解释是接触挤出，回到 #106 查碰撞体。

护栏：odom 位姿必须在沙盘 footprint 内（x∈[−25,25]、y∈[−25.1,26.1]）才敢摆 ——
侧躺空转之后 odom 可能已经推出去几十米，把它摆到板外就又变成一次坠板翻车。
只发 cmd_vel 和摆位，不重启、不杀进程、不删实体。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import wrap, yaw_of                   # noqa: E402
from reseat122 import CAR, Hub, body                    # noqa: E402
import rclpy                                             # noqa: E402
from nav_msgs.msg import Odometry                        # noqa: E402

X_LO, X_HI = -25.0, 25.0
Y_LO, Y_HI = -25.1, 26.1
LAND_Z = 6.720


class OdomHub(Hub):
    def __init__(self):
        super().__init__()
        self.odom = None            # (x, y, yaw, 钟)
        self.n_odom = 0
        self.create_subscription(Odometry, '/%s/odom' % CAR, self.on_odom, 10)

    def on_odom(self, m):
        p = m.pose.pose
        q = p.orientation
        self.odom = (p.position.x, p.position.y, yaw_of(q), self.clock)
        self.n_odom += 1


def main():
    rclpy.init()
    h = OdomHub()
    if h.wait_pose() is None:
        print('!! 没拿到 /model_states')
        return 1
    end = time.time() + 6.0
    while h.odom is None and time.time() < end:
        h.tick()
        time.sleep(0.02)
    if h.odom is None:
        print('!! 6 s 没收到 /car_6/odom（插件没发里程计？publish_odom 被关了？）')
        return 1
    print('=== 现场 ===')
    c = body(h, '现场')
    print('  真值  (%8.3f, %8.3f) yaw=%+7.1f°  z=%.4f roll=%+.1f° pitch=%+.1f°' % (
        c[0], c[1], math.degrees(c[5]), c[2], c[6], c[7]))
    print('  odom  (%8.3f, %8.3f) yaw=%+7.1f°  钟=%9.2f（%.1f s 内 %d 条）' % (
        h.odom[0], h.odom[1], math.degrees(h.odom[2]), h.odom[3], 6.0, h.n_odom))
    dx, dy = h.odom[0] - c[0], h.odom[1] - c[1]
    dyaw = wrap(h.odom[2] - c[5])
    print('  里程计比真值多算出来的量：位移 %.3f m、转角 %+.1f°'
          '  ← 这就是轮位误差换算成位姿后的样子' % (
              math.hypot(dx, dy), math.degrees(dyaw)))

    ox, oy, oyaw = h.odom[0], h.odom[1], h.odom[2]
    if not (X_LO <= ox <= X_HI and Y_LO <= oy <= Y_HI):
        print('!! odom 位姿 (%.2f, %.2f) 已在沙盘 footprint 之外，不摆 —— '
              '先按指令把车开回来，或重启 gzserver 复位插件' % (ox, oy))
        return 1

    print('\n=== 1) 把车交到插件自认的位姿：SetEntityState(%.3f, %.3f, %.3f) yaw=%+.1f° ===' % (
        ox, oy, LAND_Z, math.degrees(oyaw)))
    if not h.place(ox, oy, LAND_Z, oyaw):
        print('!! SetEntityState 失败')
        return 1
    print('=== 2) 连发零速 6 s，逐秒看它是否还想走 ===')
    last = 0.0
    p0 = (ox, oy)
    end = time.time() + 6.0
    while time.time() < end:
        h.send(0.0, 0.0)
        h.tick()
        time.sleep(0.05)
        h.tick()
        if time.time() - last >= 1.0 and h.pose:
            last = time.time()
            cur = (h.pose[1], h.pose[2])
            print('    零速 t=%9.2f (%7.3f,%7.3f) z=%6.4f 离交点位 %.3f m' % (
                h.pose[0], cur[0], cur[1], h.pose[3],
                math.hypot(cur[0] - p0[0], cur[1] - p0[1])), flush=True)
    g = body(h, '交点后')
    d = math.hypot(g[0] - ox, g[1] - oy)
    dyaw2 = math.degrees(abs(wrap(g[5] - oyaw)))
    odo_after = h.odom
    print('\n=== 判读 ===')
    print('  交点残差 位移 %.3f m、车头 %.1f°；roll=%+.1f° pitch=%+.1f° z=%6.4f' % (
        d, dyaw2, g[6], g[7], g[2]))
    if odo_after:
        print('  交点后里程计与真值仍差 %.3f m / %+.1f°' % (
            math.hypot(odo_after[0] - g[0], odo_after[1] - g[1]),
            math.degrees(wrap(odo_after[2] - g[5]))))
    if d < 0.15 and dyaw2 < 8.0:
        print('  ⇒ 证实：插件追的就是 odom 位姿。**摆位钉不住这台车**，'
              '想让它停在哪只能把 odom 也带到哪（靠指令开过去，或重启插件复位）。')
        return 0
    if d > 0.30:
        print('  ⇒ 交到 odom 位姿它照走：这条也否定。剩下的解释只有接触挤出/楔住，'
              '回 tools/whats_under106.py 查落点底下的碰撞体。')
        return 1
    print('  ⇒ 中间地带（0.15~0.30 m）：机制成立但有二阶项（轮位差未完全清零/落地回弹），'
          '先按"靠指令开过去"设计，别指望摆位。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
