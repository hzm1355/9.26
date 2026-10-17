#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#87 —— 一次实验同时判三条机制，把"摆位钉不住 car_6"查到根上。

前两轮的教训：#127 只量位移，#131 把车交到 odom 报的位姿（那时无操作，零区分力）。
真正能分辨的是**轮子转没转**，以及**odom 跟不跟随摆位**，所以这一版三相同采：
gz 轮角（left/right_wheel_joint 的累计角）、/model_states 真值位姿、/car_6/odom。

落点选等待位 (12.00,−12.80)：车头保险杠在 y=−12.40，离入口杆轴 y=−11.05 还有 1.35 m，
四周没有任何 B 类设施碰撞体 —— 摆位瞬间不可能被顶，于是"接触挤出"这一支在这里天然被排除，
剩下的只有两种可能。三条判据：

  ① 轮角判纯滚动：相内 车体路径 ≈ R×(ΔL+ΔR)/2，且 车头变化 ≈ R×(ΔR−ΔL)/SEP。
     吻合 ⇒ diff_drive 在把轮关节驱向它自己累计的 target_position（摆位不重置这个目标，
     于是每次摆位都攒出一段轮位误差，车就"自己走回去"）—— **摆位永远钉不住，只能靠指令开过去**。
     不吻合（车体动而轮角几乎不变）⇒ 是被实体顶/纯滑移，回 tools/whats_under106.py 查碰撞体。
  ② odom 跟随性：摆位后第一次 odom 采样若仍指摆位**前**的位姿 ⇒ 里程计是轮积分出来的，
     插件手里根本没有"世界位姿"目标，#131 那句"追 odom 位姿"就地作废。
  ③ 静默相（不发布）位移：>0.3 m 且轮角同向增长 ⇒ 目标驱动与发布无关，坐实 ①。

只发 cmd_vel 和摆位，不重启、不杀进程、不删实体。
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import SEP, RADIUS, wrap          # noqa: E402
from pose_full121 import gz_read                    # noqa: E402
from reseat122 import CAR, Hub, PLACE               # noqa: E402
from reseat_to_odom131 import OdomHub               # noqa: E402
import rclpy                                        # noqa: E402


class Sample:
    """一次采齐三路证据；gz_read 是子进程（约 1 s），所以按 2 s 节奏采。"""

    def __init__(self, h, tag):
        g = gz_read(CAR)
        while h.pose is None:
            h.tick()
            time.sleep(0.02)
        self.tag = tag
        self.x, self.y, self.z = g[0], g[1], g[2]
        self.yaw = g[5]
        self.roll, self.pitch = g[6], g[7]
        self.l, self.r = g[8], g[9]
        self.od = h.odom                      # (x, y, yaw, 钟) 或 None
        self.pose = h.pose                    # (钟, x, y, z, yaw) 来自 /model_states
        print('  %-8s 位(%7.3f,%7.3f) z=%6.4f yaw=%+7.1f° roll=%+5.1f° pitch=%+5.1f° '
              '轮角 左%9.3f 右%9.3f | odom(%8.3f,%8.3f)%s' % (
                  tag, self.x, self.y, self.z, math.degrees(self.yaw), self.roll,
                  self.pitch, self.l, self.r,
                  self.od[0] if self.od else 0.0, self.od[1] if self.od else 0.0,
                  '' if self.od else ' 无'), flush=True)


def phase(h, tag, secs, send_zero, step=2.0):
    """返回该相的采样列表；send_zero=True 时全程连发零速，否则一条都不发。"""
    print('\n=== %s（%.0f s，%s）===' % (
        tag, secs, '连发零速' if send_zero else '完全不发布'))
    out = [Sample(h, tag + '起')]
    end = time.time() + secs
    last = 0.0
    while time.time() < end:
        if send_zero:
            h.send(0.0, 0.0)
        h.tick()
        time.sleep(1.0 / 20.0)
        h.tick()
        if time.time() - last >= step:
            last = time.time()
            out.append(Sample(h, tag + '%.0f' % (time.time() - (end - secs))))
    out.append(Sample(h, tag + '末'))
    return out


def roll_consistency(a, b, name):
    """把两相采样的首末差换算成"轮子该滚出多少"，与实测车体路径/车头对比。"""
    dl, dr = b.l - a.l, b.r - a.r
    dpath = math.hypot(b.x - a.x, b.y - a.y)
    dyaw = wrap(b.yaw - a.yaw)
    p_path = RADIUS * (dl + dr) / 2.0
    p_yaw = RADIUS * (dr - dl) / SEP
    print('  %s：车体位移 %.3f m / 车头 %+6.1f°；轮角增量 左%+7.3f 右%+7.3f rad'
          ' → 纯滚动应走 %+.3f m、转 %+6.1f°' % (
              name, dpath, math.degrees(dyaw), dl, dr, p_path, math.degrees(p_yaw)))
    err_path = abs(dpath - abs(p_path))
    err_yaw = abs(math.degrees(wrap(dyaw - p_yaw)))
    verdict = '吻合（在滚）' if err_path < max(0.25, 0.35 * dpath) else '不吻合（顶/滑）'
    print('        路径残差 %.3f m、车头残差 %.1f° ⇒ %s' % (err_path, err_yaw, verdict))
    return dpath, err_path, err_yaw, dl, dr, verdict


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
        print('!! 6 s 没收到 /car_6/odom —— 插件没发里程计，判据 ② 无从谈起')
        return 1

    print('=== 0) 摆位前基线：连发零速 4 s，轮角必须已经静止 ===')
    base = phase(h, '基线', 4.0, True)
    d0 = roll_consistency(base[0], base[-1], '基线（静止检验）')
    if abs(d0[3]) > 0.05 or abs(d0[4]) > 0.05:
        print('  !! 摆位前轮角仍在变（左%+.3f 右%+.3f）⇒ 上一段轮位误差还没跑完，'
              '这轮的对照不干净；再刹 6 s' % (d0[3], d0[4]))
        base = phase(h, '基线2', 6.0, True)
        d0 = roll_consistency(base[0], base[-1], '基线2（静止检验）')
        if abs(d0[3]) > 0.05 or abs(d0[4]) > 0.05:
            print('  !! 仍然在转 ⇒ 这台车的插件目标一直追不上（轮子被打滑/被顶），'
                  '摆位实验做不了，先靠指令把车开走')
            return 1

    pre = base[-1]
    print('\n=== 1) SetEntityState 到等待位 (%.3f, %.3f, %.3f) yaw=%+.1f° ===' % (
        PLACE[0], PLACE[1], PLACE[2], math.degrees(PLACE[3])))
    if not h.place(*PLACE):
        print('!! SetEntityState 失败，实验中止')
        return 1

    print('\n=== 2) 摆位后连发零速 8 s ===')
    brake = phase(h, '零速', 8.0, True)
    d1 = roll_consistency(brake[0], brake[-1], '零速相')

    print('\n=== 3) 停止发布 8 s ===')
    silent = phase(h, '静默', 8.0, False)
    d2 = roll_consistency(silent[0], silent[-1], '静默相')

    print('\n=== 判读 ===')
    # ② odom 跟随性：摆位后第一次采样，odom 指的是摆位前还是摆位后？
    o = brake[0].od
    if o:
        d_pre = math.hypot(o[0] - pre.x, o[1] - pre.y)
        d_p1 = math.hypot(o[0] - PLACE[0], o[1] - PLACE[1])
        print('  ② 摆位后 odom 离「摆位前真值」%.3f m、离「摆位目标」%.3f m ⇒ %s' % (
            d_pre, d_p1,
            'odom 不跟随摆位（纯轮积分）——插件手里没有世界位姿目标，'
            '"追 odom 位姿"的说法作废' if d_pre < d_p1 - 0.2 else
            ('odom 跟随了摆位（插件被世界位姿同步过）' if d_p1 < d_pre - 0.2
             else 'odom 与两者都接近，无区分力（车本来就几乎没动）')))
    else:
        print('  ② 摆位后没拿到 odom 样本')

    moved = max(d1[0], d2[0])
    if moved < 0.30:
        print('  ⇒ 这一摆坐住了：两相位移都 <0.3 m。之前的"钉不住"是**落点楔在设施上**'
              '（等待位离杆轴 1.75 m 但车头朝外时会顶到别的东西），不是插件自走。')
    elif (d1[5].startswith('吻合') or d2[5].startswith('吻合')):
        print('  ⇒ ① 成立：车体走的路正是轮角增量换算出来的路 ⇒ diff_drive 在追自己累计的'
              '轮位目标，SetEntityState 不重置它。**摆位钉不住这台车，'
              '想让它停在哪只能靠 cmd_vel 开过去（或重启 gzserver 复位插件）。**')
    else:
        print('  ⇒ ① 否定、且落点四周无碰撞体：车体动而轮子不滚 ⇒ 只剩"被实体顶"这一条，'
              '回 tools/whats_under106.py 把落点 2.5 m 内的碰撞体全列出来')
    print('  末态 位(%.3f,%.3f) z=%.4f roll=%+.1f° pitch=%+.1f° yaw=%+.1f°' % (
        silent[-1].x, silent[-1].y, silent[-1].z, silent[-1].roll, silent[-1].pitch,
        math.degrees(silent[-1].yaw)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
