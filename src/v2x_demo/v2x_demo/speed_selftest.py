#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测速路段节点合成数据自测（不需要 Gazebo，也不依赖 gzserver 的 /clock）：

  - 每格先喂位姿、再喂 10 Hz 仿真时钟，让车沿车道匀速走，断言**定点瞬时**读数就是
    v*3.6 —— 第一版把速度读成 0.25 km/h（缩 10 倍）就是这一条；
  - 同一车速分别过阈值 2.0 与 4.0 的点位，断言一边告警一边干净（否则"阈值在起作用"
    只是无条件报警的假象）；
  - 断言**区间平均** = 区间长度/行程时间，且反向通过不计、中途瞬移只作废本次通行
    并在复位点重建断面状态；
  - 逐个调 /speed/zones 的六种 op，断言异常只会回 success=False：服务回调里的异常
    会顺着 spin 把整个进程带走（实测常量名写错成 `SetSpeedZone.OP_LIST`，这些常量
    挂在 Request 上，节点就跑完一轮热配置就没气了）。

不 spin 是前提：真在跑的 gzserver 会以 97 Hz 推 /model_states 进来，把合成车队的数据冲花。
用法（WSL，source install/setup.bash 后）：
  python3 src/v2x_demo/v2x_demo/speed_selftest.py
退出码 0=PASS 1=FAIL
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rosgraph_msgs.msg import Clock
from v2x_msgs.srv import SetSpeedZone

import speed_detect_node as sdn

V = 0.7                  # m/s，与 signal_drive 的巡航速度一致
DT = 0.1                 # 仿真时钟一格
KM = V * sdn.MS2KMH      # 期望读数 2.52 km/h
CAR = 'car_selftest'
DECOY = 'building_1'
PLATE = '测A00001'

FAILED = []


def check(cond, what, detail=''):
    if cond:
        print('  ok   %s' % what)
    else:
        FAILED.append(what)
        print('  FAIL %s  %s' % (what, detail))


class Sink:
    """替掉 ROS 发布器：自测不跑 spin，直接收节点在回调里 publish 出来的消息。"""

    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)

    def last(self):
        return self.msgs[-1] if self.msgs else None


def pose_at(x, y):
    return Pose(position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(w=1.0))


def states(x, y):
    m = ModelStates()
    m.name = [DECOY, CAR]        # 非车模型混在里面，验证前缀过滤
    m.pose = [pose_at(0.0, 0.0), pose_at(x, y)]
    m.twist = [Twist(), Twist(linear=Vector3(x=V))]
    return m


def clock(t):
    c = Clock()
    c.clock.sec = int(t)
    c.clock.nanosec = int(round((t - int(t)) * 1e9))
    return c


def drive(node, x, y, ux, uy, ticks, t):
    """沿单位向量 (ux,uy) 以 V 匀速走 ticks 格，返回 (新时刻, 新位置)。"""
    step = V * DT
    for i in range(ticks):
        node.on_states(states(x + ux * step * i, y + uy * step * i))
        node.on_clock(clock(t + DT * i))
    return t + DT * ticks, x + ux * step * ticks, y + uy * step * ticks


def reset(node):
    """换场景就清一次：在途状态与统计都归零，后面每条断言只看本段自己产生的事件。"""
    node.enc.clear()
    node.open_seg.clear()
    node.inside.clear()
    node.hist.clear()
    node.latest.clear()
    node.samples.clear()
    node.total.clear()
    node.seeded = False


def main():
    rclpy.init()
    node = sdn.SpeedDetectNode()
    node.noise = 0.0                     # 关掉噪声，雷达读数才能逐位断言
    node.plates[CAR] = PLATE
    cap, seg, al, st = Sink(), Sink(), Sink(), Sink()
    node.pub_cap, node.pub_seg = cap, seg
    node.pub_alert, node.pub_stats = al, st

    z_inst = node.find('SZ-B-south-instant')
    z_easy = node.find('SZ-B-north-instant')
    z_seg = node.find('SZ-A-south-segment')
    if not (z_inst and z_easy and z_seg):
        print('安装的 speed_zones.yaml 里缺自测要用的点位，无法定位')
        return 1
    print('自测点位：%s(限%.1f) / %s(限%.1f) / %s(限%.1f) 期望读数 %.2f km/h' % (
        z_inst.id, z_inst.limit, z_easy.id, z_easy.limit, z_seg.id, z_seg.limit, KM))

    t = node.get_clock().now().nanoseconds / 1e9    # 统计窗口按节点时钟裁剪

    # ---------------------------------------------------------------- 播种
    reset(node)
    x0 = z_inst.center[0] + 4.0
    node.on_states(states(x0, z_inst.center[1]))
    node.on_clock(clock(t))
    check(node.seeded and not cap.msgs, '首帧只播种不产事件', '%d 条' % len(cap.msgs))

    # ------------------------------------------------- 条目2/3：定点瞬时 + 阈值告警
    t, _, _ = drive(node, x0, z_inst.center[1], -1.0, 0.0, 120, t)
    c = cap.last()
    check(len(cap.msgs) == 1, '一次通过出一个瞬时抓拍', '%d 条' % len(cap.msgs))
    if c:
        check(abs(c.speed_pose_kmh - KM) < 0.05, '位姿通道 = v*3.6（不缩 10 倍）',
              '读数 %.3f 期望 %.3f' % (c.speed_pose_kmh, KM))
        check(abs(c.speed_kmh - KM) < 0.05, '无噪声时雷达通道与位姿通道一致',
              '雷达 %.3f' % c.speed_kmh)
        check(c.beam_angle_deg < 2.0, '沿波束轴通行时夹角≈0', '%.2f°' % c.beam_angle_deg)
        check(c.vehicle_id == CAR and c.plate_number == PLATE,
              '抓拍带车辆编号与车牌', '%s/%s' % (c.vehicle_id, c.plate_number))
        check(abs(c.pose.position.x - z_inst.center[0]) < z_inst.radius,
              '抓拍位置落在断面内', 'x=%.2f' % c.pose.position.x)
        check(c.over and c.over_kmh > 0 and len(al.msgs) == 1,
              '低阈值点位告警', 'over=%s 告警%d条' % (c.over, len(al.msgs)))

    # ------------------------------------------------------ 阈值对照组（同一车速）
    reset(node)
    cap.msgs = []
    al.msgs = []
    t, _, _ = drive(node, z_easy.center[0] - 4.0, z_easy.center[1], 1.0, 0.0, 120, t)
    c = cap.last()
    check(c is not None and abs(c.speed_kmh - KM) < 0.05,
          '对照组点位读数同为 %.2f km/h' % KM,
          '%s' % getattr(c, 'speed_kmh', None))
    check(c is not None and not c.over and not al.msgs,
          '高阈值点位不告警（阈值真的在起作用）',
          'over=%s 告警%d条' % (getattr(c, 'over', None), len(al.msgs)))

    # ---------------------------------------------------------- 条目2：区间平均
    reset(node)
    seg.msgs = []
    al.msgs = []
    sx0 = z_seg.entry[0] + 3.0
    t, _, _ = drive(node, sx0, z_seg.entry[1], -1.0, 0.0, 300, t)
    s = seg.last()
    check(len(seg.msgs) == 1, '顺向通过区间出一条均速', '%d 条' % len(seg.msgs))
    if s:
        check(abs(s.length_m - z_seg.length) < 0.1, '区间长度取两断面间距',
              '%.2f m' % s.length_m)
        check(abs(s.travel_time_s - z_seg.length / V) < 0.2, '行程时间 = 长度/速度',
              '%.2f s 期望 %.2f' % (s.travel_time_s, z_seg.length / V))
        check(abs(s.avg_kmh - KM) < 0.1, '区间均速 = 长度/行程时间',
              '%.3f 期望 %.3f' % (s.avg_kmh, KM))
        check(s.over and len(al.msgs) == 1, '区间均速超阈值出告警',
              'over=%s 告警%d条' % (s.over, len(al.msgs)))

    # ------------------------------------------------------------ 反向通过剔除
    reset(node)
    seg.msgs = []
    t, _, _ = drive(node, z_seg.exit[0] - 3.0, z_seg.exit[1], 1.0, 0.0, 300, t)
    check(not seg.msgs, '反向通过区间不计（先见到出口断面）', '%d 条' % len(seg.msgs))
    check(len(node.open_seg) == 1, '反向那次只留下一个在途区间',
          '%d 个在途' % len(node.open_seg))

    # ---------------------------------------------------------- 中途复位/瞬移
    reset(node)
    seg.msgs = []
    t, x, y = drive(node, sx0, z_seg.entry[1], -1.0, 0.0, 60, t)
    check(len(node.open_seg) == 1, '入口断面已触发一次在途通行',
          '%d 个在途' % len(node.open_seg))
    jx = x + 8.0                       # 一格跳 8 m：signal_drive 卡死时的 teleport
    node.on_states(states(jx, y))
    node.on_clock(clock(t))
    t += DT
    check(not seg.msgs and not node.open_seg,
          '中途瞬移只作废本次通行，不出假均速',
          '事件%d条 在途%d个' % (len(seg.msgs), len(node.open_seg)))
    t, _, _ = drive(node, jx, y, -1.0, 0.0, 400, t)     # 故意不 reset：靠复位点重建标志
    check(len(seg.msgs) == 1, '瞬移作废后区间仍能正常触发（断面状态已重建）',
          '%d 条' % len(seg.msgs))

    # -------------------------------------- 小跳变（<5 m 弦长）不得变成几十 km/h 假告警
    # 线上出过 20.08 km/h 的假告警和一个 44.77 的假峰，根因不是车速而是跳变：
    # 弦长法把单格 2 m 抖动摊进 0.2 s 整窗就是 36 km/h，旧的 5 m 弦长护栏对它放行。
    # 现在窗口内取子区间速度的中位数：一格离群不影响读数；连跳两格才触发物理上限护栏。
    reset(node)
    cap.msgs = []
    al.msgs = []
    node.jump_drop = 0
    step = V * DT
    xa = z_inst.center[0] + 4.0
    for i in range(120):
        off = -2.0 if i >= 40 else 0.0     # 第 40 格向前跳 2 m（刮蹭级别，不是整段复位）
        node.on_states(states(xa - step * i + off, z_inst.center[1]))
        node.on_clock(clock(t + DT * i))
    t += DT * 120
    check(node.jump_drop == 0, '单格抖动被中位数吸收（不该触发跳变护栏）',
          '拦了 %d 帧' % node.jump_drop)
    check(len(cap.msgs) == 1 and abs(cap.last().speed_kmh - KM) < 0.05,
          '抖动当趟照常出一张干净抓拍，读数不冒充车速',
          '%d 条 读数 %s' % (len(cap.msgs), [c.speed_kmh for c in cap.msgs]))

    # 连跳两格：中位数也被污染，必须落到物理上限护栏上
    reset(node)
    cap.msgs = []
    node.jump_drop = 0
    for i in range(120):
        # 连跳两格：i=41 跳 2 m，i=42 再跳 2 m（累计 -4 m）
        off = (-2.0 if i >= 41 else 0.0) + (-2.0 if i >= 42 else 0.0)
        node.on_states(states(xa - step * i + off, z_inst.center[1]))
        node.on_clock(clock(t + DT * i))
    t += DT * 120
    check(node.jump_drop >= 1, '连跳两格时跳变护栏接住', '拦了 %d 帧' % node.jump_drop)
    check(not [c for c in cap.msgs if c.speed_kmh > node.max_sane],
          '跳变趟不出超上限的抓拍（当趟作废而不是报警）',
          '读数 %s' % [c.speed_kmh for c in cap.msgs])
    # 从圈外反向再走一趟：证明断面状态已按新位置重建，作废不会把点位打死
    t, _, _ = drive(node, xa, z_inst.center[1], -1.0, 0.0, 120, t)
    check(len(cap.msgs) == 1 and abs(cap.last().speed_kmh - KM) < 0.05,
          '作废后下一趟照常抓拍 %.2f km/h' % KM,
          '%d 条 读数 %s' % (len(cap.msgs), getattr(cap.last(), 'speed_kmh', None)))

    # ------------------------------------------------- 条目1：/speed/zones 六种 op
    def call(**kw):
        req = SetSpeedZone.Request()
        resp = SetSpeedZone.Response()
        for k, v in kw.items():
            setattr(req, k, v)
        node.on_zones(req, resp)
        return resp

    RQ = SetSpeedZone.Request
    r = call(op=RQ.OP_LIST)
    check(r.success, 'OP_LIST 可用（常量挂在 Request 上）', r.message)
    r = call(op=RQ.OP_ADD, zone_id='SZ-ST-seg', mode=1, limit_kmh=2.0, radius=1.6,
             entry_x=-19.6, entry_y=-5.0, exit_x=-19.6, exit_y=-13.0, road='自测区间')
    check(r.success and node.find('SZ-ST-seg') is not None,
          'OP_ADD 运行期新增区间点位', r.message)
    r = call(op=RQ.OP_ADD, zone_id='SZ-ST-inst', mode=0, limit_kmh=2.0, radius=1.6,
             center_x=-11.0, center_y=4.6, road='自测瞬时')
    check(r.success and node.find('SZ-ST-inst') is not None,
          'OP_ADD 运行期新增瞬时点位', r.message)
    r = call(op=RQ.OP_SET_LIMIT, zone_id='SZ-ST-inst', limit_kmh=3.5)
    check(r.success and abs(node.find('SZ-ST-inst').limit - 3.5) < 1e-6,
          'OP_SET_LIMIT 改单点阈值', r.message)
    r = call(op=RQ.OP_SET_GLOBAL_LIMIT, limit_kmh=5.0)
    check(r.success and all(abs(z.limit - 5.0) < 1e-6 for z in node.zones),
          'OP_SET_GLOBAL_LIMIT 改全部阈值', r.message)
    r = call(op=RQ.OP_REMOVE, zone_id='SZ-ST-seg')
    r2 = call(op=RQ.OP_REMOVE, zone_id='SZ-ST-不存在')
    check(r.success and not r2.success, 'OP_REMOVE 命中为真、落空为假',
          '%s / %s' % (r.message, r2.message))
    r = call(op=RQ.OP_CLEAR)
    check(r.success and len(node.zones) == len(node.yaml_zones),
          'OP_CLEAR 回到 YAML 基线', '%d 个点位' % len(node.zones))
    r = call(op=99)
    check(not r.success, '未知 op 只回失败不打死节点', r.message)
    check(rclpy.ok() and node.find(z_inst.id) is not None, '节点活到热配置之后')
    # CLEAR 会把 self.zones 换成从 YAML 重建的**新对象**，老引用还停在被
    # OP_SET_GLOBAL_LIMIT 改过的阈值上；后面的统计断言必须拿新对象比。
    z_inst = node.find(z_inst.id)

    # ------------------------------------------------------------ 条目4：流量统计
    reset(node)
    cap.msgs = []
    st.msgs = []
    al.msgs = []
    t, _, _ = drive(node, x0, z_inst.center[1], -1.0, 0.0, 120, t)
    node.publish_stats()
    row = [m for m in st.msgs if m.zone_id == z_inst.id]
    check(bool(row), '统计话题按点位各一条', '%d 条' % len(st.msgs))
    if row:
        m = row[-1]
        check(m.flow == len(cap.msgs) and m.vehicles == 1,
              '车流量/车辆数与逐个事件一致', 'flow=%d 事件%d' % (m.flow, len(cap.msgs)))
        check(abs(m.avg_kmh - KM) < 0.05 and m.max_kmh >= m.min_kmh,
              '平均车速落在窗口里', 'avg=%.2f' % m.avg_kmh)
        check(m.over_count == len(al.msgs) and m.total_flow >= m.flow,
              '超速次数与告警条数一致', 'over=%d 告警%d' % (m.over_count, len(al.msgs)))
        check(abs(m.limit_kmh - z_inst.limit) < 1e-6,
              '统计里的阈值随点位生效', '%.2f' % m.limit_kmh)
    node.destroy_node()
    rclpy.shutdown()
    if FAILED:
        print('SELFTEST FAIL：%d 项\n  - %s' % (len(FAILED), '\n  - '.join(FAILED)))
        return 1
    print('SELFTEST PASS：瞬时/区间/阈值/反向/瞬移/热配置/统计全部符合预期')
    return 0


if __name__ == '__main__':
    try:
        rc = main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)
