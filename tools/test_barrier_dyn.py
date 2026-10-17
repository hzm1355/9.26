#!/usr/bin/env python3
"""在**独立 gzserver + 独立 ROS 域**里实测道闸动力学与四种故障注入。

为什么不在正在跑的栈里测：抬杆/落杆/堵转要看的是"杆真的被物理顶住"这件事，
一次要跑十几轮往复；在主栈里跑，车队与四路 1080p 相机全在抢同一个物理/渲染
线程，超时判据（raise_timeout_s=5.0）会被 RTF 抖动污染，量到的力矩曲线也不能信。
所以照 tools/test_barrier_anchor.py 的路子：GAZEBO_MASTER_URI=11399 +
ROS_DOMAIN_ID=42，只放一道闸、地坪和一个车盒子，跑完自己收掉，主栈全程不受影响。

要回答的问题（对应任务五条目 3、4）：
  1. 位置环 kp/d_gain/effort 在这根 mass_kg×length 的杆上，抬/落到位各要几秒 ——
     这个只能量，算不出来；单程耗时决定 5 s 超时有没有余量。
  2. 抬杆/落杆两个物理动作是否真的到位（|角误差| ≤ angle_tolerance），
     raise_count/lower_count 是否跟着涨。
  3. 地感占用与 yaml 线圈几何对不对得上：核验位**不**占用、杆下**占用**。
  4. 把车开到杆底下再落杆，不打任何注入，插件应当自己从"力矩顶满 + 角速度归零"
     里测出堵转，并且目标角保持在抬起侧（安全侧）。
  5. 故障清除后能恢复正常动作 —— 自持的故障位若不给出口，一次误判就永久瘫痪。
  6. 四种注入各自的挂起形态；其中 SENSOR_FAIL 只冻结地感读数，**不得**把杆本体
     的状态机拖进 STATE_FAULT（那是 manager 降级为计时放行的依据）。

用法：python3 tools/test_barrier_dyn.py [--gate gate_in] [--keep] [--settle 20]
      退出码 0 = 全部断言通过。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from v2x_msgs.msg import BarrierState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import yaml  # noqa: E402
import gen_parking_world as G  # noqa: E402

MASTER_PORT = 11399
ROS_DOMAIN = "42"
WORLD_NAME = "bdyn"
TEST_WORLD = f"/tmp/{WORLD_NAME}.world"
LOG = f"/tmp/{WORLD_NAME}_gzserver.log"

S_CLOSED, S_OPENING, S_OPEN, S_CLOSING, S_FAULT = 0, 1, 2, 3, 4
F_NONE, F_STUCK, F_RAISE, F_NOCLOSE, F_SENSOR = 0, 1, 2, 3, 4
STATE_NAME = {S_CLOSED: "CLOSED", S_OPENING: "OPENING", S_OPEN: "OPEN",
              S_CLOSING: "CLOSING", S_FAULT: "FAULT"}
FAULT_NAME = {F_NONE: "NONE", F_STUCK: "STUCK", F_RAISE: "RAISE_FAIL",
              F_NOCLOSE: "NO_CLOSE", F_SENSOR: "SENSOR_FAIL"}


class Probe(Node):
    """订阅闸机 BarrierState，提供目标角下发、故障注入与"等一个条件成立"。"""

    def __init__(self, gate):
        # 刻意**不**开 use_sim_time：本进程只发指令、看消息、用墙钟计时；
        # 仿真秒一律从消息字段读（action_elapsed_s / header.stamp）。独立实例里
        # 不保证有 /clock 发布者，开 sim 时间反而会把 rclpy 的钟冻在 0。
        super().__init__("barrier_probe")
        self.gate = gate
        self.latest = None
        self.n_msg = 0
        self.transitions = []
        self._last_key = None
        # 发布者全部在构造时建好：DDS 的发现要时间，"create_publisher 完立刻 publish"
        # 实测就是"指令没送到"的假故障（尤其 fault_inject 只有一个 gzserver 端订阅者）。
        self.create_subscription(BarrierState, f"/parking/gate/{gate}/state",
                                 self.on_state, 10)
        self.pub_cmd = self.create_publisher(Float64, f"/parking/gate/{gate}/cmd", 10)
        self.pub_fault = self.create_publisher(String, "/parking/fault_inject", 10)

    def on_state(self, m):
        self.n_msg += 1
        self.latest = m
        key = (m.state, m.fault, m.loop_occupied, m.at_target)
        if self._last_key != key:
            self._last_key = key
            self.transitions.append(
                f"{STATE_NAME.get(m.state, m.state)}/{FAULT_NAME.get(m.fault, m.fault)}"
                f" tgt={m.target_rad:.2f} ang={m.angle_rad:.2f} v={m.angvel_rps:+.2f}"
                f" τ={m.effort_nm:+.0f} t={m.action_elapsed_s:.2f}"
                f" loop={int(m.loop_occupied)}")

    def settle_discovery(self):
        """给发现期一点时间：第一次发布前不 spun，头几条指令会丢。"""
        self.spin_for(3.0)

    def spin_for(self, secs):
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def cmd(self, rad, reps=3):
        m = Float64()
        m.data = float(rad)
        for _ in range(reps):
            self.pub_cmd.publish(m)
            self.spin_for(0.2)

    def fault(self, kind, reps=3):
        m = String()
        m.data = f"{self.gate}:{kind}"
        for _ in range(reps):
            self.pub_fault.publish(m)
            self.spin_for(0.2)
        self.spin_for(0.4)

    def wait(self, pred, secs):
        """等 pred(msg) 成立；返回 (msg|None, 墙钟耗时)。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None and pred(self.latest):
                return self.latest, time.monotonic() - t0
        return None, time.monotonic() - t0


CAR_SDF = """  <model name="car_test">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="chassis">
      <inertial>
        <mass>200.0</mass>
        <inertia><ixx>13.33</ixx><iyy>45.33</iyy><izz>53.33</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="v"><geometry><box><size>1.6 0.8 0.4</size></box></geometry>
        <material><diffuse>0.2 0.4 0.8 1</diffuse></material></visual>
      <collision name="c"><geometry><box><size>1.6 0.8 0.4</size></box></geometry></collision>
    </link>
  </model>
"""


def build_world(cfg, g) -> str:
    """只取这道闸的模型（**保留插件块**）+ 地坪 + 一个车盒子。"""
    block = G.gen({"parking": cfg})
    name = f"parking_{g['id']}"
    head = f'    <model name="{name}">\n'
    start = block.find(head)
    if start < 0:
        raise SystemExit(f"生成块里没有 {name}")
    end = block.find("    </model>\n", start) + len("    </model>\n")
    gate_sdf = block[start:end]
    z = cfg["road_z"]
    return (
        '<?xml version="1.0"?>\n'
        f'<sdf version="1.6"><world name="{WORLD_NAME}">\n'
        '  <physics type="ode"><max_step_size>0.001</max_step_size>'
        '<real_time_factor>1</real_time_factor>'
        '<real_time_update_rate>1000</real_time_update_rate></physics>\n'
        '  <scene><shadows>false</shadows></scene>\n'
        '  <model name="bdyn_floor"><static>true</static><link name="l">'
        f'<visual name="v"><pose>0 0 {z - 3.35} 0 0 0</pose>'
        '<geometry><box><size>60 60 6.7</size></box></geometry></visual>'
        f'<collision name="c"><pose>0 0 {z - 3.35} 0 0 0</pose>'
        '<geometry><box><size>60 60 6.7</size></box></geometry></collision>'
        '</link></model>\n'
        + CAR_SDF.format(x=g["camera"]["capture_point"][0], y=g["queue_point"][1],
                         z=car_z(cfg))
        + gate_sdf
        + "</world></sdf>\n"
    )


def car_z(cfg) -> float:
    """车盒中心高程：半高 0.2 → 顶面 = road_z + 0.45，与 yaml vehicle.body_top
    的实测值同高。杆放平时占 road_z+hub_z±截面高/2 = +0.27..+0.33，正落在车体
    高度带内 —— "落杆砸到车"因此是真接触，不是状态位。"""
    return cfg["road_z"] + 0.25


def move_car(env, x, y, z):
    """平移到 (x,y,z)。gz model 只给 -x/-y/-z 时其余分量按 0，正是要的"不转只挪"。"""
    p = subprocess.run(["gz", "model", "-m", "car_test",
                        "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}"],
                       env=env, capture_output=True, text=True, timeout=20)
    if p.returncode != 0:
        print(f"  !! gz model 挪车失败 rc={p.returncode}: {p.stdout}{p.stderr}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", default="gate_in")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--settle", type=float, default=20.0)
    args = ap.parse_args()

    zones = os.path.join(HERE, os.pardir, "src", "v2x_demo", "config",
                         "parking_zones.yaml")
    with open(zones, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)["parking"]
    hit = [g for g in cfg["gates"] if g["id"] == args.gate]
    if not hit:
        raise SystemExit(f"yaml 里没有闸机 {args.gate}")
    g = hit[0]
    bar = cfg["barrier"]
    OPEN, TOL = bar["open_angle"], bar["angle_tolerance"]
    x_lane = g["camera"]["capture_point"][0]
    qx, qy = g["queue_point"]
    sy = g["staging_point"][1]
    boom_y = g["boom"]["pivot"][1]
    lcx, lcy = g["loop"]["center"]
    lsx, lsy = g["loop"]["size"]
    cz = car_z(cfg)
    # 砸车试验的摆位：车要压在"杆放平后的那条线"上，又必须让开竖起来的杆 ——
    # 竖起杆在铰点 x±截面宽/2，车就整体往车道内侧挪 (半车长 + 0.15)。
    # 不这么摆的话，把车移到杆轴正下方会和竖起的杆互穿，ODE 当场弹射（摆位弹射
    # 是这台的既有实测），测到的就不是"落杆被车顶住"而是碰撞回弹。
    sign = 1.0 if g["direction"] == "in" else -1.0
    jam_x = g["boom"]["pivot"][0] + sign * (0.5 * cfg["vehicle"]["length"] + 0.15)

    world = build_world(cfg, g)
    with open(TEST_WORLD, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(world)
    import xml.etree.ElementTree as ET
    ET.fromstring(world)
    hx = 0.5 * (cfg["vehicle"]["length"] + cfg["vehicle"]["width"])

    # 占用期望：用与插件**同一个公式**（外接方框半边 = (长+宽)/2）从 yaml 几何算出来。
    # 不能按入口闸的直觉写死"核验位一定不占用"：出口线圈是刻意跨在杆轴北侧的
    # （center y=-10.75 对 queue_point y=-9.95），出场车停在核验位时就已经压在框内，
    # "等待即占用"是 parking_zones.yaml 里的设计，manager 靠它拒绝落杆。
    def expect_occ(px, py):
        return (abs(px - lcx) <= hx + lsx / 2 + 1e-9) and \
               (abs(py - lcy) <= hx + lsy / 2 + 1e-9)

    # 占用探测点的 y：优先取线圈中心，但**出口线圈是刻意跨在杆轴上的**（center 距杆轴
    # 0.30 m），车体 y 向半宽 0.4 会压到放平的那根杆 —— gate_out 首轮实测摆位瞬间杆被
    # 顶到 v=+3.4 rad/s、angle 从 0.005 弹到 0.303，量到的是碰撞回弹而不是地感。
    # 所以退到"离杆轴 >= 半车宽 + 0.15"，并要求它仍在占用框内（下面当场验，不验就退出）。
    half_w = 0.5 * cfg["vehicle"]["width"]
    occ_y = lcy
    if abs(occ_y - boom_y) < half_w + 0.15:
        # 往线圈中心那一侧退，退完仍在框内
        occ_y = boom_y + (1.0 if lcy >= boom_y else -1.0) * (half_w + 0.15)
    if not expect_occ(x_lane, occ_y):
        raise SystemExit(f"占用探测点 ({x_lane},{occ_y}) 落在 {g['loop']['id']} 的占用框外，"
                         f"摆位规则要改（框半边 {hx + lsx / 2:.2f}×{hx + lsy / 2:.2f} m）")
    print(f"测试世界 {TEST_WORLD}：{len(world)} 字节，闸机={g['id']} "
          f"open={OPEN:.4f} tol={TOL} 超时={bar['raise_timeout_s']}/"
          f"{bar['lower_timeout_s']} s kp/kd={bar['p_gain']}/{bar['d_gain']} "
          f"effort={bar['effort']}")
    print(f"地感：线圈 {lcx},{lcy} 尺寸 {lsx}×{lsy}，占用框半边 "
          f"{hx + lsx / 2:.2f}×{hx + lsy / 2:.2f} m（车体外接方框 (长+宽)/2={hx:.2f}）")

    env = dict(os.environ, GAZEBO_MASTER_URI=f"http://localhost:{MASTER_PORT}",
               ROS_DOMAIN_ID=ROS_DOMAIN)
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN      # 本进程的 rclpy 也要同域
    logfh = open(LOG, "w")
    # 不带 -u：gzserver 的 -u 是 **--pause**（gzserver --help 实证，名字反直觉）。
    # 带着它启动的世界 paused: true、iterations 恒 0，插件的发布门按 world SimTime()
    # 计时，于是 gzserver 日志里 Load 行照打、/parking/gate/*/state 一条都不发
    # （实测 ros2 topic info -v 显示 Publisher count=1 但 hz 无输出）。
    proc = subprocess.Popen(["gzserver", TEST_WORLD], env=env,
                            stdout=logfh, stderr=subprocess.STDOUT)
    print(f"gzserver PID={proc.pid} MASTER={env['GAZEBO_MASTER_URI']} "
          f"ROS_DOMAIN_ID={ROS_DOMAIN}，等 {args.settle:.0f} s")
    time.sleep(args.settle)
    if proc.poll() is not None:
        print(f"!! gzserver 提前退出（码 {proc.returncode}），日志尾部：")
        print("".join(open(LOG, errors="replace").readlines()[-40:]))
        return 1

    rclpy.init()
    p = Probe(g["id"])
    p.settle_discovery()
    checks = []

    def ck(name, cond, detail):
        checks.append((name, bool(cond)))
        print(f"  {'✓' if cond else '✗'} {name}：{detail}")

    def m():
        return p.latest

    # ── 1 插件上线 ────────────────────────────────────────────────────
    print("\n[1] 插件上线")
    first, _ = p.wait(lambda x: True, 25.0)
    if first is None:
        print("!! 没收到 BarrierState —— 插件没起来。gzserver 日志尾部：")
        print("".join(open(LOG, errors="replace").readlines()[-30:]))
        p.destroy_node()
        rclpy.shutdown()
        proc.kill()
        return 1
    ck("开机静止在落杆位", first.state == S_CLOSED and abs(first.angle_rad) < TOL,
       f"state={STATE_NAME[first.state]} angle={first.angle_rad:.3f} "
       f"target={first.target_rad:.2f} loop={int(first.loop_occupied)} "
       f"fault={FAULT_NAME[first.fault]}")
    n0 = (first.raise_count, first.lower_count)

    # ── 2 正常抬落：单程耗时是后面所有超时判据的底 ────────────────────
    print("\n[2] 正常抬杆→落杆（条目 3 的两个物理动作）")
    p.cmd(OPEN)
    saw_opening, _ = p.wait(lambda x: x.state == S_OPENING, 3.0)
    r, dt = p.wait(lambda x: x.state == S_OPEN and x.at_target, 10.0)
    ck("抬杆到位", r is not None,
       f"OPENING={'有' if saw_opening else '没抓到'} 墙钟 {dt:.1f} s "
       f"仿真 elapsed={r.action_elapsed_s if r else float('nan'):.2f} s "
       f"angle={r.angle_rad if r else float('nan'):.3f}（容差 ±{TOL}）")
    # 保持期采样：真实拉簧配重的杆到位后不该继续抖
    lo = hi = r.angle_rad if r else 0.0
    for _ in range(20):
        p.spin_for(0.1)
        lo, hi = min(lo, m().angle_rad), max(hi, m().angle_rad)
    print(f"      保持 2 s 实测角 {lo:.3f}..{hi:.3f} rad，"
          f"τ={m().effort_nm:+.1f} N·m（重力前馈在 90° 处应接近 0）")
    ck("开到位后不抖", hi - lo < 2 * TOL, f"角波动 {hi - lo:.3f} rad < 2×容差")

    p.cmd(0.0)
    l, dt = p.wait(lambda x: x.state == S_CLOSED and x.at_target, 10.0)
    one_way = max(r.action_elapsed_s if r else 0.0, l.action_elapsed_s if l else 0.0)
    ck("落杆到位", l is not None,
       f"墙钟 {dt:.1f} s 仿真 elapsed={l.action_elapsed_s if l else float('nan'):.2f} s "
       f"angle={l.angle_rad if l else float('nan'):.3f}")
    ck("raise/lower 计数各 +1", l is not None
       and (l.raise_count, l.lower_count) == (n0[0] + 1, n0[1] + 1),
       f"{n0} -> {((l.raise_count, l.lower_count) if l else None)}")
    print(f"      → 单程最长 {one_way:.2f} s，超时阈值 {bar['raise_timeout_s']} s，"
          f"余量 {bar['raise_timeout_s'] / max(one_way, 0.01):.1f} 倍")

    # ── 3 地感占用 ────────────────────────────────────────────────────
    print("\n[3] 地感占用判定（线圈几何 vs 插件外接方框）")
    move_car(env, x_lane, qy, cz)
    p.spin_for(2.0)
    ck("核验位占用与几何期望一致", m() and m().loop_occupied == expect_occ(x_lane, qy),
       f"车 y={qy} 距线圈中心 {abs(qy - lcy):.2f} m，框半边 y 向 {hx + lsy / 2:.2f} m → "
       f"期望={int(expect_occ(x_lane, qy))} 实测 loop={int(m().loop_occupied)}"
       f"（{'入口：核验位在框外，等帧期间杆不会因占用被卡住' if g['direction'] == 'in' else '出口：等待即占用，manager 据此不落杆'}）")
    move_car(env, x_lane, occ_y, cz)
    p.spin_for(2.0)
    ck("杆下/线圈上占用", m() and m().loop_occupied,
       f"车 y={occ_y}（线圈中心 {lcy}，放平的杆在 {boom_y}）"
       f"loop={int(m().loop_occupied)} vehicle_id='{m().vehicle_id}' "
       f"（{g['loop']['id']}）")
    move_car(env, x_lane - (hx + lsx / 2 + 0.3), occ_y, cz)
    p.spin_for(2.0)
    ck("横向驶离后释放", m() and not m().loop_occupied,
       f"车 x={x_lane - (hx + lsx / 2 + 0.3):.2f} loop={int(m().loop_occupied)}")

    # ── 4 真物理防砸：不打注入，杆被车顶住应自己测出堵转 ──────────────
    print("\n[4] 落杆砸到车（无注入；判据必须从实测力矩/角速度里长出来）")
    p.fault("NONE")
    p.cmd(OPEN)
    p.wait(lambda x: x.state == S_OPEN, 10.0)
    move_car(env, jam_x, boom_y, cz)
    p.spin_for(2.0)
    print(f"      车已置于 ({jam_x:.2f},{boom_y})：压在放平杆的线上、让开竖起的杆；"
          f"fault={FAULT_NAME[m().fault]} loop={int(m().loop_occupied)} "
          f"angle={m().angle_rad:.2f}")
    p.cmd(0.0)
    jammed, dt = p.wait(lambda x: x.fault in (F_STUCK, F_NOCLOSE), 12.0)
    ck("自己测出故障", jammed is not None,
       f"墙钟 {dt:.1f} s 后 fault={FAULT_NAME[jammed.fault] if jammed else '无'} "
       f"state={STATE_NAME[jammed.state] if jammed else '?'}")
    if jammed:
        ck("安全侧保持抬起（目标角不回落）", jammed.target_rad > 0.5 * OPEN,
           f"target={jammed.target_rad:.2f} angle={jammed.angle_rad:.2f} "
           f"τ={jammed.effort_nm:+.0f} N·m（上限 {bar['effort']}）")
        p.spin_for(2.0)
        print(f"      继续顶 2 s：angle={m().angle_rad:.2f} v={m().angvel_rps:+.3f} "
              f"τ={m().effort_nm:+.0f} elapsed={m().action_elapsed_s:.2f} s "
              f"（杆被车顶住没落下去）")

    # ── 5 故障清除后恢复 ─────────────────────────────────────────────
    print("\n[5] 故障清除后恢复正常动作")
    p.fault("NONE")
    move_car(env, x_lane, qy, cz)
    p.spin_for(2.0)
    p.cmd(OPEN)
    rec, dt = p.wait(lambda x: x.state == S_OPEN and x.at_target
                     and x.fault == F_NONE, 10.0)
    ck("清障后能重新抬杆", rec is not None, f"墙钟 {dt:.1f} s")
    p.cmd(0.0)
    rec, dt = p.wait(lambda x: x.state == S_CLOSED and x.at_target
                     and x.fault == F_NONE, 10.0)
    ck("清障后能重新落杆、故障位归零", rec is not None,
       f"墙钟 {dt:.1f} s state={STATE_NAME[m().state]} fault={FAULT_NAME[m().fault]}")

    # ── 6 四种注入 ───────────────────────────────────────────────────
    print("\n[6] 四种故障注入")
    p.fault("STUCK")
    p.cmd(OPEN)
    r, _ = p.wait(lambda x: x.fault == F_STUCK and x.state == S_FAULT, 12.0)
    ck("STUCK：卡滞挂起、安全侧保持抬起", r is not None and r.target_rad > 0.5 * OPEN,
       f"fault={FAULT_NAME[m().fault]} state={STATE_NAME[m().state]} "
       f"angle={m().angle_rad:.2f} τ={m().effort_nm:+.0f}")
    p.fault("NONE")
    p.cmd(0.0)
    p.wait(lambda x: x.state == S_CLOSED and x.at_target, 10.0)

    p.fault("RAISE_FAIL")
    p.cmd(OPEN)
    r, dt = p.wait(lambda x: x.fault == F_RAISE and x.state == S_FAULT, 12.0)
    ck("RAISE_FAIL：抬杆方向驱动力被掐走、杆原地不动",
       r is not None and r.angle_rad < TOL,
       f"墙钟 {dt:.1f} s angle={m().angle_rad:.3f}（没抬起）"
       f" elapsed={m().action_elapsed_s:.2f} target={m().target_rad:.2f}")
    p.fault("NONE")
    p.cmd(OPEN)
    p.wait(lambda x: x.state == S_OPEN, 10.0)

    p.fault("NO_CLOSE")
    p.cmd(0.0)
    r, dt = p.wait(lambda x: x.fault == F_NOCLOSE and x.state == S_FAULT, 12.0)
    ck("NO_CLOSE：落杆方向被掐走、杆保持抬起",
       r is not None and r.angle_rad > 0.5 * OPEN,
       f"墙钟 {dt:.1f} s angle={m().angle_rad:.3f} target={m().target_rad:.2f}")
    p.fault("NONE")
    p.cmd(0.0)
    p.wait(lambda x: x.state == S_CLOSED, 10.0)

    # 用占用探测点而不是杆轴：这里的杆是放平状态，把车摆到杆轴上和 [4] 一样会互穿，
    # 而地感占用在 occ_y 同样成立。
    move_car(env, x_lane, occ_y, cz)
    p.spin_for(2.0)
    occ_before = m().loop_occupied
    p.fault("SENSOR_FAIL")
    # 挪到摆渡位：两点都按 expect_occ 算，必须一个占用一个不占用，否则"读数没变"
    # 可能只是本来就没变，冻结证据不成立（出口核验位就落在框内，所以不能用它）。
    move_car(env, x_lane, sy, cz)
    p.spin_for(2.5)
    ck("SENSOR_FAIL：地感读数冻结、杆本体的状态机不被拖进 FAULT",
       m().fault == F_SENSOR and m().state != S_FAULT and m().loop_occupied == occ_before
       and expect_occ(x_lane, sy) != occ_before,
       f"注入前（占用点 y={occ_y}）占用={int(occ_before)}，车已挪到摆渡位 y={sy}"
       f"（几何期望={int(expect_occ(x_lane, sy))}）仍报={int(m().loop_occupied)} "
       f"state={STATE_NAME[m().state]}（manager 靠这个位降级为计时放行）")
    p.fault("NONE")
    p.spin_for(2.0)
    ck("清除后占用判定恢复实时",
       m().fault == F_NONE and m().loop_occupied == expect_occ(x_lane, sy),
       f"车在摆渡位 y={sy} 期望={int(expect_occ(x_lane, sy))} "
       f"实测 loop={int(m().loop_occupied)}")

    # ── 报告 ─────────────────────────────────────────────────────────
    print(f"\n收到 {g['id']} 状态 {p.n_msg} 条（20 Hz 名义）")
    print("--- 状态迁移轨迹（插件自己的口径）---")
    for line in p.transitions:
        print("  " + line)
    print("--- gzserver 日志里的 v2x_barrier 行 ---")
    for line in open(LOG, errors="replace").read().splitlines():
        if "v2x_barrier" in line:
            print("  " + line.strip())
    bad = [nm for nm, okk in checks if not okk]
    print(f"\n断言 {len(checks)} 条，通过 {len(checks) - len(bad)}，失败 {len(bad)}"
          + (f"：{bad}" if bad else ""))

    p.destroy_node()
    rclpy.shutdown()
    logfh.close()
    if args.keep:
        print(f"按 --keep 保留 gzserver（PID {proc.pid}）；"
              f"手查完 kill -9 或 fuser -k {MASTER_PORT}/tcp")
    else:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
