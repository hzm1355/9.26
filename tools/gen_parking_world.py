#!/usr/bin/env python3
"""从 parking_zones.yaml 生成停车场内联 SDF，幂等地插进 v2x_city_scene.world。

用法
    python3 tools/gen_parking_world.py --print      # 只打生成结果，不碰 world
    python3 tools/gen_parking_world.py              # 备份后写回 world
    python3 tools/gen_parking_world.py --zones X --world Y

为什么要有这个脚本：条目1 要求"点位、尺寸可配置"。手改 world 等于把 100 多个数字
再抄到第二个地方，改一处漏一处；这里所有几何都从同一份 yaml 反算，而体检脚本
tools/check_parking_layout.py 量的就是本脚本将要生成的那件东西（碰撞半径、杆高、
机箱位置都从 yaml 读），所以"通过体检"和"生成结果"不会各说各话。

三条硬规矩（前面几个任务用真金白银换来的）：
  1) 地坪 / 划线 / 地感框一律**纯 visual 无碰撞**。沙盘支承来自 room_33 那个
     50×6.7×51.2 的等效盒（顶面 z=6.7），再叠一层碰撞只会跟它抢接触对、把车弹飞
     （任务四的复位风暴就是这么来的）。
  2) 带碰撞的静态件距车道中心 ≥1.5 m；道闸机箱是例外，所以机箱整体偏到车道外，
     只留一根 r=0.10 的柱子贴在车道缘线上。
  3) 挂了 camera sensor + SensorPlugin 的模型**永远不要 /delete_entity**，
     那会把整个 gzserver 带走（任务三实测）。所以门架与相机要做错也得改完重来，
     不能运行期增删。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

BEGIN_MARK = "════ BEGIN GENERATED: parking（tools/gen_parking_world.py 生成，勿手改）════"
END_MARK = "════ END GENERATED: parking ════"
ANCHOR = "    <!-- Gazebo ROS 插件 -->"

# ── 配色（Gazebo Classic 的 ambient/diffuse 四元组）──────────────────────
# 停车场地坪 2026-10-17 由黑沥青改绿：绿色调仍按"哑光铺装"取值（diffuse 绿通道
# 0.38，不是荧光绿），保证与 C_LINE 白线、C_YELLOW 黄线的亮度差留在原比例上 ——
# 卡口抓拍与画面判读吃的是对比度，不是色相。
C_ASPHALT = ("0.07 0.14 0.09 1", "0.18 0.38 0.22 1")
C_LINE = ("0.55 0.55 0.52 1", "0.86 0.86 0.82 1")
C_YELLOW = ("0.55 0.46 0.10 1", "0.86 0.72 0.16 1")
C_STEEL = ("0.25 0.25 0.28 1", "0.45 0.45 0.50 1")
C_DARK = ("0.05 0.05 0.06 1", "0.15 0.15 0.18 1")
C_BOOM_RED = ("0.55 0.06 0.06 1", "0.86 0.12 0.12 1")
C_BOOM_WHITE = ("0.60 0.60 0.58 1", "0.92 0.92 0.88 1")
C_BOOTH = ("0.20 0.26 0.30 1", "0.38 0.46 0.52 1")
C_GLASS = ("0.10 0.16 0.20 1", "0.55 0.70 0.78 1")
C_SHUTTLE = ("0.10 0.42 0.48 1", "0.18 0.66 0.72 1")


def n(v: float) -> str:
    """把浮点成 SDF 里的定宽小数，顺带把 -0.0000 归零，diff 才干净。"""
    s = f"{v:.4f}"
    if float(s) == 0.0:
        s = "0.0000"
    return s


def mat(amb: str, dif: str) -> str:
    return (f"<material><ambient>{amb}</ambient><diffuse>{dif}</diffuse>"
            f"<specular>0 0 0 1</specular><emissive>0 0 0 1</emissive></material>")


def box(sx: float, sy: float, sz: float) -> str:
    return f"<geometry><box><size>{n(sx)} {n(sy)} {n(sz)}</size></box></geometry>"


def cyl(r: float, length: float) -> str:
    return f"<geometry><cylinder><radius>{n(r)}</radius><length>{n(length)}</length></cylinder></geometry>"


def visual(name: str, pose: str, geom: str, colors) -> str:
    amb, dif = colors
    return (f'        <visual name="{name}">\n'
            f"          <pose>{pose}</pose>\n"
            f"          {geom}\n"
            f"          {mat(amb, dif)}\n"
            f"        </visual>\n")


def collision(name: str, pose: str, geom: str) -> str:
    return (f'        <collision name="{name}">\n'
            f"          <pose>{pose}</pose>\n"
            f"          {geom}\n"
            f"        </collision>\n")


def inertial_box(mass: float, sx: float, sy: float, sz: float, pose: str = "") -> str:
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    p = f"          <pose>{pose}</pose>\n" if pose else ""
    return (f"        <inertial>\n{p}"
            f"          <mass>{n(mass)}</mass>\n"
            f"          <inertia>"
            f"<ixx>{n(ixx)}</ixx><ixy>0</ixy><iyy>{n(iyy)}</iyy>"
            f"<ixz>0</ixz><iyz>0</iyz><izz>{n(izz)}</izz>"
            f"</inertia>\n"
            f"        </inertial>\n")


class Stripe:
    """一块地面上的划线：薄盒 + 可选朝向。纯 visual，无碰撞。"""

    def __init__(self):
        self.parts: list[str] = []
        self.k = 0

    def line(self, x0: float, y0: float, x1: float, y1: float, width: float,
             colors, dz: float = 0.0) -> None:
        """两点定义的一条直线段（按轴向对齐的矩形；斜线按长度铺开并旋转）。"""
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        yaw = math.atan2(dy, dx)
        self.patch(cx, cy, length, width, 0.012, yaw, colors, dz)

    def patch(self, cx: float, cy: float, sx: float, sy: float, sz: float,
              yaw: float, colors, dz: float = 0.0, name: str | None = None) -> None:
        self.k += 1
        nm = name or f"s{self.k:03d}"
        pose = f"{n(cx)} {n(cy)} {n(dz + sz / 2.0)} 0 0 {n(yaw)}"
        self.parts.append(visual(nm, pose, box(sx, sy, sz), colors))


def fmt_pose_xyz(x: float, y: float, z: float, r: float = 0.0, p: float = 0.0, yw: float = 0.0) -> str:
    return f"{n(x)} {n(y)} {n(z)} {n(r)} {n(p)} {n(yw)}"


# ══════════════════════════════════════════════════════════════════════
def gen_shuttle_car(pk: dict) -> str:
    """摆渡车 car_6：与 car_1..car_5 同一套底盘，只是换名、换 namespace、换初始位姿。

    为什么走 world 内联而不是运行期 spawn：停车场那一侧的判定全挂在**模型名**上
    （BarrierState 的 vehicle_id 取自 /model_states，插件按 shuttle.prefix 过滤；
    贴牌插件按模型名找 visual）。用 `gz model` 或 spawn 服务起的车名是 car_6_0 之类
    的自动后缀，整条链路都会对不上号。内联进生成块还有一条好处：重跑本脚本幂等，
    不会出现"这次起车失败所以停车场没车"那种只在现场复现的状态。

    尺寸全部从 yaml 的 parking.vehicle 反算（条目1 的"尺寸可配置"对车也一样）；
    底盘动力学参数（max_wheel_acceleration 6.0 等）逐字照抄既有五车 —— 那套数是被
    任务四的甩尾和复位弹射校准过的，换一辆车不该重新发明。
    """
    sh = pk["shuttle"]
    v = pk["vehicle"]
    L, W = v["length"], v["width"]
    model = sh["model"]
    sx, sy, heading = sh["entry_route"][0]
    # park_pose 是给"退役但保留模型"的车用的：car_6 不再上场后，它的出生位必须与
    # fleets 里在用车的出生位分开（同格加载必互顶弹射），而航线仍照 entry_route[0]
    # 标定。没写 park_pose 时行为与以前逐字一致。
    if sh.get("park_pose"):
        sx, sy, heading = sh["park_pose"]
    # 车姿 z 沿用五车的 7.0：路面 6.7 是等效盒顶面，车落定后 chassis 原点约 6.70，
    # 初值给 +0.30 让它自由落体几十毫秒，避免一开始就与盒体互楔。
    z = pk["road_z"] + 0.30
    sep = W - 0.10                      # 轮心横距（与五车同为 0.70）
    wb = L / 2.0 - 0.40                 # 轮心纵距：前轮 +0.40、后轮 -0.40
    r = 0.10
    out: list[str] = []
    out.append(f'    <!-- ── 停车场摆渡车 {model}（本函数生成，与五车同底盘）──\n'
               f'         初始位姿取 parking.shuttle.'
               f'{"park_pose" if sh.get("park_pose") else "entry_route[0]"} = '
               f'[{n(sx)}, {n(sy)}, {n(heading)}]\n    -->\n')
    out.append(f'    <model name="{model}">\n')
    out.append(f'      <pose>{n(sx)} {n(sy)} {n(z)} 0 0 {n(heading)}</pose>\n\n')
    out.append('      <link name="chassis">\n')
    out.append(inertial_box(80.0, L, W, 0.4, f'0 0 {n(0.25)}'))
    out.append(collision("chassis_collision", f'0 0 {n(0.25)} 0 0 0', box(L, W, 0.4)))
    out.append(visual("chassis_body", f'0 0 {n(0.25)} 0 0 0', box(L, W, 0.4), C_SHUTTLE))
    ph = f"{n(v['plate_ahead'])} 0 {n(0.15)}"
    pa = f'{-v["plate_ahead"]:.4f} 0 {n(0.15)}'
    plate_mat = ('<material><script>\n'
                 '            <uri>file:///tmp/v2x_plates/placeholder/placeholder.material'
                 '</uri>\n'
                 '            <name>v2x/plate_placeholder</name>\n'
                 '          </script></material>')
    out.append(f'        <visual name="plate_front">\n'
               f'          <pose>{ph} 0 0 0</pose>\n'
               f'          <geometry><box><size>0.0200 0.4400 0.1400</size></box></geometry>\n'
               f'          {plate_mat}\n'
               f'        </visual>\n')
    out.append(f'        <visual name="plate_rear">\n'
               f'          <pose>{pa} 0 0 {n(math.pi)}</pose>\n'
               f'          <geometry><box><size>0.0200 0.4400 0.1400</size></box></geometry>\n'
               f'          {plate_mat}\n'
               f'        </visual>\n')
    out.append(visual("cabin", f'0 {n(-0.08)} {n(0.57)} 0 0 0',
                      box(0.72, 0.68, 0.24), C_DARK))
    out.append('      </link>\n\n')
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        out.append(f'      <link name="{side}_wheel_link">\n')
        out.append(f'        <pose>{n(wb)} {n(sgn * sep / 2.0)} {n(r)} 0 0 0</pose>\n')
        out.append('        <inertial>\n          <mass>2.0</mass>\n'
                   '          <inertia><ixx>0.005067</ixx><iyy>0.010000</iyy>'
                   '<izz>0.005067</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n'
                   '        </inertial>\n')
        # collision 里必须把 surface 一起写完（helper collision() 只出 pose+geometry，
        # 摩擦/接触写在它外面就成了 link 级的野标签，SDF 校验直接拒）
        out.append(f'        <collision name="collision">\n'
                   f'          <pose>0 0 0 {n(math.pi / 2)} 0 0</pose>\n'
                   f'          {cyl(r, 0.08)}\n'
                   '          <surface>\n'
                   '            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>\n'
                   '            <contact><ode><min_depth>0.001</min_depth><kp>1e8</kp></ode>'
                   '</contact>\n          </surface>\n'
                   '        </collision>\n')
        out.append(visual("visual", f'0 0 0 {n(math.pi / 2)} 0 0', cyl(r, 0.08), C_DARK))
        out.append('      </link>\n')
        out.append(f'      <joint name="{side}_wheel_joint" type="revolute">\n'
                   f'        <parent>chassis</parent>\n'
                   f'        <child>{side}_wheel_link</child>\n'
                   '        <axis><xyz>0 1 0</xyz><dynamics><friction>0.1</friction>'
                   '</dynamics></axis>\n'
                   f'      </joint>\n\n')
    out.append('      <link name="caster_link">\n')
    out.append(f'        <pose>{n(-wb)} 0 {n(0.06)} 0 0 0</pose>\n')
    out.append('        <inertial>\n          <mass>0.5</mass>\n'
               '          <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>'
               '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>\n        </inertial>\n')
    out.append('        <collision name="collision">\n'
               '          <geometry><sphere><radius>0.0600</radius></sphere></geometry>\n'
               '          <surface>\n'
               '            <friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction>\n'
               '            <contact><ode><min_depth>0.001</min_depth><kp>1e8</kp></ode>'
                   '</contact>\n          </surface>\n'
               '        </collision>\n')
    out.append(visual("visual", "0 0 0 0 0 0", '<geometry><sphere><radius>0.0600</radius>'
                      '</sphere></geometry>', C_STEEL))
    out.append('      </link>\n')
    out.append('      <joint name="caster_joint" type="fixed">\n'
               '        <parent>chassis</parent>\n        <child>caster_link</child>\n'
               '      </joint>\n\n')
    out.append('      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">\n'
               '        <ros>\n'
               f'          <namespace>/{model}</namespace>\n'
               '          <remapping>cmd_vel:=cmd_vel</remapping>\n'
               '          <remapping>odom:=odom</remapping>\n'
               '        </ros>\n'
               '        <update_rate>100</update_rate>\n'
               '        <cmd_vel_timeout>0.3</cmd_vel_timeout>\n'
               '        <left_joint>left_wheel_joint</left_joint>\n'
               '        <right_joint>right_wheel_joint</right_joint>\n'
               f'        <wheel_separation>{n(sep)}</wheel_separation>\n'
               f'        <wheel_diameter>{n(2 * r)}</wheel_diameter>\n'
               '        <max_wheel_torque>12.0</max_wheel_torque>\n'
               '        <max_wheel_acceleration>6.0</max_wheel_acceleration>\n'
               '        <publish_odom>true</publish_odom>\n'
               '        <publish_odom_tf>false</publish_odom_tf>\n'
               '        <publish_wheel_tf>false</publish_wheel_tf>\n'
               f'        <odometry_frame>{model}_odom</odometry_frame>\n'
               f'        <robot_base_frame>{model}/base_link</robot_base_frame>\n'
               '      </plugin>\n'
               '    </model>\n\n')
    return "".join(out)


def gen(cfg: dict) -> str:
    pk = cfg["parking"]
    road_z = pk["road_z"]
    floor_z = pk["floor_z"]
    stripe_z = pk["stripe_z"]
    lot = pk["lot"]
    x0, x1, y0, y1 = lot["x0"], lot["x1"], lot["y0"], lot["y1"]
    bar = pk["barrier"]
    st = pk["stripes"]
    gan = st["gantry"]
    med = st["median"]
    sec_w, sec_h = bar["section"]
    post_r = bar["post_radius"]
    post_h = bar["post_height"]
    stub = bar["stub_back"]
    hx, hy, hz = bar["housing_size"]

    out: list[str] = []
    out.append(f"    <!-- {BEGIN_MARK}\n"
               "         数据源：src/v2x_demo/config/parking_zones.yaml（唯一真值来源）\n"
               "         重生成：python3 tools/gen_parking_world.py\n"
               "         改点位请改 yaml 再重跑本脚本 + tools/check_parking_layout.py，\n"
               "         直接改这一段会在下次生成时被整块覆盖掉。\n"
               "    -->\n")

    # ══ 1 地坪 ═══════════════════════════════════════════════════════
    # 沙盘网格在这块场地一片三角形都没有（见 yaml 顶部 (2) 的覆盖复核），所以地坪
    # 必须自铺。厚度 0.05、顶面 floor_z：底面嵌进 room_33 等效盒体内，既不给车
    # 增加第二个接触面，也不会和 6.7 的盒顶面 Z-fighting。
    lx, ly = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    out.append('    <model name="parking_floor">\n'
               '      <static>true</static>\n'
               f'      <pose>{fmt_pose_xyz(cx, cy, road_z)}</pose>\n'
               '      <link name="link">\n')
    out.append(visual("asphalt", fmt_pose_xyz(0, 0, floor_z - road_z - 0.025),
                      box(lx, ly, 0.05), C_ASPHALT))
    out.append("      </link>\n    </model>\n\n")

    # ══ 2 划线 ═══════════════════════════════════════════════════════
    s = Stripe()
    dz0 = stripe_z - road_z                      # 划线层的相对高度基准
    W = 0.10                                     # 车道边线宽

    # 场地四周围
    s.line(x0, y0, x1, y0, W, C_LINE, dz0)
    s.line(x0, y1, x1, y1, W, C_LINE, dz0)
    s.line(x0, y0, x0, y1, W, C_LINE, dz0)
    s.line(x1, y0, x1, y1, W, C_LINE, dz0)

    # 通道带（东西向）两侧
    ai = st["aisle"]
    s.line(ai["x0"], ai["center_y"] - ai["width"] / 2, ai["x1"],
           ai["center_y"] - ai["width"] / 2, W, C_LINE, dz0)
    s.line(ai["x0"], ai["center_y"] + ai["width"] / 2, ai["x1"],
           ai["center_y"] + ai["width"] / 2, W, C_LINE, dz0)

    # 进出口两条车道边线 + 中央分隔岛（黄线，禁停）
    for tag in ("lane_in", "lane_out"):
        ln = st[tag]
        s.line(ln["x0"], ln["y0"], ln["x0"], ln["y1"], W, C_LINE, dz0)
        s.line(ln["x1"], ln["y0"], ln["x1"], ln["y1"], W, C_LINE, dz0)
    mw = 0.08
    s.line(med["x0"], med["y0"], med["x1"], med["y0"], mw, C_YELLOW, dz0)
    s.line(med["x0"], med["y1"], med["x1"], med["y1"], mw, C_YELLOW, dz0)
    s.line(med["x0"], med["y0"], med["x0"], med["y1"], mw, C_YELLOW, dz0)
    s.line(med["x1"], med["y0"], med["x1"], med["y1"], mw, C_YELLOW, dz0)

    # 车位：开口朝通道侧（南侧），画三条边 + 车位编号块
    for pkm in pk["stalls"]:
        sx0, sy0 = pkm["center"]
        pw, pd = pkm["size"]
        left, right = sx0 - pw / 2, sx0 + pw / 2
        south, north = sy0 - pd / 2, sy0 + pd / 2
        s.line(left, south, left, north, W, C_LINE, dz0)
        s.line(right, south, right, north, W, C_LINE, dz0)
        s.line(left, north, right, north, W, C_LINE, dz0)
        # 挡车器（车轮止退块），贴在车位北端
        s.patch(sx0, north - 0.16, pw * 0.55, 0.10, 0.09, 0.0, C_YELLOW, dz0)

    # 停止线：闸杆正前方 0.22 m（车停核验位时车头离杆 0.2~0.45 m，画在杆前一掌）
    for g in pk["gates"]:
        py = g["boom"]["pivot"][1]
        ys = py - 0.22 if g["direction"] == "in" else py + 0.22
        ln = st["lane_in" if g["direction"] == "in" else "lane_out"]
        s.line(ln["x0"] + 0.05, ys, ln["x1"] - 0.05, ys, 0.20, C_LINE, dz0)

    # 地感线圈：黄框 + 内框（现实里是切槽埋线，这里画成方框供肉眼核对位置）
    for g in pk["gates"]:
        lp = g["loop"]
        lcx, lcy = lp["center"]
        lw, ld = lp["size"]
        l0, l1 = lcx - lw / 2, lcx + lw / 2
        t0, t1 = lcy - ld / 2, lcy + ld / 2
        for (a, b, c, d) in ((l0, t0, l1, t0), (l0, t1, l1, t1),
                             (l0, t0, l0, t1), (l1, t0, l1, t1)):
            s.line(a, b, c, d, 0.06, C_YELLOW, dz0 + 0.004)
        s.patch(lcx, lcy, 0.18, 0.18, 0.014, math.radians(45), C_YELLOW, dz0 + 0.004)

    # 行进方向箭头：入口车道北行（+y）、出口车道南行（-y）。
    # patch 的 sx 是沿局部 x 的长度，所以箭杆必须 sx=长、sy=宽，yaw 取方向角；
    # 两翼各是一条短条，一个端点钉在箭头上、向后张开 35°，
    # 于是翼中心 = 箭头 + (翼长/2)·u，u 是与方向夹 180∓35° 的单位向量。
    SHAFT_L, WING_L, WING_DEG, STROKE = 0.55, 0.26, 35.0, 0.06
    for tag, dsign in (("lane_in", 1.0), ("lane_out", -1.0)):
        ln = st[tag]
        yaw = math.radians(90.0) * dsign
        ux, uy = math.cos(yaw), math.sin(yaw)        # = (0, dsign)
        tip_x, tip_y = ln["center_x"], ln["y0"] + (0.90 if dsign > 0 else 0.30)
        s.patch(tip_x - ux * SHAFT_L / 2, tip_y - uy * SHAFT_L / 2,
                SHAFT_L, STROKE, 0.012, yaw, C_LINE, dz0)
        for sgn in (-1, 1):
            a = yaw + sgn * math.radians(180.0 - WING_DEG)
            s.patch(tip_x + math.cos(a) * WING_L / 2, tip_y + math.sin(a) * WING_L / 2,
                    WING_L, STROKE, 0.012, a, C_LINE, dz0)

    out.append('    <model name="parking_stripes">\n'
               '      <static>true</static>\n'
               f'      <pose>{fmt_pose_xyz(cx, cy, road_z)}</pose>\n'
               '      <link name="link">\n')
    out.append("".join(s.parts))
    out.append("      </link>\n    </model>\n\n")

    # ══ 3 岗亭 ═══════════════════════════════════════════════════════
    bx, by = med["booth"]
    bw, bd = med["booth_size"]
    bh = med["booth_height"]
    out.append('    <model name="parking_booth">\n'
               '      <static>true</static>\n'
               f'      <pose>{fmt_pose_xyz(bx, by, road_z)}</pose>\n'
               '      <link name="link">\n')
    out.append(visual("body", fmt_pose_xyz(0, 0, bh / 2), box(bw, bd, bh), C_BOOTH))
    out.append(visual("glass", fmt_pose_xyz(0, 0, bh * 0.62), box(bw * 1.01, bd * 1.01, bh * 0.22), C_GLASS))
    out.append(visual("roof", fmt_pose_xyz(0, 0, bh + 0.05), box(bw + 0.24, bd + 0.24, 0.10), C_STEEL))
    out.append(collision("body_col", fmt_pose_xyz(0, 0, bh / 2), box(bw, bd, bh)))
    out.append("      </link>\n    </model>\n\n")

    # ══ 4 龙门架 + 两台卡口相机 ══════════════════════════════════════
    gx0, gx1 = gan["x0"], gan["x1"]
    beam_cx = (gx0 + gx1) / 2.0
    beam_z = gan["z_center"] - road_z
    bw_, bd_, bh_ = gan["beam_size"]
    mast_top = beam_z + bh_ / 2 + gan["mast_top_pad"]
    out.append('    <model name="parking_gantry">\n'
               '      <static>true</static>\n'
               f'      <pose>{fmt_pose_xyz(beam_cx, gan["y"], road_z)}</pose>\n'
               '      <!-- 相机吊在两条车道中心线的正上方、且整体退到镜头平面之后'
               '（任务三 v2 的教训：横臂前伸 6 cm 就抓到 82% 自家立杆）。'
               '本模型带 camera sensor，运行期绝不可 delete_entity。 -->\n'
               '      <link name="link">\n')
    for i, (mx, my) in enumerate(gan["masts"], start=1):
        rel_x = mx - beam_cx
        out.append(visual(f"mast{i}", fmt_pose_xyz(rel_x, my - gan["y"], mast_top / 2),
                          cyl(gan["mast_radius"], mast_top), C_STEEL))
        out.append(collision(f"mast{i}_col", fmt_pose_xyz(rel_x, my - gan["y"], mast_top / 2),
                             cyl(gan["mast_radius"], mast_top)))
        out.append(visual(f"mast{i}_base", fmt_pose_xyz(rel_x, my - gan["y"], 0.04),
                          cyl(gan["mast_radius"] * 2.4, 0.08), C_DARK))
    out.append(visual("beam", fmt_pose_xyz(0, 0, beam_z), box(gx1 - gx0, bd_, bh_), C_STEEL))
    out.append(visual("beam_trim", fmt_pose_xyz(0, 0, beam_z - bh_ / 2 - 0.02),
                      box(gx1 - gx0, bd_ * 0.6, 0.04), C_YELLOW))
    CAM_H = 0.16                      # 相机机壳高；yaml 的 gantry.z_center 按"下表面压住机壳顶"来定

    for g in pk["gates"]:
        cam = g["camera"]
        wx, wy, wz, cr, cp, cyw = cam["pose"]
        sensor = f'        <sensor name="gate_camera_{g["direction"]}" type="camera">\n'
        sensor += f'          <pose>{fmt_pose_xyz(wx - beam_cx, wy - gan["y"], wz - road_z, cr, cp, cyw)}</pose>\n'
        sensor += "          <always_on>true</always_on>\n"
        sensor += f'          <update_rate>{int(cam["update_rate"])}</update_rate>\n'
        sensor += "          <visualize>false</visualize>\n"
        sensor += f'          <camera name="{cam["gate_id"]}">\n'
        sensor += f'            <horizontal_fov>{n(cam["hfov"])}</horizontal_fov>\n'
        sensor += "            <image><width>%d</width><height>%d</height>" \
                  "<format>R8G8B8</format></image>\n" % (cam["width"], cam["height"])
        sensor += f'            <clip><near>{n(cam["clip_near"])}</near>' \
                  f'<far>{n(cam["clip_far"])}</far></clip>\n'
        sensor += "          </camera>\n"
        # 插件 name 属性必须逐路唯一，这就是消歧的正解，不是风格问题：本版
        # libgazebo_ros_node 的节点名取自 <plugin name=>，而 <ros><node_name> 实测惰性
        # （改前日志仍是 [parking.gazebo_ros_camera]，改后是 [parking.gazebo_ros_camera_gate_in]）。
        # 两路相机共用 namespace /parking，节点名一撞就触发
        # "Found multiple nodes with same name" 并在 libgazebo_ros_node.so 里踩空指针，
        # gzserver 直接段错误（dmesg: segfault at 3a0）。只在渲染开启时复现，headless 验不出。
        # 话题名不受影响，仍是 <namespace>/<camera_name>/image_raw。
        sensor += '          <plugin name="gazebo_ros_camera_%s" filename="libgazebo_ros_camera.so">\n' \
                  % cam["gate_id"]
        sensor += "            <ros>\n"
        sensor += f'              <namespace>{cam["namespace"]}</namespace>\n'
        sensor += "            </ros>\n"
        # 话题名就是 <namespace>/<camera_name>/image_raw，这条硬规则实测过：plugin 里
        # 加 ~/image_raw:=camera/image_raw 的 remapping 不参与命名，白写两行。
        sensor += f'            <camera_name>{cam["gate_id"]}</camera_name>\n'
        sensor += f'            <frame_name>{cam["gate_id"]}/camera_link</frame_name>\n'
        sensor += "          </plugin>\n"
        fx, fy, fz = cam["focus"]
        sensor += '          <plugin name="v2x_gate" filename="libv2x_gate.so">\n'
        sensor += f'            <gate_id>{cam["gate_id"]}</gate_id>\n'
        sensor += f'            <capture_dir>{cam["capture_dir"]}</capture_dir>\n'
        sensor += f'            <focus>{n(fx)} {n(fy)} {n(fz)}</focus>\n'
        sensor += f'            <near_radius>{n(cam["near_radius"])}</near_radius>\n'
        sensor += f'            <far_radius>{n(cam["far_radius"])}</far_radius>\n'
        sensor += f'            <frame_wait>{n(cam["frame_wait"])}</frame_wait>\n'
        # 核验位是停着等的，一次超时若不解武装就再也触发不了（进场车道内够不到 far），
        # 所以停车场两道闸必须带这个开关。yaml 里没写时按 true 生成。
        sensor += '            <rearm_on_fail>%s</rearm_on_fail>\n' \
                  % ("true" if cam.get("rearm_on_fail", True) else "false")
        sensor += f'            <render_lag>{n(cam["render_lag"])}</render_lag>\n'
        sensor += f'            <vehicle_prefix>{pk["shuttle"]["prefix"]}</vehicle_prefix>\n'
        sensor += f'            <chassis_link>{pk["shuttle"]["chassis_link"]}</chassis_link>\n'
        sensor += "            <image_format>R8G8B8</image_format>\n"
        sensor += "          </plugin>\n"
        sensor += "        </sensor>\n"
        out.append(sensor)
        # 相机机壳与吊杆（纯视觉）：横梁下表面压在机壳顶面之上；真有净空才画吊杆
        out.append(visual(f"cambox_{g['direction']}",
                          fmt_pose_xyz(wx - beam_cx, wy - gan["y"], wz - road_z),
                          box(0.30, 0.20, CAM_H), C_DARK))
        gap = beam_z - bh_ / 2.0 - (wz - road_z + CAM_H / 2.0)
        if gap > 0.02:
            out.append(visual(f"camstem_{g['direction']}",
                              fmt_pose_xyz(wx - beam_cx, wy - gan["y"],
                                           wz - road_z + CAM_H / 2.0 + gap / 2.0),
                              cyl(0.03, gap), C_STEEL))
    out.append("      </link>\n    </model>\n\n")

    # ══ 5 两道道闸（非静态 + revolute 杆）════════════════════════════
    for g in pk["gates"]:
        gid = g["id"]
        dirc = g["direction"]
        px, py = g["boom"]["pivot"]
        ax, ay, az = g["boom"]["axis"]
        alen = g["boom"]["length"]
        hub = g["boom"]["hub_z"]
        sign = 1.0 if dirc == "in" else -1.0      # 入口杆向 +x，出口杆向 -x
        com = stub + alen / 2.0                   # 整杆（含杆尾配重段）的质心离轴距离
        col_len = alen - (post_r + 0.02)          # 碰撞段留到接收柱前 2 cm，放平不互顶

        out.append(f'    <model name="parking_{gid}">\n'
                   '      <static>false</static>\n'
                   f'      <pose>{fmt_pose_xyz(px, py, road_z)}</pose>\n'
                   f'      <!-- {g["label"]}：机箱只画不碰（它压在分隔岛里，车永远不会开进'
                   '去，真撞上说明摆渡路线本身错了，那时该改的是路线不是加碰撞盒）；'
                   '碰撞件只有铰点柱、接收柱和杆身，与 tools/check_parking_layout.py '
                   '里 statics 表一一对应。杆的碰撞段止于接收柱前 2 cm，放平时不与柱顶'
                   '接触，免得被 ODE 顶起来抖。 -->\n'
                   '      <link name="housing">\n')
        out.append(inertial_box(40.0, hx, hy, hz, fmt_pose_xyz(-sign * (stub + hx / 2 + 0.06), 0, hz / 2)))
        cab_x = -sign * (stub + hx / 2 + 0.06)
        out.append(visual("cabinet", fmt_pose_xyz(cab_x, 0, hz / 2), box(hx, hy, hz), C_STEEL))
        out.append(visual("cabinet_door", fmt_pose_xyz(cab_x - sign * hx / 2 * 1.02, 0, hz / 2),
                          box(0.01, hy * 0.8, hz * 0.8), C_DARK))
        out.append(visual("lamp", fmt_pose_xyz(cab_x, 0, hz + 0.07), cyl(0.05, 0.10),
                          C_BOOM_RED if dirc == "in" else C_YELLOW))
        out.append(visual("pivot_post", fmt_pose_xyz(0, 0, post_h / 2), cyl(post_r, post_h), C_STEEL))
        out.append(visual("pivot_base", fmt_pose_xyz(0, 0, 0.03), cyl(post_r * 1.8, 0.06), C_DARK))
        out.append(collision("pivot_post_col", fmt_pose_xyz(0, 0, post_h / 2), cyl(post_r, post_h)))
        out.append(visual("hub", fmt_pose_xyz(0, 0, hub), cyl(post_r * 0.9, sec_h * 2.2), C_DARK))
        out.append("      </link>\n")
        out.append('      <joint name="housing_world" type="fixed">\n'
                   "        <parent>world</parent>\n"
                   "        <child>housing</child>\n"
                   "      </joint>\n")

        out.append('      <link name="boom">\n')
        out.append(f'        <pose>{fmt_pose_xyz(0, 0, hub)}</pose>\n')
        out.append(inertial_box(bar["mass_kg"], stub + alen, sec_w, sec_h,
                                 fmt_pose_xyz(sign * com, 0, 0)))
        # 杆身分段红白，视觉上是真道闸；整段一个碰撞盒（含尾段纯视觉）
        segs = 6
        total = stub + alen
        for i in range(segs):
            a = -stub + i * total / segs
            b = a + total / segs
            colors = C_BOOM_RED if i % 2 == 0 else C_BOOM_WHITE
            out.append(visual(f"boom_seg{i}", fmt_pose_xyz(sign * (a + b) / 2, 0, 0),
                              box(b - a, sec_w, sec_h), colors))
        out.append(visual("boom_tip", fmt_pose_xyz(sign * alen, 0, -0.01),
                          box(0.05, sec_w * 1.3, sec_h * 1.6), C_DARK))
        out.append(collision("boom_col", fmt_pose_xyz(sign * col_len / 2, 0, 0),
                              box(col_len, sec_w, sec_h)))
        out.append("      </link>\n")

        uplim = bar["open_angle"]
        out.append('      <joint name="boom_joint" type="revolute">\n'
                   "        <parent>housing</parent>\n"
                   "        <child>boom</child>\n"
                   "        <axis>\n"
                   f"          <xyz>{n(ax)} {n(ay)} {n(az)}</xyz>\n"
                   "          <use_parent_model_frame>true</use_parent_model_frame>\n"
                   f"          <limit><lower>{n(bar['closed_angle'])}</lower>"
                   f"<upper>{n(uplim)}</upper>"
                   f"<effort>{n(bar['effort'])}</effort>"
                   f"<velocity>{n(bar['velocity'])}</velocity></limit>\n"
                   f"          <dynamics><damping>{n(bar['damping'])}</damping>"
                   f"<friction>{n(bar['friction'])}</friction></dynamics>\n"
                   "        </axis>\n"
                   "      </joint>\n")

        out.append('      <plugin name="v2x_barrier" filename="libv2x_barrier.so">\n')
        lcx, lcy = g["loop"]["center"]
        llx, lly = g["loop"]["size"]
        for tag, val in (
            ("gate_id", gid),
            ("joint_name", "boom_joint"),
            ("boom_link", "boom"),
            ("cmd_topic", f"/parking/gate/{gid}/cmd"),
            ("state_topic", f"/parking/gate/{gid}/state"),
            ("fault_topic", pk["control"]["fault_topic"]),
            ("state_rate", n(bar["state_rate"])),
            ("open_angle", n(bar["open_angle"])),
            ("closed_angle", n(bar["closed_angle"])),
            ("angle_tolerance", n(bar["angle_tolerance"])),
            ("p_gain", n(bar["p_gain"])),
            ("d_gain", n(bar["d_gain"])),
            ("raise_timeout_s", n(bar["raise_timeout_s"])),
            ("lower_timeout_s", n(bar["lower_timeout_s"])),
            ("effort_limit", n(bar["effort"])),
            ("jam_effort_ratio", n(bar["jam_effort_ratio"])),
            ("gravity_comp", "true"),
            # 地感线圈：插件按 world 系的矩形做占用判定，把结果放进同一份
            # BarrierState（loop_occupied/vehicle_id），所以"杆下有没有车"这件事
            # 的权威在物理侧，不在 manager。
            ("loop_id", g["loop"]["id"]),
            ("loop_center", f'{n(lcx)} {n(lcy)} {n(road_z)}'),
            ("loop_size", f'{n(llx)} {n(lly)} 0.1'),
            ("vehicle_length", n(pk["vehicle"]["length"])),
            ("vehicle_width", n(pk["vehicle"]["width"])),
            ("vehicle_prefix", pk["shuttle"]["prefix"]),
            ("chassis_link", pk["shuttle"]["chassis_link"]),
        ):
            out.append(f"        <{tag}>{val}</{tag}>\n")
        out.append("      </plugin>\n")
        out.append("    </model>\n\n")

        # 接收柱：杆放平时的落点，独立静态模型（不放进闸机模型，免得被非静态连带）
        rx = px + sign * alen
        out.append(f'    <model name="parking_{gid}_receiver">\n'
                   '      <static>true</static>\n'
                   f'      <pose>{fmt_pose_xyz(rx, py, road_z)}</pose>\n'
                   '      <link name="link">\n')
        out.append(visual("post", fmt_pose_xyz(0, 0, post_h / 2), cyl(post_r, post_h), C_STEEL))
        out.append(visual("cap", fmt_pose_xyz(0, 0, post_h + 0.02),
                          box(sec_w * 1.6, sec_w * 1.6, 0.04), C_YELLOW))
        out.append(collision("post_col", fmt_pose_xyz(0, 0, post_h / 2), cyl(post_r, post_h)))
        out.append("      </link>\n    </model>\n\n")

    out.append(gen_shuttle_car(pk))
    out.append(f"    <!-- {END_MARK} -->\n")
    return "".join(out)


# ══════════════════════════════════════════════════════════════════════
def splice(world_text: str, block: str) -> tuple[str, str]:
    """把生成块插进 world；已有生成块则整块替换（幂等）。返回 (新文本, 说明)。"""
    start = world_text.find(BEGIN_MARK)
    end = world_text.find(END_MARK)
    if start >= 0 and end >= 0:
        # 从该行注释的 "<!--" 起，到 END 注释行末尾止
        line_start = world_text.rfind("\n", 0, world_text.rfind("\n", 0, start) + 1) + 1
        tail = world_text.find("-->", end)
        tail = world_text.find("\n", tail)
        new = world_text[:line_start] + block + world_text[tail + 1:]
        return new, "替换了已有的生成块"
    pos = world_text.find(ANCHOR)
    if pos < 0:
        raise SystemExit(f"找不到插入锚点：{ANCHOR!r}（world 结构变了，改脚本而不是硬插）")
    return world_text[:pos] + block + "\n" + world_text[pos:], "插到 Gazebo ROS 插件之前"


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--zones", default=os.path.join(
        here, os.pardir, "src", "v2x_demo", "config", "parking_zones.yaml"))
    ap.add_argument("--world", default=os.path.join(
        here, os.pardir, "src", "smart_car_sim", "worlds", "v2x_city_scene.world"))
    ap.add_argument("--print", dest="dry", action="store_true",
                    help="只打印生成结果与体检计数，不写 world")
    ap.add_argument("--model-prefix", default="parking_",
                    help="检查与既有模型重名的前缀，空串=检查全部新模型")
    args = ap.parse_args()

    with open(args.zones, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    block = gen(cfg)

    # 生成结果自己先过一遍 XML：SDF 里少一个闭标签，gzserver 只报 "XML Error" 不给行号
    probe = "<sdf version=\"1.6\"><world name=\"probe\">\n" + block + "\n</world></sdf>"
    ET.fromstring(probe)
    # XML 合法不等于 SDF 合法：<pose> 只认 3 个或 6 个数。拼串时把平移段和旋转段
    # 多数了一遍就会得到 8 个，gzserver 只报一句 "XML Error" 不给行号，轻则世界起不来，
    # 重则这块 pose 被截断读成前 6 个 —— 车牌朝后就变成了朝前，取证时才看得出来。
    bad = [p for p in re.findall(r'<pose>([^<]*)</pose>', block)
           if len(p.split()) not in (3, 6)]
    if bad:
        raise SystemExit("生成的 <pose> 参数个数不对（SDF 只接受 3 或 6 个）：%s"
                         % '; '.join('%r 是 %d 个' % (p, len(p.split())) for p in bad[:6]))
    names = re.findall(r'<model name="([^"]+)"', block)
    print(f"生成模型 {len(names)} 个：{', '.join(names)}")
    print(f"行数 {block.count(chr(10))}，字符 {len(block)}")

    if args.dry:
        print(block)
        return 0

    with open(args.world, encoding="utf-8") as fh:
        text = fh.read()
    for nm in names:
        if f'<model name="{nm}"' in text and BEGIN_MARK not in text:
            raise SystemExit(f"world 里已有同名模型 {nm}，且不是本脚本生成的块 —— 停手，别盖掉别人的东西")
    dup = [nm for nm in names if text.count(f'<model name="{nm}"') > 1]
    if dup:
        raise SystemExit(f"重名：{dup}")

    new_text, how = splice(text, block)
    # 写盘前先按语义验一遍：ET 只能保证"是合法 XML"，不能保证这些 model 真的
    # 挂在 <world> 下面。锚点要是落在某个 <plugin> 块里，gzserver 照样起世界，
    # 但停车场整块会被静默忽略 —— 那种"起来了但什么都没有"最难查。
    root = ET.fromstring(new_text)
    wel = root.find("world")
    if wel is None:
        raise SystemExit("world 里找不到 <world> 元素")
    got = {m.get("name") for m in wel.findall("model")}
    missing = [nm for nm in names if nm not in got]
    if missing:
        raise SystemExit(f"生成的模型没成为 <world> 的直接子节点：{missing}"
                         f"（插入锚点 {ANCHOR!r} 的位置不对，改锚点别硬插）")
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    bak = f"{args.world}.pre_parking_{stamp}.bak"
    # 同一秒内连跑两次会撞名：第二次的"备份"其实已是第一次改过的内容，
    # 于是真正的原件就没了。撞名就加序号，绝不覆盖已有备份。
    seq, exists = 1, os.path.exists(bak)
    while os.path.exists(bak):
        seq += 1
        bak = f"{args.world}.pre_parking_{stamp}_{seq}.bak"
    if exists:
        print(f"注意：{stamp} 这一秒已有备份，本次写到 ..._{seq}.bak")
    with open(bak, "w", encoding="utf-8", newline="") as fh:
        fh.write(text)
    with open(args.world, "w", encoding="utf-8", newline="") as fh:
        fh.write(new_text)
    ET.fromstring(new_text)          # 整份 world 再解析一次，写坏了当场知道
    print(f"{how}；原文件已备份到 {bak}")
    print(f"world 字节 {len(text)} → {len(new_text)}")
    print("注意：改的是 world，必须整栈重启才生效（gzserver 不会热加载 world）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
