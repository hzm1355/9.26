#!/usr/bin/env python3
"""parking_zones.yaml 的几何自洽体检。

这个文件同时是 world SDF 生成器与三个节点的几何真值来源，任何一处数字互相
矛盾，实机上就是"抬了杆但相机没拍到""落杆砸到车"这类难查的故障。所以这里把
整套关系逐条断言，其中卡口相机那条反推链与任务三标定 gate_j3_north 时同源：

    俯角 pitch  ->  水平距离 = (相机z - 车牌z)/tan(pitch)
                ->  抓拍点 = 相机xy + 水平距离·视线方向
                ->  抓拍点必须正好落在"车停在核验位时的车牌面"上
                ->  focus = 核验位 + near_radius·行进方向（车爬到核验位那一帧才触发）

带 --obj 时还会把 room_33 网格栅格化，确认占地内确实没有沙盘画好的建筑/景观。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys

import yaml

# 沙盘车道中心线（signal_drive_params.yaml）。场地里带碰撞的静态件必须远离它们。
# 这六条必须是**挪网之后**的真值（signal_drive_params.yaml:23 记录 4.6/-14.6 -> 1.5/-16.8、
# 16.8 -> 20.8 那次整网外移，:43 记北环/南环向东延长到 x=20.80；路线实读同文件 :68/:95）。
# 拿旧值判会把"场地离车流其实很远"读成"贴脸"，反过来也会把真侵入读成安全。
LANES = {
    "东环南行 x=20.8": ("v", 20.8),
    "中轴北行 x=1.8": ("v", 1.8),
    "RingA 中轴 x=-0.8": ("v", -0.8),
    "RingA 西环 x=-21.8": ("v", -21.8),
    "南环西行 y=-16.8": ("h", -16.8),
    "北环东行 y=1.5": ("h", 1.5),
}
LANE_CLEAR = 1.5
BOOM_Y = -11.05            # 两道闸机的杆轴（reset 段必须整条在它以南）

fails: list[str] = []
notes: list[str] = []


def ok(cond: bool, msg: str) -> bool:
    if not cond:
        fails.append(msg)
    return cond


def near(a: float, b: float, tol: float, msg: str) -> bool:
    return ok(abs(a - b) <= tol, f"{msg}：{a:.3f} 对 {b:.3f}（容差 {tol}）")


def fwd(theta: float) -> tuple[float, float]:
    return math.cos(theta), math.sin(theta)


def aabb(cx: float, cy: float, sx: float, sy: float, theta: float):
    ex = abs(sx / 2.0 * math.cos(theta)) + abs(sy / 2.0 * math.sin(theta))
    ey = abs(sx / 2.0 * math.sin(theta)) + abs(sy / 2.0 * math.cos(theta))
    return cx - ex, cx + ex, cy - ey, cy + ey


def in_box(x: float, y: float, box, m: float = 0.0) -> bool:
    return box[0] + m <= x <= box[1] - m and box[2] + m <= y <= box[3] - m


def dist_point_lane(x: float, y: float, r: float):
    out = []
    for name, (orient, v) in LANES.items():
        d = abs(x - v) if orient == "v" else abs(y - v)
        out.append((name, d - r))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml_path")
    ap.add_argument("--obj", default=None, help="room_33.obj 路径，附带做占地净空复核")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.yaml_path, encoding="utf-8"))["parking"]
    road_z, plate_z = cfg["road_z"], cfg["plate_z"]
    L = cfg["lot"]
    lot = (L["x0"], L["x1"], L["y0"], L["y1"])
    v = cfg["vehicle"]
    CAR_L, CAR_W = v["length"], v["width"]
    PA, TURN_R = v["plate_ahead"], math.hypot(v["length"], v["width"]) / 2.0

    print(f"场地 x {lot[0]}..{lot[1]} × y {lot[2]}..{lot[3]}，路面 z={road_z}，"
          f"车体 {CAR_L}×{CAR_W}，原地转扫掠半径 {TURN_R:.2f}")

    statics: list[tuple[str, float, float, float]] = []
    # 碰撞半径一律取自 yaml —— 体检必须量"将要生成的那件东西"，不是脚本自己想象的尺寸
    BAR = cfg["barrier"]
    POST_R = BAR["post_radius"]
    MAST_R = cfg["stripes"]["gantry"].get("mast_radius", 0.09)

    # ══ 1 道闸 / 卡口相机 / 地感 ══════════════════════════════════════
    routes = {
        "in": cfg["shuttle"]["entry_route"],
        "out": cfg["shuttle"]["exit_route"],
    }
    for g in cfg["gates"]:
        gid, dirc = g["id"], g["direction"]
        px, py = g["boom"]["pivot"]
        alen, hub_z, axis = g["boom"]["length"], g["boom"]["hub_z"], g["boom"]["axis"]
        cam, loop = g["camera"], g["loop"]
        lane = cfg["stripes"]["lane_in" if dirc == "in" else "lane_out"]
        heading = math.pi / 2 if dirc == "in" else -math.pi / 2
        fx, fy = fwd(heading)
        qx, qy = g["queue_point"]
        sx, sy = g["staging_point"]

        ok(abs(py - lane["y1"]) < 1e-9, f"{gid}: 杆轴 y={py} 应贴在车道带北缘 {lane['y1']}")
        if dirc == "in":
            near(px, lane["x0"], 1e-9, f"{gid}: 铰点 x 应贴车道西缘")
            near(px + alen, lane["x1"], 1e-9, f"{gid}: 杆长应正好横越到车道东缘")
            ok(axis == [0, -1, 0], f"{gid}: 入口杆轴应为 [0,-1,0]（+角=抬起）")
        else:
            near(px, lane["x1"], 1e-9, f"{gid}: 铰点 x 应贴车道东缘")
            near(px - alen, lane["x0"], 1e-9, f"{gid}: 杆长应正好横越到车道西缘")
            ok(axis == [0, 1, 0], f"{gid}: 出口杆轴应为 [0,1,0]（+角=抬起）")
        ok(road_z + hub_z + alen < road_z + 2.6,
           f"{gid}: 抬到位杆顶 z={road_z + hub_z + alen:.2f} 超出 2.6 m，横臂会伸到路面外")
        statics += [(f"{gid} 铰点柱", px, py, POST_R),
                    (f"{gid} 接收柱", px + (alen if dirc == "in" else -alen), py, POST_R)]
        # 杆放平必须真拦得住车：下缘要在车身碰撞盒顶面之下（留 10 cm 咬合），
        # 又要在牌面之上，否则抬落过程杆从牌上刮过去。
        sec_h = BAR["section"][1]
        boom_lo, boom_hi = hub_z - sec_h / 2.0, hub_z + sec_h / 2.0
        ok(boom_lo < v["body_top"] - 0.10,
           f"{gid}: 杆放平时下缘 z={boom_lo:.2f} 高于车身顶 {v['body_top'] - 0.10:.2f}，"
           f"物理碰撞是假的（杆从车顶掠过）")
        ok(boom_hi > v["plate_top"] + 0.05,
           f"{gid}: 杆放平时上缘 z={boom_hi:.2f} 低于牌顶 {v['plate_top']:.2f}，抬落要刮牌")
        ok(BAR["post_height"] + 0.02 < boom_lo,
           f"{gid}: 柱顶 {BAR['post_height']:.2f} 与杆下缘 {boom_lo:.2f} 接触，"
           f"放平时 ODE 会把杆顶起来抖")

        # 相机反推链
        cx, cy, cz, _, pitch, yaw = cam["pose"]
        horiz = (cz - plate_z) / math.tan(pitch)
        vx, vy = fwd(yaw)
        capx, capy = cx + horiz * vx, cy + horiz * vy
        near(capx, cam["capture_point"][0], 0.02, f"{gid}: 由 pose/pitch 反推的抓拍点 x")
        near(capy, cam["capture_point"][1], 0.02, f"{gid}: 由 pose/pitch 反推的抓拍点 y")
        # 抓拍点必须正好是"车停在核验位时的车牌面"
        sgn = 1.0 if dirc == "in" else -1.0      # 入口读前牌、出口读后牌
        near(qx + sgn * PA * fx, cam["capture_point"][0], 0.02,
             f"{gid}: 核验位上的{'车头' if dirc=='in' else '车尾'}牌 x 未对齐抓拍点")
        near(qy + sgn * PA * fy, cam["capture_point"][1], 0.02,
             f"{gid}: 核验位上的{'车头' if dirc=='in' else '车尾'}牌 y 未对齐抓拍点")
        near_r = cam["near_radius"]
        # 武装条件是"车停稳后离 focus 的距离 <= near_radius"，所以这里只能按**带余量的
        # 不等式**查。上一版查的是 focus = 核验位 + near·行进向 这个等式，等于把停稳点
        # 正好摆在阈值上：摆渡车到位半径 0.12 m、首轮实测停在 1.10 m 处（>1.0），
        # 于是核验位停 30 s 一次都不抓拍、整栈重启后仍然零通行。
        d_stop = math.hypot(qx - cam["focus"][0], qy - cam["focus"][1])
        arrive = float(cfg["shuttle"]["arrive_radius"])
        ok(d_stop + arrive + 0.10 <= near_r,
           f"{gid}: 停稳点离 focus {d_stop:.2f} m，加上到位残差 {arrive:.2f} m 仍够不到 "
           f"near_radius {near_r}，可能永不武装（插件条件 d<=near，见 v2x_gate_plugin.cc）")
        far = cam["far_radius"]
        ok(far > near_r, f"{gid}: far_radius 必须大于 near_radius")
        # 等帧窗口必须盖住渲染线程的真实出帧间隔。实测（tools/cam_rate.py 20 s 窗，
        # 全场景四路 1080p 卡口相机共用一个渲染线程）：间隔中位 0.20 s、90 分位
        # 0.6~0.7 s、最大 1.6 s，RTF 0.950 所以仿真秒与墙钟秒同量级。
        fw = cam.get("frame_wait", 0.25)
        ok(fw >= 2.4, f"{gid}: frame_wait={fw} 盖不住实测最大出帧间隔 1.6 s（要求 ≥1.5 倍）")
        # 停车场车道是断头路：没成功过杆时，车离 focus 最远只 1.66 m，越不过 far 2.5。
        # 所以"抓拍超时"必须由插件解除武装，否则 armed 永久留着 true，这道闸再也不触发。
        ok(bool(cam.get("rearm_on_fail", False)),
           f"{gid}: 缺 rearm_on_fail：核验位是停着等的，一次超时就永不重新触发"
           f"（退到等待位也只有 1.59 m < far {cam['far_radius']}）")
        ok(abs(cam.get("update_rate", 10) - 10) < 1e-9 and cam["width"] == 1920
           and cam["height"] == 1080,
           f"{gid}: 分辨率/帧率与任务三标定的识别窗口不同源，牌面像素尺寸要重新标定")
        # 等待位必须在 near 圈**外**且留出到位残差，否则车还没爬到核验位就已经武装，
        # 核验位那一帧不再产生上升沿（插件只在 !armed && d<=near 时触发）
        d_stage = math.hypot(sx - cam["focus"][0], sy - cam["focus"][1])
        ok(d_stage > near_r + arrive,
           f"{gid}: 等待位离 focus 只有 {d_stage:.2f} m，near={near_r}+残差 {arrive:.2f}，"
           f"进场/出场那一帧不会再有武装上升沿")
        # 抓拍之后车必须开出 far 圈，插件才会重新武装（否则下一轮零抓拍）
        pts = routes[dirc]
        qi = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - qx, pts[i][1] - qy))
        after = pts[qi:] + cfg["shuttle"]["reset_route"] + pts[: qi + 1]
        dmax = max(math.hypot(x - cam["focus"][0], y - cam["focus"][1]) for x, y, _ in after)
        ok(dmax > far, f"{gid}: 抓拍后离 focus 最远只有 {dmax:.2f} m ≤ far_radius {far}，永不重新武装")
        # 相机吊在车道中心线正上方，且杆件整体在镜头平面以北
        near(cx, lane["center_x"], 1e-9, f"{gid}: 相机 x 应在车道中心线上（横向零偏置）")
        # 话题名是 gazebo_ros_camera 的硬规则：<namespace>/<camera_name>/image_raw，
        # plugin 里的 ~/image_raw remapping 不参与（实测过）。上一版把 namespace 写成
        # /parking/gate_in、camera_name 又写 parking_gate_in，结果话题多出一层。
        ok(cam["gate_id"] == gid,
           f"{gid}: 相机 gate_id={cam['gate_id']} 与闸机 id 不在同一命名空间，"
           f"ParkingPass.gate_id 与 BarrierState.gate_id 对不上账")
        want_topic = cam["namespace"].rstrip("/") + "/" + cam["gate_id"] + "/image_raw"
        ok(cam["image_topic"] == want_topic,
           f"{gid}: image_topic={cam['image_topic']} ≠ {want_topic}")
        ok(in_box(cx, cy, lot, 0.05), f"{gid}: 相机超出场地")
        ok(cy <= cfg["stripes"]["gantry"]["y"] + 1e-9,
           f"{gid}: 相机不得比门架横梁更靠南，否则横梁占满画面")

        # 地感
        lcx, lcy = loop["center"]
        near(lcx, lane["center_x"], 1e-9, f"{gid}: 地感中心 x 应在车道中心线上")
        near(loop["size"][0], lane["x1"] - lane["x0"], 1e-9, f"{gid}: 地感宽度应等于车道宽")
        ok(in_box(lcx, lcy, lot, 0.05), f"{gid}: 地感线圈超出场地")
        # 插件里的占用框是车体**外接方框**∪线圈：半边长 (长+宽)/2 + 半线圈尺寸。
        # 体检必须按这个同一个框量，不能按真旋转矩形量 —— 差出来的 0.4 m 正是
        # "地感到底什么时候检出"的分界，而它直接决定入口会不会提前落杆砸车。
        hy = (CAR_L + CAR_W) / 2.0 + loop["size"][1] / 2.0
        arr = cfg["shuttle"]["arrive_radius"]
        # 占用框要盖住"杆下危险区"，两边各留 arrive_radius：
        #   检出必须早于车头碰杆，否则 manager 只能靠 hold_time_s 到点落杆，实测就把杆
        #     落在了正在爬的车头上（2026-10-12 那次：核验位到检出点 0.26 m 要爬 1.3 s，
        #     比 hold 1.5 s 慢，clear_s 永远 -1，车随后顶在放平的杆上 deadlock）；
        #   释放必须晚于整条车尾过杆，否则落杆时车身还在杆下。
        # 用行进坐标 t = 行进方向·y 写，进场/出场就是同一个式子（进场 t=y，出场 t=-y）。
        sgn = 1.0 if dirc == "in" else -1.0
        tp, tq, ts = sgn * py, sgn * qy, sgn * sy          # 杆轴 / 核验位 / 等待位
        lt = sgn * lcy                                     # 占用框中心（行进坐标）
        detect, release = lt - hy, lt + hy                 # 先在这一帧占用、越过这侧才释放
        ok(detect <= tp - CAR_L / 2.0 - arr,
           f"{gid}: 地感检出太晚（行进坐标 {detect:.2f} > 碰杆点 "
           f"{tp - CAR_L / 2.0:.2f} - arrive {arr}），车头还没被检出就可能被落杆砸到")
        ok(release >= tp + CAR_L / 2.0 + arr,
           f"{gid}: 地感释放太早（{release:.2f} < 整车过杆点 {tp + CAR_L / 2.0:.2f}"
           f" + arrive {arr}），释放那一帧车身还在杆下")
        # 等待位必须在框外，否则车一进场就是"已占用"，manager 等不到上升沿
        ok(abs(ts - lt) > hy,
           f"{gid}: 等待位（行进坐标 {ts:.2f}）落在占用框内，进场前就没有占用沿可等")
        in_q = abs(tq - lt) <= hy
        print('  %s 核验位%s压在占用框内（框 [%.2f, %.2f]，核验位 %.2f，'
              '检出余量 %.2f m，释放余量 %.2f m）' % (
                  gid, '' if in_q else '没有', lt - hy, lt + hy, tq,
                  (tp - CAR_L / 2.0 - arr) - detect,
                  release - (tp + CAR_L / 2.0 + arr)))
        # 核验位的整条车体必须在**划线场地**内（它夹在闸机设施中间）；等待位/终点按
        # **可行驶带**判。口径差别是必须的：lot 只是 gen_parking_world 铺那块沥青贴片
        # 的范围，而贴片纯 visual 无碰撞（生成器硬规矩 1），真正承力的是 room_33 的
        # 沙盘等效盒体，它比贴片大得多。car_3 等待位南移到 -13.95 后车体南缘 -14.75
        # 已出贴片 0.95 m，实测 z=6.69871、pitch≈0、5 s 不动（/tmp/pk185_main.log），
        # 落地障碍高度图该矩形 0 格（tools/lot_clearance.py --rect=11.60,12.40,
        # -14.75,-13.15）。带界取"整条车体不得压进南环西行车流走廊"（-16.8 + 半车宽），
        # 再往南就会把车流顶住。车头距闸杆仍留 0.18~0.45 m。
        y0_wait = LANES["南环西行 y=-16.8"][1] + CAR_W / 2.0
        for tag, (bx, by) in (("核验位", (qx, qy)), ("等待/终点", (sx, sy))):
            y0 = lot[2] if tag == "核验位" else y0_wait
            body = aabb(bx, by, CAR_L, CAR_W, heading)
            ok(in_box(bx, by, (lot[0], lot[1], y0, lot[3]), 0.02)
               and body[2] >= y0 and body[3] <= lot[3]
               and body[0] >= lot[0] and body[1] <= lot[1],
               f"{gid}: {tag} 的车体 {tuple(round(b, 2) for b in body)} 超出可行驶带 "
               f"(y {y0:.2f}..{lot[3]:.2f})")
        # 入口车从南往北停，车头是它的北缘；出口车往南停，车头是南缘
        gap = py - (qy + CAR_L / 2.0) if dirc == "in" else (qy - CAR_L / 2.0) - py
        ok(0.18 <= gap <= 0.45, f"{gid}: 核验位车头距闸杆 {gap:.2f} m，应在 0.18~0.45 m")

    # ══ 2 车位 ═══════════════════════════════════════════════════════
    for s in cfg["stalls"]:
        cx, cy = s["center"]
        ok(in_box(cx, cy, lot, 0.05), f"车位 {s['id']} 超出场地")
        sb = aabb(cx, cy, s["size"][0], s["size"][1], 0.0)
        cb = aabb(cx, cy, CAR_L, CAR_W, s["heading"])
        ok(cb[0] >= sb[0] - 1e-9 and cb[1] <= sb[1] + 1e-9
           and cb[2] >= sb[2] - 1e-9 and cb[3] <= sb[3] + 1e-9,
           f"车位 {s['id']} 放不下按 heading 停放的车：车 {tuple(round(b,2) for b in cb)} vs 位 {tuple(round(b,2) for b in sb)}")

    # ══ 3 门架立杆 / 岗亭 ════════════════════════════════════════════
    gan = cfg["stripes"]["gantry"]
    bw, bh = cfg["stripes"]["median"].get("booth_size", [1.0, 1.0])
    for i, (mx, my) in enumerate(gan["masts"]):
        statics.append((f"门架立杆{i+1}", mx, my, MAST_R))
        ok(in_box(mx, my, lot, 0.05), f"门架立杆{i+1} 超出场地")
        near(my, gan["y"], 1e-9, f"门架立杆{i+1} 的 y 应与横梁同线")
        for tag in ("lane_in", "lane_out"):
            ls = cfg["stripes"][tag]
            # 立杆可以贴着车道边，但碰撞圆必须让开行走扫掠带（中心线 ±半车宽）
            d = abs(mx - ls["center_x"])
            ok(d > CAR_W / 2.0 + MAST_R + 0.10,
               f"门架立杆{i+1} 距 {tag} 中心线只有 {d:.2f} m，车体会刮杆")
    bx, by = cfg["stripes"]["median"]["booth"]
    statics.append(("岗亭", bx, by, math.hypot(bw, bh) / 2.0))
    med = cfg["stripes"]["median"]
    ok(med["x0"] <= bx <= med["x1"] and med["y0"] <= by <= med["y1"], "岗亭不在中央分隔岛内")

    # ══ 4 摆渡路线 ═══════════════════════════════════════════════════
    sh = cfg["shuttle"]
    all_routes = {"entry": sh["entry_route"], "exit": sh["exit_route"], "reset": sh["reset_route"]}
    for rname, pts in all_routes.items():
        for i, (x, y, th) in enumerate(pts):
            ok(in_box(x, y, lot, 0.02), f"{rname}[{i}] 点 ({x},{y}) 超出场地")
            turn = i and abs(x - pts[i - 1][0]) < 1e-9 and abs(y - pts[i - 1][1]) < 1e-9
            if i and not turn:
                ok(abs(x - pts[i - 1][0]) < 1e-9 or abs(y - pts[i - 1][1]) < 1e-9,
                   f"{rname}[{i}] 不是直线段：从 {pts[i-1][:2]} 到 {(x, y)}")
                # 直行段：横向半宽内不得有碰撞件
                for nm, sx2, sy2, r in statics:
                    if abs(sx2 - x) < 1e-9 or abs(sy2 - y) < 1e-9:
                        d = abs(sx2 - x) if abs(sy2 - y) < 1e-9 else abs(sy2 - y)
                        ok(d > CAR_W / 2.0 + r + 0.10,
                           f"{rname}[{i}] 直行段离 {nm} 太近（横向 {d:.2f} m）")
            if turn:
                ok(in_box(x, y, lot, TURN_R + 0.02),
                   f"{rname}[{i}] 原地转的扫掠圆 r={TURN_R:.2f} 超出场地")
                for nm, sx2, sy2, r in statics:
                    d = math.hypot(x - sx2, y - sy2)
                    ok(d > TURN_R + r + 0.10,
                       f"{rname}[{i}] 原地转会扫到 {nm}（距离 {d:.2f} < {TURN_R + r + 0.10:.2f}）")
    ok(all_routes["reset"][-1][:2] == all_routes["entry"][0][:2],
       "reset 终点没接上 entry 起点，进出场循环跑不起来")
    ok(all_routes["exit"][-1][:2] == all_routes["reset"][0][:2],
       "exit 终点没接上 reset 起点")
    for x, y, th in all_routes["reset"]:
        ok(y < BOOM_Y, f"reset 经过 y={y} ≥ 杆轴 {BOOM_Y}，复位会越杆")
    ok(sh["arrive_radius"] < 0.5, "arrive_radius 过大：车会在闸杆前停过头")

    # ══ 5 车道净空 ═══════════════════════════════════════════════════
    for nm, x, y, r in statics:
        for lane, clear in dist_point_lane(x, y, r):
            if clear < LANE_CLEAR and clear < 20:
                ok(False, f"{nm} 距 {lane} 只有 {clear:.2f} m（含碰撞半径），要求 ≥{LANE_CLEAR}")
    print(f"静态碰撞件 {len(statics)} 件：{', '.join(n for n, *_ in statics)}")

    # ══ 6 占地净空（可选，栅格化 room_33 网格）═══════════════════════
    if args.obj:
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "survey_parking_footprint.py")
        r = subprocess.run([sys.executable, script, args.obj,
                            "--x0", str(lot[0]), "--x1", str(lot[1]),
                            "--y0", str(lot[2]), "--y1", str(lot[3]),
                            "--cell", "0.25", "--pitch", "0.3"],
                           capture_output=True, text=True)
        m = re.search(r"受阻格子 (\d+) / (\d+)", r.stdout)
        if m is None:
            ok(False, f"净空复核没跑出结果：{r.stdout[:200]}{r.stderr[:200]}")
        else:
            blocked, cells = int(m.group(1)), int(m.group(2))
            print(f"占地净空：{cells} 格 0.25 m 栅格，受阻 {blocked} 格")
            ok(blocked == 0, f"占地内有 {blocked}/{cells} 格存在路面以上 0.3 m 的沙盘几何")

    print()
    for n in notes:
        print("  · " + n)
    if fails:
        print(f"体检未通过，共 {len(fails)} 处：")
        for f in fails:
            print("  ✗ " + f)
        return 1
    print("体检通过：杆轴/相机反推链/地感防砸/路线扫掠/车道净空全部自洽")
    return 0


if __name__ == "__main__":
    sys.exit(main())
