#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V2X 场景生成器（协议 §3 场景建模的配置化落地）

用法:
  python3 scene_builder.py <v2x_scene.yaml> <out.world> <out_devices.yaml>

读取场景唯一配置源 v2x_scene.yaml，生成：
  1) Gazebo world：道路拓扑/多车道划线/斑马线/高架/停车场/ETC 门架/RSU/
     信号灯/绿化/学校与楼宇，全部由配置驱动；
  2) 设备坐标表 scene_devices.yaml：路口、路段、车道、斑马线、信号灯、
     ETC 门架、RSU、道闸、高架的世界坐标，供信号灯控制/V2X 通信等模块引用。
"""
import math
import random
import re
import sys

import yaml


# ───────────────────────── 基础工具 ─────────────────────────
def fnum(x):
    s = f'{float(x):.4f}'.rstrip('0').rstrip('.')
    return '0' if s in ('', '-0') else s


def vec(c):
    return f'{fnum(c[0])} {fnum(c[1])} {fnum(c[2])} 1'


def norm(dx, dy):
    l = math.hypot(dx, dy)
    return dx / l, dy / l


class Emitter:
    def __init__(self):
        self.parts = []
        self.count = 0

    def add(self, s):
        self.parts.append(s)
        self.count += s.count('<model name=')
        return s

    def text(self):
        return '\n'.join(self.parts)


# ───────────────────────── SDF 基元 ─────────────────────────
def box_model(mid, x, y, z, sx, sy, sz, color, with_collision=False, yaw=0.0):
    col = ''
    if with_collision:
        col = ('      <collision name="c"><geometry>'
               f'<box><size>{fnum(sx)} {fnum(sy)} {fnum(sz)}</size></box>'
               '</geometry>'
               '<surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>'
               '</collision>\n')
    return (f'  <!-- {mid} -->\n'
            f'  <model name="{mid}">\n'
            f'    <static>true</static>\n'
            f'    <link name="link">\n'
            f'      <pose>{fnum(x)} {fnum(y)} {fnum(z)} 0 0 {fnum(yaw)}</pose>\n'
            f'{col}'
            f'      <visual name="v">\n'
            f'        <geometry><box><size>{fnum(sx)} {fnum(sy)} {fnum(sz)}</size></box></geometry>\n'
            f'        <material><ambient>{vec(color)}</ambient>'
            f'<diffuse>{vec(color)}</diffuse></material>\n'
            f'      </visual>\n'
            f'    </link>\n'
            f'  </model>\n')


def cyl_model(mid, x, y, z, r, h, color, with_collision=False):
    col = ''
    if with_collision:
        col = ('      <collision name="c"><geometry>'
               f'<cylinder><radius>{fnum(r)}</radius><length>{fnum(h)}</length></cylinder>'
               '</geometry>'
               '<surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>'
               '</collision>\n')
    return (f'  <!-- {mid} -->\n'
            f'  <model name="{mid}">\n'
            f'    <static>true</static>\n'
            f'    <link name="link">\n'
            f'      <pose>{fnum(x)} {fnum(y)} {fnum(z)} 0 0 0</pose>\n'
            f'{col}'
            f'      <visual name="v">\n'
            f'        <geometry><cylinder><radius>{fnum(r)}</radius>'
            f'<length>{fnum(h)}</length></cylinder></geometry>\n'
            f'        <material><ambient>{vec(color)}</ambient>'
            f'<diffuse>{vec(color)}</diffuse></material>\n'
            f'      </visual>\n'
            f'    </link>\n'
            f'  </model>\n')


def sphere_model(mid, x, y, z, r, color):
    return (f'  <!-- {mid} -->\n'
            f'  <model name="{mid}">\n'
            f'    <static>true</static>\n'
            f'    <link name="link">\n'
            f'      <pose>{fnum(x)} {fnum(y)} {fnum(z)} 0 0 0</pose>\n'
            f'      <visual name="v">\n'
            f'        <geometry><sphere><radius>{fnum(r)}</radius></sphere></geometry>\n'
            f'        <material><ambient>{vec(color)}</ambient>'
            f'<diffuse>{vec(color)}</diffuse></material>\n'
            f'      </visual>\n'
            f'    </link>\n'
            f'  </model>\n')


def include_model(mid, uri, x, y, z, yaw):
    return (f'  <!-- {mid} -->\n'
            f'  <include>\n'
            f'    <name>{mid}</name>\n'
            f'    <uri>{uri}</uri>\n'
            f'    <pose>{fnum(x)} {fnum(y)} {fnum(z)} 0 0 {fnum(yaw)}</pose>\n'
            f'  </include>\n')


def traffic_light_model(mid, x, y, yaw, state_topic):
    """内联生成一盏可由 ROS 点亮的信号灯（灯球 emissive 由 v2x_traffic_light 插件驱动）。
    每灯单独内联而非 include，以便按信号组注入不同的 <state_topic> 实现多路口联动。"""
    return (f'  <!-- 信号灯 {mid} (组话题 {state_topic}) -->\n'
            f'  <model name="{mid}">\n'
            f'    <static>true</static>\n'
            f'    <pose>{fnum(x)} {fnum(y)} 0 0 0 {fnum(yaw)}</pose>\n'
            f'    <link name="link">\n'
            f'      <visual name="pole">\n'
            f'        <pose>0 0 1.2 0 0 0</pose>\n'
            f'        <geometry><cylinder><radius>0.06</radius><length>2.4</length></cylinder></geometry>\n'
            f'        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>\n'
            f'      </visual>\n'
            f'      <collision name="pole_c">\n'
            f'        <pose>0 0 1.2 0 0 0</pose>\n'
            f'        <geometry><cylinder><radius>0.06</radius><length>2.4</length></cylinder></geometry>\n'
            f'      </collision>\n'
            f'      <visual name="housing">\n'
            f'        <pose>0 0 2.5 0 0 0</pose>\n'
            f'        <geometry><box><size>0.18 0.30 0.90</size></box></geometry>\n'
            f'        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse></material>\n'
            f'      </visual>\n'
            f'      <visual name="red">\n'
            f'        <pose>0.10 0 2.78 0 0 0</pose>\n'
            f'        <geometry><sphere><radius>0.09</radius></sphere></geometry>\n'
            f'        <material><ambient>0.45 0 0 1</ambient><diffuse>0.5 0 0 1</diffuse><emissive>0.9 0.05 0.05 1</emissive></material>\n'
            f'      </visual>\n'
            f'      <visual name="yellow">\n'
            f'        <pose>0.10 0 2.50 0 0 0</pose>\n'
            f'        <geometry><sphere><radius>0.09</radius></sphere></geometry>\n'
            f'        <material><ambient>0.45 0.36 0 1</ambient><diffuse>0.5 0.4 0 1</diffuse></material>\n'
            f'      </visual>\n'
            f'      <visual name="green">\n'
            f'        <pose>0.10 0 2.22 0 0 0</pose>\n'
            f'        <geometry><sphere><radius>0.09</radius></sphere></geometry>\n'
            f'        <material><ambient>0 0.45 0 1</ambient><diffuse>0 0.5 0 1</diffuse></material>\n'
            f'      </visual>\n'
            f'    </link>\n'
            f'    <plugin name="v2x_traffic_light" filename="libv2x_traffic_light.so">\n'
            f'      <state_topic>{state_topic}</state_topic>\n'
            f'      <link_name>link</link_name>\n'
            f'    </plugin>\n'
            f'  </model>\n')



def camera_model(mid, width, height, hfov, update_rate):
    """可移动场景相机：kinematic 模型 + gazebo_ros_camera，发布 /<mid>/cam/image_raw。
    界面通过 /set_entity_state 设置其位姿实现"实景3D"轨道视角。
    注意：kinematic/gravity/inertial 必须在 <link> 内（model 级 kinematic 会被忽略，
    导致相机自由落体）；<always_on> 必须为 1，否则 headless gzserver 不更新传感器。"""
    return (f'  <!-- 场景相机 {mid}（界面实景3D 视频源） -->\n'
            f'  <model name="{mid}">\n'
            f'    <static>false</static>\n'
            f'    <pose>0 -40 30 0 0 0</pose>\n'
            f'    <link name="link">\n'
            f'      <kinematic>true</kinematic>\n'
            f'      <gravity>false</gravity>\n'
            f'      <inertial><mass>0.1</mass>'
            f'<inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>\n'
            f'      <visual name="body">\n'
            f'        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>\n'
            f'        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>\n'
            f'      </visual>\n'
            f'      <sensor name="cam" type="camera">\n'
            f'        <always_on>1</always_on>\n'
            f'        <camera>\n'
            f'          <horizontal_fov>{fnum(hfov)}</horizontal_fov>\n'
            f'          <image><width>{width}</width><height>{height}</height></image>\n'
            f'          <clip><near>0.1</near><far>500</far></clip>\n'
            f'        </camera>\n'
            f'        <update_rate>{fnum(update_rate)}</update_rate>\n'
            f'        <plugin name="cam_plugin" filename="libgazebo_ros_camera.so">\n'
            f'          <ros>\n'
            f'            <namespace>/{mid}</namespace>\n'
            f'          </ros>\n'
            f'        </plugin>\n'
            f'      </sensor>\n'
            f'    </link>\n'
            f'  </model>\n')


def build_scene_camera(cfg, E, dev):
    cam = cfg.get('scene_camera')
    if not cam:
        return None
    mid = cam.get('id', 'v2x_scene_cam')
    E.add(camera_model(mid, int(cam.get('width', 960)), int(cam.get('height', 600)),
                       float(cam.get('hfov', 1.15)), float(cam.get('update_rate', 25))))
    dev['scene_camera'] = dict(id=mid, image_topic=f'/{mid}/cam/image_raw',
                               width=int(cam.get('width', 960)), height=int(cam.get('height', 600)),
                               default=cam.get('default', {}))
    return mid


HEAD = '''<?xml version="1.0" ?>
<!-- ============================================================================
  {name} —— 由 scene_builder.py 从 v2x_scene.yaml 自动生成，勿手改。
  元素：道路拓扑(多车道) / 人行横道 / 高架 / 停车场出入口 / ETC 门架 / RSU /
        信号灯 / 绿化 / 学校与楼宇 / 激光锚点。设备坐标见 scene_devices.yaml
============================================================================ -->
<sdf version="1.6">
  <world name="{name}">

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>1</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>{sun}</direction>
    </light>

    <!-- 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{gx} {gy}</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{gx} {gy}</size></plane></geometry>
          <material><ambient>{gcol}</ambient><diffuse>{gcol}</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 区域触发底座位姿源：发布 /model_states（全模型位姿+速度） -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>

'''

TAIL = '''
  </world>
</sdf>
'''


# ───────────────────────── 部件生成 ─────────────────────────
def edge_half(e):
    return e['lanes_per_dir'] * e['lane_width'] + e['median'] / 2.0 + e['shoulder']


def build_roads(cfg, E, dev):
    nodes = cfg['topology']['nodes']
    edges = cfg['topology']['edges']
    asphalt = cfg['global']['asphalt']['color']
    yellow = cfg['global']['yellow']
    white = [0.92, 0.92, 0.92]
    frames = {}
    surf = {}
    # 每个节点的交叉块半宽 = 该节点所有邻边 half 的最大值（仅 cross 节点使用）。
    # 路口采用“边路面 + 交叉块”互不重叠的铺法：多层路面 box 在路口堆叠
    # 会造成接触求解异常（实测：车进入 4 层重叠区后 ODE 冻结死寂）。
    chalf = {}
    for nid in nodes:
        hs = [edge_half(e) for e in edges if nid in (e['a'], e['b'])]
        chalf[nid] = max(hs) if hs else 0.0
    for e in edges:
        ax, ay = nodes[e['a']]['pose'][:2]
        bx, by = nodes[e['b']]['pose'][:2]
        L = math.hypot(bx - ax, by - ay)
        ux, uy = norm(bx - ax, by - ay)
        vx, vy = -uy, ux
        # 路面统一 z：边路面与交叉块互不重叠，无需分层防 z-fighting
        zbase = 0.010
        ztop = zbase + 0.02
        half = edge_half(e)
        yaw = math.atan2(uy, ux)
        frames[e['id']] = dict(ax=ax, ay=ay, bx=bx, by=by, L=L, ux=ux, uy=uy,
                                vx=vx, vy=vy, cx=(ax + bx) / 2, cy=(ay + by) / 2,
                                yaw=yaw, half=half, ztop=ztop)
        surf[e['id']] = ztop
        # 边路面（带碰撞，支撑车轮）：cross 端让出交叉块，endpoint 端铺到端点
        ca = chalf[e['a']] if nodes[e['a']]['type'] == 'cross' else 0.0
        cb = chalf[e['b']] if nodes[e['b']]['type'] == 'cross' else 0.0
        Lr = max(L - ca - cb, 0.1)
        E.add(box_model(f'road_{e["id"]}',
                        ax + ux * (ca + Lr / 2), ay + uy * (ca + Lr / 2), zbase + 0.01,
                        Lr, half * 2, 0.02, asphalt, with_collision=True, yaw=yaw))
        # 各端 break 区（虚线/边线避让路口）
        br_a = 13.5 if nodes[e['a']]['type'] == 'cross' else 3.0
        br_b = 13.5 if nodes[e['b']]['type'] == 'cross' else 3.0
        frames[e['id']]['br_a'], frames[e['id']]['br_b'] = br_a, br_b
        lz = ztop + 0.002                    # 线标浮层
        # 中央双黄线
        for sgn in (1, -1):
            px = (ax + bx) / 2 + vx * sgn * 0.1
            py = (ay + by) / 2 + vy * sgn * 0.1
            E.add(box_model(f'median_{e["id"]}_{"p" if sgn > 0 else "m"}',
                            px, py, lz, max(L - br_a - br_b, 0.1), 0.08, 0.004,
                            yellow, yaw=yaw))
        # 边缘白实线（两条）
        for sgn in (1, -1):
            px = (ax + bx) / 2 + vx * sgn * (half - 0.075)
            py = (ay + by) / 2 + vy * sgn * (half - 0.075)
            E.add(box_model(f'edge_{e["id"]}_{"p" if sgn > 0 else "m"}',
                            px, py, lz, max(L - br_a - br_b, 0.1), 0.15, 0.004,
                            white, yaw=yaw))
        # 车道白虚线（每向 lanes_per_dir-1 条：段长3m 周期9m）
        for k in range(1, e['lanes_per_dir']):
            for sgn in (1, -1):
                s = sgn * (e['median'] / 2 + k * e['lane_width'])
                t = -L / 2 + br_a + 1.0
                j = 0
                while t + 3.0 <= L / 2 - br_b:
                    px = (ax + bx) / 2 + ux * (t + 1.5) + vx * s
                    py = (ay + by) / 2 + uy * (t + 1.5) + vy * s
                    E.add(box_model(f'dash_{e["id"]}_{k}_{"p" if sgn > 0 else "m"}_{j}',
                                    px, py, lz, 3.0, 0.12, 0.004, white, yaw=yaw))
                    t += 9.0
                    j += 1
        dev['road_segments'][e['id']] = dict(
            from_node=e['a'], to_node=e['b'], length=round(L, 2),
            lanes_per_dir=e['lanes_per_dir'], lane_width=e['lane_width'],
            total_width=round(half * 2, 2), speed_limit=e.get('speed_limit'),
            label=e.get('label', e['id']))
    # 交叉块（每个 cross 节点一块，与边路面无重叠、无缝对接）
    for nid, n in nodes.items():
        if n['type'] != 'cross':
            continue
        ch = chalf[nid]
        nx, ny = n['pose'][:2]
        E.add(box_model(f'junction_{nid}', nx, ny, zbase + 0.01,
                        ch * 2, ch * 2, 0.02, asphalt, with_collision=True))
        dev.setdefault('intersections_detail', []).append(dict(
            id=nid, pose=[nx, ny], half_size=round(ch, 2),
            ztop=round(ztop, 3), label=n.get('label', nid)))
    return frames, surf


def build_crosswalks(cfg, E, dev, frames, surf):
    nodes = cfg['topology']['nodes']
    white = cfg['global']['zebra_color']
    cw = cfg['topology']['crosswalk']
    off = cw['zebra']['offset']
    stripe_w = cw['zebra']['stripe_w']
    gap = cw['zebra']['gap']
    stop_off = cw['stopline']['offset']
    stop_w = cw['stopline']['width']
    idx = 0
    for nid, n in nodes.items():
        if n['type'] != 'cross':
            continue
        nx, ny = n['pose'][:2]
        for eid, fr in frames.items():
            e_nodes = [e for e in cfg['topology']['edges'] if e['id'] == eid][0]
            if nid not in (e_nodes['a'], e_nodes['b']):
                continue
            other = e_nodes['b'] if e_nodes['a'] == nid else e_nodes['a']
            ox, oy = nodes[other]['pose'][:2]
            dx, dy = norm(ox - nx, oy - ny)
            vx, vy = -dy, dx
            half = fr['half']
            zt = fr['ztop'] + 0.004
            zc_x = nx + dx * off
            zc_y = ny + dy * off
            # 斑马线条纹（沿垂直方向排布，条带沿行车方向）
            s = -half + 0.7
            k = 0
            while s + stripe_w / 2 <= half - 0.7:
                px = zc_x + vx * (s + stripe_w / 2)
                py = zc_y + vy * (s + stripe_w / 2)
                E.add(box_model(f'zebra_{nid}_{eid}_{k}', px, py, zt,
                                4.6, stripe_w, 0.004, white,
                                yaw=math.atan2(dy, dx)))
                s += stripe_w + gap
                k += 1
            # 停止线（两个来车半幅各一条）
            for sgn in (1, -1):
                sc = sgn * (half + 0.1) / 2
                px = nx + dx * stop_off + vx * sc
                py = ny + dy * stop_off + vy * sc
                E.add(box_model(f'stopline_{nid}_{eid}_{"p" if sgn > 0 else "m"}',
                                px, py, zt, stop_w, half - 0.1, 0.004, white,
                                yaw=math.atan2(dy, dx)))
            dev['crosswalks'].append(dict(
                id=f'CW-{nid}-{eid}', node=nid, edge=eid,
                center=[round(zc_x, 2), round(zc_y, 2)],
                stripes=k, label=f'{n.get("label", nid)} {eid} 口人行横道'))
            idx += 1
    return idx


def build_elevated(cfg, E, dev):
    gray = [0.35, 0.38, 0.42]
    green = [0.1, 0.45, 0.2]
    for seg in cfg.get('elevated', []):
        (x0, y0), (x1, y1) = seg['path']
        L = math.hypot(x1 - x0, y1 - y0)
        ux, uy = norm(x1 - x0, y1 - y0)
        vx, vy = -uy, ux
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        z = seg['z']
        w = seg['width']
        yaw = math.atan2(uy, ux)
        E.add(box_model(f'{seg["id"]}_deck', cx, cy, z + 0.2, L, w, 0.4, gray,
                        with_collision=True, yaw=yaw))
        # 护栏
        for sgn in (1, -1):
            px = cx + vx * sgn * (w / 2 - 0.1)
            py = cy + vy * sgn * (w / 2 - 0.1)
            E.add(box_model(f'{seg["id"]}_rail_{"p" if sgn > 0 else "m"}',
                            px, py, z + 0.4 + seg['guard_rail'] / 2, L, 0.15,
                            seg['guard_rail'], green, yaw=yaw))
        # 端封梁
        for sgn in (1, -1):
            px = cx + ux * sgn * L / 2
            py = cy + uy * sgn * L / 2
            E.add(box_model(f'{seg["id"]}_end_{"p" if sgn > 0 else "m"}',
                            px, py, z + 0.4 + 0.55, 0.25, w, 1.1, gray, yaw=yaw))
        # 立柱
        pi = seg['pillar']['interval']
        ps = seg['pillar']['size']
        t = 0.0
        k = 0
        while t <= L + 1e-6:
            px = x0 + ux * t
            py = y0 + uy * t
            E.add(box_model(f'{seg["id"]}_pillar_{k}', px, py, z / 2,
                            ps[0], ps[1], z, [0.5, 0.5, 0.52],
                            with_collision=True))
            t += pi
            k += 1
        dev['elevated'].append(dict(
            id=seg['id'], z=z, width=w, length=round(L, 2),
            path=seg['path'], pillars=k, label=seg.get('label', seg['id'])))


def build_parking(cfg, E, dev):
    white = [0.92, 0.92, 0.92]
    gray = [0.42, 0.44, 0.47]
    red = [0.85, 0.15, 0.12]
    blue = [0.1, 0.25, 0.8]
    for p in cfg.get('parking', []):
        cx, cy = p['center'][:2]
        sw, sd = p['stall']['width'], p['stall']['depth']
        rows, stalls = p['rows'], p['stalls_per_row']
        aisle = p['aisle']
        W = stalls * sw
        D = rows * sd + aisle
        pid = p['id']
        # 地坪
        E.add(box_model(f'{pid}_pad', cx, cy, 0.008, W + 0.6, D + 0.6, 0.02, gray))
        # 两排车位（南排贴边，中间车道，北排贴边）
        y0 = cy - D / 2
        row_centers = []
        ry = y0 + sd / 2
        for _ in range(rows):
            row_centers.append(ry)
            ry += sd + aisle
        for ri, rc in enumerate(row_centers):
            for i in range(stalls + 1):
                E.add(box_model(f'{pid}_stall_{ri}_{i}',
                                cx - W / 2 + i * sw, rc, 0.024, 0.12, sd, 0.004, white))
            for yy in (rc - sd / 2, rc + sd / 2):
                E.add(box_model(f'{pid}_rowb_{ri}_{"a" if yy < rc else "b"}',
                                cx, yy, 0.024, W, 0.12, 0.004, white))
        # 出入口（南缘中点）：红白警示柱×2 + 起落杆 + P 牌
        gy = cy - D / 2
        for sgn, tag in ((-1, 'w'), (1, 'e')):
            gx = cx + sgn * 3.0
            for k, (zc, hh) in enumerate(((0.3, 0.6), (0.9, 0.6), (1.5, 0.6))):
                col = red if k % 2 == 0 else white
                E.add(box_model(f'{pid}_post_{tag}_{k}', gx, gy, zc, 0.18, 0.18, hh, col))
        E.add(box_model(f'{pid}_barrier', cx, gy + 0.35, 1.05, 6.4, 0.09, 0.09, white))
        E.add(box_model(f'{pid}_barrier_red', cx, gy + 0.35, 1.05, 1.6, 0.1, 0.1, red))
        px, py = cx + 4.5, gy - 0.6
        E.add(cyl_model(f'{pid}_sign_pole', px, py, 1.5, 0.06, 3.0, gray))
        E.add(box_model(f'{pid}_sign_P', px, py, 3.35, 1.1, 0.15, 1.3, blue))
        # 联络道（南门 → 东西路北缘 y=6.4）
        ROAD_EDGE = 6.4
        if p.get('gate', {}).get('side') == 'south' and gy > ROAD_EDGE + 0.5:
            link_len = gy - ROAD_EDGE
            E.add(box_model(f'{pid}_access', cx, (gy + ROAD_EDGE) / 2, 0.006,
                            6.0, link_len, 0.02, [0.15, 0.15, 0.15]))
        dev['parking'].append(dict(
            id=pid, center=[cx, cy], rows=rows, stalls_per_row=stalls,
            total_stalls=rows * stalls, gate=[cx, gy, 1.05],
            gate_side=p.get('gate', {}).get('side'), label=p.get('label', pid)))


def build_etc(cfg, E, dev):
    green = [0.05, 0.5, 0.25]
    dark = [0.2, 0.2, 0.22]
    white = [0.92, 0.92, 0.92]
    for g in cfg.get('etc_gantries', []):
        x, y = g['pose'][:2]
        h, span, yaw = g['height'], g['span'], g.get('yaw', 0.0)
        gid = g['id']
        cos, sin = math.cos(yaw), math.sin(yaw)

        def loc(dx, dy):
            return x + dx * cos - dy * sin, y + dx * sin + dy * cos

        for sgn, tag in ((1, 'e'), (-1, 'w')):
            px, py = loc(sgn * span / 2, 0)
            E.add(box_model(f'{gid}_post_{tag}', px, py, h / 2, 0.4, 0.4, h,
                            [0.55, 0.55, 0.58], with_collision=True, yaw=yaw))
        bx, by = loc(0, 0)
        E.add(box_model(f'{gid}_beam', bx, by, h - 0.4, span, 0.6, 0.8, green, yaw=yaw))
        E.add(box_model(f'{gid}_stripe', bx, by, h - 0.85, span, 0.62, 0.06, white, yaw=yaw))
        for sgn, tag in ((1, 'e'), (-1, 'w')):
            px, py = loc(sgn * 2.5, 0)
            E.add(box_model(f'{gid}_reader_{tag}', px, py, h - 0.95, 0.8, 0.5, 0.4,
                            dark, yaw=yaw))
        dev['etc_gantries'].append(dict(
            id=gid, pose=[x, y], span=span, height=h, yaw=yaw,
            covered_lanes=g.get('lanes'), road=g.get('road'),
            reader_height=round(h - 0.95, 2), label=g.get('label', gid)))


def build_rsu(cfg, E, dev):
    gray = [0.45, 0.47, 0.5]
    dark = [0.18, 0.2, 0.24]
    for r in cfg.get('rsu', []):
        x, y = r['pose'][:2]
        h = r['height']
        rid = r['id']
        E.add(cyl_model(f'{rid}_pole', x, y, h / 2, 0.07, h, gray, with_collision=True))
        E.add(box_model(f'{rid}_box', x, y, h + 0.2, 0.6, 0.4, 0.35, dark))
        dev['rsu'].append(dict(
            id=rid, pose=[x, y], height=h, antenna_top=round(h + 0.375, 2),
            label=r.get('label', rid)))


def build_greenery(cfg, E, dev, frames):
    total = 0
    for gr in cfg.get('greenery', []):
        fr = frames.get(gr['along'])
        if fr is None:
            print(f'[builder] 警告: greenery {gr["id"]} 引用未知边 {gr["along"]}')
            continue
        rnd = random.Random(gr.get('seed', 0))
        interval = gr['interval']
        off = gr['offset']
        sides = [1, -1] if gr.get('side', 'both') == 'both' else [1]
        # 树在路外，独立 break：路口 11m（避灯/RSU/斑马线），端点 3m
        br_a = min(fr['br_a'], 11.0)
        br_b = min(fr['br_b'], 11.0)
        t = br_a + interval * 0.6
        while t <= fr['L'] - br_b:
            for sgn in sides:
                px = fr['ax'] + fr['ux'] * t + fr['vx'] * sgn * (fr['half'] + off)
                py = fr['ay'] + fr['uy'] * t + fr['vy'] * sgn * (fr['half'] + off)
                tr = rnd.uniform(0.07, 0.11)
                th = rnd.uniform(1.7, 2.4)
                cr = rnd.uniform(0.85, 1.3)
                jit = rnd.uniform(-0.06, 0.08)
                E.add(cyl_model(f'{gr["id"]}_t{total}', px, py, th / 2, tr, th,
                                [0.42, 0.30, 0.18]))
                E.add(sphere_model(f'{gr["id"]}_c{total}', px, py, th + cr * 0.62, cr,
                                   [0.18 + jit, 0.45 + jit * 0.5, 0.16]))
                total += 1
            t += interval
        dev.setdefault('greenery_rows', []).append(dict(
            id=gr['id'], along=gr['along'], interval=interval, label=gr.get('label', gr['id'])))
    return total


def build_lights_landmarks_legacy(cfg, E, dev):
    for tl in cfg.get('traffic_lights', []):
        x, y = tl['pose'][:2]
        grp = tl.get('group', 'default')
        # 组话题名须为合法 ROS 话题名：非字母数字下划线一律替换为下划线
        safe = re.sub(r'[^A-Za-z0-9_]', '_', str(grp))
        topic = f'traffic_light/{safe}/state'
        E.add(traffic_light_model(tl['id'], x, y, tl.get('yaw', 0), topic))
        dev['traffic_lights'].append(dict(
            id=tl['id'], pose=[x, y], yaw=tl.get('yaw'), face=tl.get('face'),
            group=grp, state_topic=topic, label=tl.get('label', tl['id'])))
    for lm in cfg.get('landmarks', []):
        x, y, zo = lm['pose'][0], lm['pose'][1], lm.get('z_offset', 0)
        E.add(include_model(lm['id'], lm['uri'], x, y, zo, lm.get('yaw', 0)))
        dev['landmarks'].append(dict(
            id=lm['id'], uri=lm['uri'], pose=[x, y],
            z_offset=zo, yaw=lm.get('yaw'), label=lm.get('label', lm['id'])))
    for an in cfg.get('legacy_anchors', []):
        x, y, z = an['pose']
        sx, sy, sz = an['size']
        E.add(box_model(an['id'], x, y, z, sx, sy, sz,
                        [0.5 + (hash(an['id']) % 3) * 0.05,
                         0.5 + (hash(an['id']) % 5) * 0.03, 0.55],
                        with_collision=True))
        dev['anchors'].append(dict(
            id=an['id'], pose=[x, y, z], size=list(an['size']),
            label=an.get('label', an['id'])))


# ───────────────────────── 主流程 ─────────────────────────
def main():
    if len(sys.argv) < 4:
        print('用法: scene_builder.py <v2x_scene.yaml> <out.world> <out_devices.yaml>')
        sys.exit(1)
    cfg_path, out_world, out_dev = sys.argv[1], sys.argv[2], sys.argv[3]
    with open(cfg_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    g = cfg['global']
    dev = dict(scene=g['name'], frame='world（= odom = 初始 map，单位 m）',
               intersections={}, road_segments={}, crosswalks=[],
               traffic_lights=[], etc_gantries=[], rsu=[], parking=[],
               elevated=[], landmarks=[], anchors=[])

    E = Emitter()
    E.add(HEAD.format(name=g['name'],
                      sun=' '.join(fnum(v) for v in g['sun']['direction']),
                      gx=fnum(g['ground']['size'][0]), gy=fnum(g['ground']['size'][1]),
                      gcol=vec(g['ground']['color'])))

    for nid, n in cfg['topology']['nodes'].items():
        dev['intersections'][nid] = dict(
            pose=list(n['pose'][:2]), type=n['type'], label=n.get('label', nid))

    frames, surf = build_roads(cfg, E, dev)
    n_cw = build_crosswalks(cfg, E, dev, frames, surf)
    build_elevated(cfg, E, dev)
    build_parking(cfg, E, dev)
    build_etc(cfg, E, dev)
    build_rsu(cfg, E, dev)
    n_trees = build_greenery(cfg, E, dev, frames)
    build_lights_landmarks_legacy(cfg, E, dev)
    cam_id = build_scene_camera(cfg, E, dev)
    E.add(TAIL)

    with open(out_world, 'w', encoding='utf-8') as f:
        f.write(E.text())
    with open(out_dev, 'w', encoding='utf-8') as f:
        yaml.safe_dump(dev, f, sort_keys=False, allow_unicode=True, width=100)

    # XML 良构校验（仅解析本生成器自身输出的本地可信文件，无不可信输入面）
    import xml.etree.ElementTree as ET
    ET.parse(out_world)
    print(f'[builder] world OK -> {out_world}')
    print(f'[builder] devices OK -> {out_dev}')
    print(f'[builder] 模型数 {E.count} | 边 {len(cfg["topology"]["edges"])} | '
          f'斑马线组 {n_cw} | 树 {n_trees} | 信号灯 {len(dev["traffic_lights"])} | '
          f'ETC {len(dev["etc_gantries"])} | RSU {len(dev["rsu"])} | 场景相机 {cam_id or "无"}')


if __name__ == '__main__':
    main()
