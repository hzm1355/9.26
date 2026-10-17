#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2X 可视化操作界面 —— 对应协议「总体技术要求 4」大屏可视化操作界面。

三大区域（可拖拽分隔、F11 全屏，适配大屏）：
  1) 场景视口 SceneView   ：俯视 2D 地图（道路/路口/人行横道/区域），实时绘制
                            /model_states 车辆、回放幽灵、信号灯灯态（按 group 着色）。
  2) 参数控制面板 ControlPanel：信号配时(G/Y/R)、预设方案、紧急全绿、轨迹记录/回放。
  3) 日志窗口 LogView      ：区域触发事件(/v2x/region_events) + 命令回执 + 状态。

设计要点：
  - 单线程：QTimer 周期调用 rclpy.spin_once 泵送订阅/服务，回调与 UI 同线程，
    避免跨线程操作 Qt 组件的崩溃；服务用 call_async，结果由 done_callback 处理。
  - 地图几何来自 scene_devices.yaml / regions.yaml，与仿真世界同源，无需硬编码坐标。
  - 无 Gazebo/无数据时仍可启动（只显示静态地图），便于演示与调试。
"""
import os
import sys
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from PyQt5 import QtCore, QtGui, QtWidgets

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from v2x_msgs.msg import RegionEvent

try:
    # 确保能导入同目录的 trajectory_player 模块
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    if _script_dir not in sys.path:
        sys.path.insert(0, _script_dir)
    from trajectory_player import sample_at  # noqa: F401  (仅确保依赖可导入)
except Exception:  # noqa: BLE001
    pass


def _yaw_from_quat(qx, qy, qz, qw):
    # 平面偏航角（world 帧，绕 +z）
    import math
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# ============================================================================
# ROS 数据/命令层（与 Qt 解耦，便于无界面自测）
# ============================================================================
class DashboardNode(Node):
    def __init__(self):
        super().__init__('v2x_dashboard')
        cfg_dir = os.path.join(get_package_share_directory('v2x_demo'), 'config')
        self.declare_parameter('scene_devices', os.path.join(cfg_dir, 'scene_devices.yaml'))
        self.declare_parameter('regions_file', os.path.join(cfg_dir, 'regions.yaml'))
        self.declare_parameter('tl_params', os.path.join(cfg_dir, 'traffic_light_params.yaml'))
        self.declare_parameter('scene_yaml', os.path.join(cfg_dir, 'v2x_scene.yaml'))
        self.declare_parameter('tl_node', '/traffic_light_manager')
        self.declare_parameter('recorder_node', '/trajectory_recorder')
        self.declare_parameter('player_node', '/trajectory_player')
        # 实景3D：Gazebo 场景相机（视频源 + 可被 /set_entity_state 驱动的位姿）
        self.declare_parameter('scene_cam', 'v2x_scene_cam')
        self.declare_parameter('scene_cam_topic', '')   # 空=从 scene_devices.yaml 读取

        # ---- 静态地图几何 ----
        self.nodes = {}          # id -> dict(x,y,type,label)
        self.segments = []       # list of dict(ax,ay,bx,by,width,label,speed)
        self.crosswalks = []     # list of dict(x,y)
        self.lights = []         # list of dict(id,x,y,group)
        self.regions = []        # list of dict(id,type,cx,cy,sx,sy)
        self.presets = []        # list of preset names
        self.groups = []         # list of signal group ids
        # 3D 地物（来自 v2x_scene.yaml）：高架/建筑/门架/RSU/停车场
        self.elevated = []       # list of dict(path,z,width,pillar)
        self.landmarks = []      # list of dict(id,x,y,yaw,z_offset,kind)
        self.anchors = []        # list of dict(x,y,z,sx,sy,sz)
        self.gantries = []       # list of dict(x,y,span,height,yaw)
        self.rsu = []            # list of dict(x,y,height)
        self.parking = []        # list of dict(cx,cy,yaw,rows,stalls,sw,sd,aisle)
        self.cam_topic = '/v2x_scene_cam/cam/image_raw'   # 实景相机图像话题（可被配置覆盖）
        self._load_scene(self.get_parameter('scene_devices').value)
        self._load_regions(self.get_parameter('regions_file').value)
        self._load_tl_params(self.get_parameter('tl_params').value)
        self._load_scene3d(self.get_parameter('scene_yaml').value)

        # ---- 运行时状态 ----
        self.vehicles = {}       # name -> dict(x,y,z,yaw)
        self.light_state = {}    # group -> 'RED'/'YELLOW'/'GREEN'
        self.playback = {}       # name -> dict(x,y,z,yaw)
        self.events = deque(maxlen=400)   # (stamp, region_id, vehicle, kind, speed)
        self.sim_time = 0.0
        self.event_listeners = []   # UI 回调（在主线程 spin_once 中触发，安全）
        self.cam_default = {}       # 实景相机默认轨道参数（_load_scene3d 覆盖）
        self.cam_image = None       # 最新一帧 QImage（实景3D 视频源）
        self.cam_frame_stamp = 0.0  # 最近收到帧的墙钟时间
        self.cam_frames = 0         # 累计帧计数
        self._cam_last_cmd = 0.0    # cmd_camera 节流时间戳

        # ---- 订阅 ----
        self.create_subscription(ModelStates, '/model_states', self.on_model_states, 10)
        self.create_subscription(RegionEvent, '/v2x/region_events', self.on_region_event, 10)
        for g in self.groups:
            self.create_subscription(String, f'traffic_light/{g}/state',
                                     lambda m, gg=g: self.on_light(gg, m), 10)
        # 回放幽灵位姿（玩家为每辆车发布 /v2x/playback/<name>/pose）
        self._playback_subs = {}

        # 实景3D：订阅 Gazebo 场景相机图像（sensor_data QoS，best_effort）
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        cam_topic = self.get_parameter('scene_cam_topic').value or self.cam_topic
        self.create_subscription(Image, cam_topic, self.on_cam_image, cam_qos)

        # ---- 发布 / 客户端 ----
        self.preset_pub = self.create_publisher(String, 'traffic_light/preset', 10)
        tl = self.get_parameter('tl_node').value
        self.emergency_cli = self.create_client(SetBool, f'{tl}/emergency')
        self.setparam_cli = self.create_client(SetParameters, f'{tl}/set_parameters')
        rec = self.get_parameter('recorder_node').value
        ply = self.get_parameter('player_node').value
        self.rec_start = self.create_client(Trigger, f'{rec}/start')
        self.rec_stop = self.create_client(Trigger, f'{rec}/stop')
        self.rec_save = self.create_client(Trigger, f'{rec}/save')
        self.ply_load = self.create_client(Trigger, f'{ply}/load')
        self.ply_play = self.create_client(Trigger, f'{ply}/play')
        self.ply_stop = self.create_client(Trigger, f'{ply}/stop')
        # 实景3D：驱动场景相机位姿
        self.setstate_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.scene_cam_name = self.get_parameter('scene_cam').value

        self.get_logger().info(
            f'DashboardNode ready: {len(self.nodes)} nodes, {len(self.segments)} segments, '
            f'{len(self.lights)} lights, groups={self.groups}, presets={self.presets}')

    # ---------------- 配置装载 ----------------
    def _load_scene(self, path):
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'读取 scene_devices 失败({path}): {e}')
            return
        for nid, n in (d.get('intersections') or {}).items():
            p = n.get('pose', [0, 0])
            self.nodes[nid] = dict(x=float(p[0]), y=float(p[1]),
                                   type=n.get('type', 'endpoint'), label=n.get('label', nid))
        for sid, s in (d.get('road_segments') or {}).items():
            a = self.nodes.get(s.get('from_node'))
            b = self.nodes.get(s.get('to_node'))
            if not a or not b:
                continue
            self.segments.append(dict(ax=a['x'], ay=a['y'], bx=b['x'], by=b['y'],
                                      width=float(s.get('total_width', 8.0)),
                                      label=sid, speed=s.get('speed_limit', 30)))
        for cw in (d.get('crosswalks') or []):
            c = cw.get('center', [0, 0])
            self.crosswalks.append(dict(x=float(c[0]), y=float(c[1])))
        for tl in (d.get('traffic_lights') or []):
            p = tl.get('pose', [0, 0])
            self.lights.append(dict(id=tl.get('id'), x=float(p[0]), y=float(p[1]),
                                    group=tl.get('group', 'default')))
        sc = d.get('scene_camera') or {}
        if sc.get('image_topic'):
            self.cam_topic = str(sc['image_topic'])

    def _load_regions(self, path):
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'读取 regions 失败({path}): {e}')
            return
        for r in (d.get('regions') or []):
            c = r.get('center', [0, 0, 1.0])
            s = r.get('size', [1, 1, 1])
            self.regions.append(dict(id=r.get('id'), type=r.get('type', 'custom'),
                                     cx=float(c[0]), cy=float(c[1]),
                                     sx=float(s[0]), sy=float(s[1])))

    def _load_tl_params(self, path):
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'读取 tl_params 失败({path}): {e}')
            return
        self.presets = list((d.get('presets') or {}).keys())
        # group 列表：优先 scene_devices 灯组，其次 tl_params groups
        gs = []
        for tl in self.lights:
            if tl['group'] not in gs:
                gs.append(tl['group'])
        for g in (d.get('groups') or {}):
            if g not in gs:
                gs.append(g)
        self.groups = gs or ['default']
        self.default_preset = d.get('preset', self.presets[0] if self.presets else 'default')

    def _load_scene3d(self, path):
        """读取 v2x_scene.yaml 的 3D 地物（高架/地标建筑/素 box/门架/RSU/停车场）。"""
        try:
            with open(path, encoding='utf-8') as f:
                d = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'读取 v2x_scene 失败({path}): {e}')
            return
        for el in (d.get('elevated') or []):
            self.elevated.append(dict(path=el.get('path', []), z=float(el.get('z', 7.0)),
                                      width=float(el.get('width', 8.0)),
                                      pillar=el.get('pillar', {})))
        # 地标建筑：按 id 关键字给一个近似包围盒（仅可视化用）
        kind_size = {'school': (24, 16, 12), 'office': (16, 16, 22), 'law': (14, 14, 26)}
        for lm in (d.get('landmarks') or []):
            p = lm.get('pose', [0, 0, 0])
            uri = str(lm.get('uri', ''))
            kind = 'school' if 'school' in uri else ('office' if 'office' in uri else 'law')
            sx, sy, sz = kind_size[kind]
            self.landmarks.append(dict(id=lm.get('id'), x=float(p[0]), y=float(p[1]),
                                       yaw=float(lm.get('yaw', 0.0)),
                                       z=float(p[2]) + float(lm.get('z_offset', 0.0)),
                                       sx=sx, sy=sy, sz=sz))
        for an in (d.get('legacy_anchors') or []):
            p = an.get('pose', [0, 0, 0]); s = an.get('size', [1, 1, 1])
            self.anchors.append(dict(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                                     sx=float(s[0]), sy=float(s[1]), sz=float(s[2])))
        for g in (d.get('etc_gantries') or []):
            p = g.get('pose', [0, 0])
            self.gantries.append(dict(x=float(p[0]), y=float(p[1]),
                                      span=float(g.get('span', 14.0)),
                                      height=float(g.get('height', 5.5)),
                                      yaw=float(g.get('yaw', 0.0))))
        for r in (d.get('rsu') or []):
            p = r.get('pose', [0, 0])
            self.rsu.append(dict(x=float(p[0]), y=float(p[1]), height=float(r.get('height', 4.0))))
        for pk in (d.get('parking') or []):
            c = pk.get('center', [0, 0])
            st = pk.get('stall', {})
            self.parking.append(dict(cx=float(c[0]), cy=float(c[1]), yaw=float(pk.get('yaw', 0.0)),
                                     rows=int(pk.get('rows', 1)), stalls=int(pk.get('stalls_per_row', 1)),
                                     sw=float(st.get('width', 2.6)), sd=float(st.get('depth', 5.0)),
                                     aisle=float(pk.get('aisle', 6.0))))
        self.get_logger().info(
            f'3D 地物: elevated={len(self.elevated)} landmarks={len(self.landmarks)} '
            f'anchors={len(self.anchors)} gantries={len(self.gantries)} rsu={len(self.rsu)} '
            f'parking={len(self.parking)}')
        cam = d.get('scene_camera') or {}
        self.cam_default = cam.get('default', {}) or {}

    # ---------------- 订阅回调 ----------------
    def on_model_states(self, msg: ModelStates):
        self.sim_time = self.get_clock().now().nanoseconds / 1e9
        names = msg.name
        poses = msg.pose
        seen = set()
        for i, nm in enumerate(names):
            if i >= len(poses):
                break
            if not (nm.startswith('v2x_') or nm.startswith('playback_')):
                continue
            p = poses[i]
            q = p.orientation
            rec = dict(x=p.position.x, y=p.position.y, z=p.position.z,
                       yaw=_yaw_from_quat(q.x, q.y, q.z, q.w))
            if nm.startswith('playback_'):
                self.playback[nm[len('playback_'):]] = rec
            else:
                self.vehicles[nm] = rec
            seen.add(nm)
        # 清理已消失的实体
        for nm in list(self.vehicles):
            if nm not in seen and f'playback_{nm}' not in seen:
                self.vehicles.pop(nm, None)

    def on_light(self, group, msg: String):
        self.light_state[group] = msg.data.strip().upper()

    def on_cam_image(self, msg: Image):
        """把最新一帧转成 QImage 缓存，供实景3D 视口绘制。支持 rgb8/bgr8。"""
        try:
            w, h = int(msg.width), int(msg.height)
            step = int(msg.step) if msg.step else w * 3
            enc = (msg.encoding or 'rgb8').lower()
            data = bytes(msg.data)
            if enc in ('rgb8', 'bgr8'):
                fmt = (QtGui.QImage.Format_RGB888 if enc == 'rgb8'
                       else QtGui.QImage.Format_BGR888)
                img = QtGui.QImage(data, w, h, step, fmt)
            elif enc in ('rgba8', 'bgra8'):
                fmt = (QtGui.QImage.Format_RGBA8888 if enc == 'rgba8'
                       else QtGui.QImage.Format_RGB32)
                img = QtGui.QImage(data, w, h, step, fmt)
            elif enc in ('mono8', '8uc1'):
                img = QtGui.QImage(data, w, h, step, QtGui.QImage.Format_Grayscale8)
            else:
                return  # 未知编码，忽略
            self.cam_image = img.copy()   # copy：脱离 msg.data 生命周期
            self.cam_frame_stamp = self.get_clock().now().nanoseconds / 1e9
            self.cam_frames += 1
        except Exception:  # noqa: BLE001
            pass

    def on_region_event(self, msg: RegionEvent):
        kind = 'ENTER' if msg.event_type == RegionEvent.EVENT_ENTER else 'EXIT'
        ev = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
              msg.region_id, msg.vehicle_id, kind, float(msg.speed))
        self.events.append(ev)
        for cb in list(self.event_listeners):
            try:
                cb(ev)
            except Exception:  # noqa: BLE001
                pass

    # ---------------- 命令 ----------------
    def _fire(self, cli, req, label, on_done=None):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{label}: 服务未就绪')
            if on_done:
                on_done(False, '服务未就绪')
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f, lb=label, cb=on_done: self._done(f, lb, cb))

    def _done(self, fut, label, on_done):
        try:
            resp = fut.result()
            ok = bool(getattr(resp, 'success', True))
            m = getattr(resp, 'message', '')
            if on_done:
                on_done(ok, m)
        except Exception as e:  # noqa: BLE001
            if on_done:
                on_done(False, str(e))

    def cmd_preset(self, name):
        m = String()
        m.data = str(name)
        self.preset_pub.publish(m)

    def cmd_emergency(self, on: bool, on_done=None):
        req = SetBool.Request()
        req.data = bool(on)
        self._fire(self.emergency_cli, req, 'emergency', on_done)

    def cmd_durations(self, green, yellow, red, on_done=None):
        req = SetParameters.Request()
        for nm, val in (('green_duration', green), ('yellow_duration', yellow),
                        ('red_duration', red)):
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(val)
            req.parameters.append(Parameter(name=nm, value=pv))
        self._fire(self.setparam_cli, req, 'set_durations',
                   (lambda ok, m: on_done(ok, m)) if on_done else None)

    def cmd_record(self, action, on_done=None):
        cli = {'start': self.rec_start, 'stop': self.rec_stop, 'save': self.rec_save}.get(action)
        if cli is None:
            return
        self._fire(cli, Trigger.Request(), f'recorder/{action}', on_done)

    def cmd_playback(self, action, on_done=None):
        cli = {'load': self.ply_load, 'play': self.ply_play, 'stop': self.ply_stop}.get(action)
        if cli is None:
            return
        self._fire(cli, Trigger.Request(), f'player/{action}', on_done)

    def cmd_camera(self, x, y, z, qx, qy, qz, qw, throttle=15.0):
        """驱动实景相机 v2x_scene_cam 的位姿（节流，避免刷爆 /set_entity_state）。"""
        now = self.get_clock().now().nanoseconds / 1e9
        if throttle and (now - self._cam_last_cmd) < (1.0 / throttle):
            return
        self._cam_last_cmd = now
        if not self.setstate_cli.service_is_ready():
            return
        req = SetEntityState.Request()
        st = req.state
        st.name = self.scene_cam_name
        st.pose.position.x = float(x)
        st.pose.position.y = float(y)
        st.pose.position.z = float(z)
        st.pose.orientation.x = float(qx)
        st.pose.orientation.y = float(qy)
        st.pose.orientation.z = float(qz)
        st.pose.orientation.w = float(qw)
        try:
            self.setstate_cli.call_async(req)
        except Exception:  # noqa: BLE001
            pass


# ============================================================================
# 场景视口
# ============================================================================
LIGHT_COLOR = {'RED': QtGui.QColor(220, 40, 40), 'YELLOW': QtGui.QColor(230, 200, 30),
               'GREEN': QtGui.QColor(40, 200, 80)}
REGION_COLOR = {
    'etc_gate': QtGui.QColor(120, 90, 220, 60),
    'parking_gate': QtGui.QColor(60, 160, 220, 60),
    'parking_loop': QtGui.QColor(60, 200, 200, 55),
    'speed_point': QtGui.QColor(240, 150, 40, 70),
    'checkpoint': QtGui.QColor(230, 70, 130, 65),
    'custom': QtGui.QColor(150, 150, 150, 55),
}


class SceneView(QtWidgets.QWidget):
    def __init__(self, dash: DashboardNode, parent=None):
        super().__init__(parent)
        self.dash = dash
        self.setMinimumSize(480, 480)
        self.setStyleSheet('background:#1b2a1b;')
        self._compute_bounds()

    def _compute_bounds(self):
        xs, ys = [], []
        for n in self.dash.nodes.values():
            xs.append(n['x']); ys.append(n['y'])
        for r in self.dash.regions:
            xs += [r['cx'] - r['sx'] / 2, r['cx'] + r['sx'] / 2]
            ys += [r['cy'] - r['sy'] / 2, r['cy'] + r['sy'] / 2]
        if not xs:
            xs, ys = [-40, 40], [-45, 40]
        m = 6.0
        self.wx0, self.wx1 = min(xs) - m, max(xs) + m
        self.wy0, self.wy1 = min(ys) - m, max(ys) + m

    def _w2s(self, x, y, scale, ox, oy):
        return ox + (x - self.wx0) * scale, oy - (y - self.wy1) * scale

    def paintEvent(self, _e):  # noqa: N802
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        r = self.rect().adjusted(6, 6, -6, -6)
        ww, wh = self.wx1 - self.wx0, self.wy1 - self.wy0
        scale = min(r.width() / ww, r.height() / wh)
        ox = r.left() + (r.width() - ww * scale) / 2
        oy = r.top() + (r.height() - wh * scale) / 2
        S = lambda x, y: self._w2s(x, y, scale, ox, oy)  # noqa: E731

        qp.fillRect(self.rect(), QtGui.QColor(27, 42, 27))

        # 道路
        qp.setPen(QtCore.Qt.NoPen)
        asphalt = QtGui.QColor(45, 45, 48)
        for s in self.dash.segments:
            ax, ay = S(s['ax'], s['ay'])
            bx, by = S(s['bx'], s['by'])
            pen = QtGui.QPen(asphalt)
            pen.setWidthF(max(s['width'] * scale, 2.0))
            pen.setCapStyle(QtCore.Qt.FlatCap)
            qp.setPen(pen)
            qp.drawLine(QtCore.QPointF(ax, ay), QtCore.QPointF(bx, by))
        # 路口方块
        qp.setPen(QtCore.Qt.NoPen)
        qp.setBrush(asphalt)
        for n in self.dash.nodes.values():
            if n['type'] != 'cross':
                continue
            cx, cy = S(n['x'], n['y'])
            half = 6.4 * scale
            qp.drawRect(QtCore.QRectF(cx - half, cy - half, half * 2, half * 2))
        qp.setBrush(QtCore.Qt.NoBrush)

        # 区域（半透明矩形，画在道路之上以便在路面上可见）
        for reg in self.dash.regions:
            x0, y0 = S(reg['cx'] - reg['sx'] / 2, reg['cy'] + reg['sy'] / 2)
            x1, y1 = S(reg['cx'] + reg['sx'] / 2, reg['cy'] - reg['sy'] / 2)
            col = REGION_COLOR.get(reg['type'], REGION_COLOR['custom'])
            qp.fillRect(QtCore.QRectF(x0, y0, x1 - x0, y1 - y0), col)
            qp.setPen(QtGui.QColor(255, 255, 255, 90))
            qp.drawRect(QtCore.QRectF(x0, y0, x1 - x0, y1 - y0))

        # 人行横道
        qp.setPen(QtGui.QColor(230, 230, 230, 160))
        for cw in self.dash.crosswalks:
            cx, cy = S(cw['x'], cw['y'])
            d = 1.2 * scale
            qp.drawLine(QtCore.QPointF(cx - d, cy - d), QtCore.QPointF(cx + d, cy + d))
            qp.drawLine(QtCore.QPointF(cx - d, cy + d), QtCore.QPointF(cx + d, cy - d))

        # 信号灯（按 group 灯态着色）
        for tl in self.dash.lights:
            lx, ly = S(tl['x'], tl['y'])
            state = self.dash.light_state.get(tl['group'], 'RED')
            col = LIGHT_COLOR.get(state, QtGui.QColor(120, 120, 120))
            rad = max(1.4 * scale, 4.0)
            qp.setPen(QtGui.QColor(0, 0, 0, 160))
            qp.setBrush(col)
            qp.drawEllipse(QtCore.QPointF(lx, ly), rad, rad)
            qp.setBrush(QtCore.Qt.NoBrush)

        # 回放幽灵（半透明绿）
        for nm, p in self.dash.playback.items():
            self._draw_vehicle(qp, S, p, QtGui.QColor(40, 220, 120, 130), scale, ghost=True)

        # 车辆
        for nm, p in self.dash.vehicles.items():
            self._draw_vehicle(qp, S, p, QtGui.QColor(70, 140, 240), scale, name=nm)

        # 比例/指北
        qp.setPen(QtGui.QColor(255, 255, 255, 180))
        qp.drawText(self.rect().adjusted(10, 6, -10, -6),
                    QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
                    f'车辆 {len(self.dash.vehicles)}  幽灵 {len(self.dash.playback)}  '
                    f'sim_t {self.dash.sim_time:.1f}s')
        qp.drawText(self.rect().adjusted(10, 6, -10, -6),
                    QtCore.Qt.AlignTop | QtCore.Qt.AlignRight, 'N↑  +Y')

    def _draw_vehicle(self, qp, S, p, color, scale, name=None, ghost=False):
        import math
        cx, cy = S(p['x'], p['y'])
        yaw = p.get('yaw', 0.0)
        L, W = 3.0 * scale, 1.6 * scale
        L = max(L, 8.0); W = max(W, 5.0)
        qp.save()
        qp.translate(cx, cy)
        qp.rotate(-math.degrees(yaw))
        qp.setPen(QtGui.QColor(0, 0, 0, 170))
        qp.setBrush(color)
        qp.drawRect(QtCore.QRectF(-L / 2, -W / 2, L, W))
        # 车头指示
        qp.setBrush(QtGui.QColor(255, 255, 255, 200))
        qp.setPen(QtCore.Qt.NoPen)
        tri = QtGui.QPolygonF([QtCore.QPointF(L / 2, 0), QtCore.QPointF(L / 2 - W * 0.4, -W * 0.35),
                               QtCore.QPointF(L / 2 - W * 0.4, W * 0.35)])
        qp.drawPolygon(tri)
        qp.restore()
        if name:
            qp.setPen(QtGui.QColor(255, 255, 255, 200))
            qp.drawText(QtCore.QPointF(cx + W, cy - W), name)


# ============================================================================
# 3D 场景视口（纯 QPainter 三维投影 + 画家算法，无需 OpenGL，离屏可验证）
# ============================================================================
import math as _math


def _box_faces(cx, cy, cz, sx, sy, sz, yaw=0.0):
    """返回轴对齐/绕 z 旋转的盒子的 6 个面（每面 4 个三维点）。cz 为盒子中心 z。"""
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    c, s = _math.cos(yaw), _math.sin(yaw)
    local = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
             (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    pts = []
    for (lx, ly, lz) in local:
        wx = cx + lx * c - ly * s
        wy = cy + lx * s + ly * c
        pts.append((wx, wy, cz + lz))
    faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return [[pts[i] for i in f] for f in faces]


def _quad(ax, ay, bx, by, halfw, z):
    """沿 a->b 方向、半宽 halfw 的水平四边形（路面/人行道）。"""
    dx, dy = bx - ax, by - ay
    L = _math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    vx, vy = -uy, ux
    return [(ax + vx * halfw, ay + vy * halfw, z), (bx + vx * halfw, by + vy * halfw, z),
            (bx - vx * halfw, by - vy * halfw, z), (ax - vx * halfw, ay - vy * halfw, z)]


class Scene3DView(QtWidgets.QWidget):
    """可旋转/缩放的 3D 俯视场景：左键拖拽旋转、滚轮缩放、中键/右键平移。"""

    def __init__(self, dash: DashboardNode, parent=None):
        super().__init__(parent)
        self.dash = dash
        self.setMinimumSize(480, 480)
        self.setStyleSheet('background:#0e1418;')
        # 轨道相机（默认对准主路口 J1）
        self.yaw = _math.radians(-60.0)
        self.pitch = _math.radians(24.0)
        self.dist = 78.0
        self.target = [0.0, -4.0, 1.0]
        self._static = self._build_static()
        self._drag_pos = None
        self._pan_btn = None

    # ---------------- 静态几何 ----------------
    def _build_static(self):
        polys = []   # list of (points3d, QColor)
        d = self.dash
        # 地面（单个大四边形，避免拼接缺角）
        g = 160.0
        polys.append(([( -g, -g, 0.0), (g, -g, 0.0), (g, g, 0.0), (-g, g, 0.0)],
                      QtGui.QColor(38, 60, 34)))
        asphalt = QtGui.QColor(48, 48, 52)
        # 路面 + 路口块
        for s in d.segments:
            polys.append((_quad(s['ax'], s['ay'], s['bx'], s['by'], s['width'] / 2, 0.03), asphalt))
        for n in d.nodes.values():
            if n['type'] != 'cross':
                continue
            polys.append((_quad(n['x'] - 6.4, n['y'] - 6.4, n['x'] + 6.4, n['y'] - 6.4, 6.4, 0.03), asphalt))
            polys.append((_quad(n['x'] - 6.4, n['y'] + 6.4, n['x'] + 6.4, n['y'] + 6.4, 6.4, 0.03), asphalt))
        # 人行横道（白条）
        for cw in d.crosswalks:
            polys.append((_quad(cw['x'] - 2.0, cw['y'] - 2.0, cw['x'] + 2.0, cw['y'] + 2.0, 2.0, 0.05),
                          QtGui.QColor(210, 210, 210)))
        # 区域（半透明顶面）
        for reg in d.regions:
            col = REGION_COLOR.get(reg['type'], REGION_COLOR['custom'])
            polys.append((_quad(reg['cx'] - reg['sx'] / 2, reg['cy'] - reg['sy'] / 2,
                                reg['cx'] + reg['sx'] / 2, reg['cy'] - reg['sy'] / 2,
                                reg['sy'] / 2, 0.06), col))
            polys.append((_quad(reg['cx'] - reg['sx'] / 2, reg['cy'] + reg['sy'] / 2,
                                reg['cx'] + reg['sx'] / 2, reg['cy'] + reg['sy'] / 2,
                                reg['sy'] / 2, 0.06), col))
        # 高架：桥面 + 立柱
        for el in d.elevated:
            path = el['path']
            if len(path) >= 2:
                (ax, ay), (bx, by) = path[0], path[-1]
                deck = QtGui.QColor(90, 90, 96)
                for f in _box_faces((ax + bx) / 2, (ay + by) / 2, el['z'],
                                    _math.hypot(bx - ax, by - ay), el['width'], 0.5,
                                    _math.atan2(by - ay, bx - ax)):
                    polys.append((f, deck))
                # 立柱
                n = max(2, int(_math.hypot(bx - ax, by - ay) / float(el['pillar'].get('interval', 8.0))))
                for i in range(n + 1):
                    t = i / float(n)
                    px, py = ax + (bx - ax) * t, ay + (by - ay) * t
                    for f in _box_faces(px, py, el['z'] / 2, 0.7, 0.7, el['z']):
                        polys.append((f, QtGui.QColor(120, 120, 125)))
        # 地标建筑 / 素 box
        for lm in d.landmarks:
            for f in _box_faces(lm['x'], lm['y'], lm['z'] + lm['sz'] / 2, lm['sx'], lm['sy'], lm['sz'], lm['yaw']):
                polys.append((f, QtGui.QColor(150, 130, 110)))
        for an in d.anchors:
            for f in _box_faces(an['x'], an['y'], an['z'], an['sx'], an['sy'], an['sz']):
                polys.append((f, QtGui.QColor(140, 120, 100)))
        # ETC 门架：横梁 + 两立柱
        for g in d.gantries:
            for f in _box_faces(g['x'], g['y'], g['height'], 0.4, g['span'], 0.4, g['yaw']):
                polys.append((f, QtGui.QColor(200, 160, 40)))
            for sgn in (-1, 1):
                py = g['y'] + sgn * g['span'] / 2
                for f in _box_faces(g['x'], py, g['height'] / 2, 0.3, 0.3, g['height']):
                    polys.append((f, QtGui.QColor(180, 140, 40)))
        # RSU 杆
        for r in d.rsu:
            for f in _box_faces(r['x'], r['y'], r['height'] / 2, 0.25, 0.25, r['height']):
                polys.append((f, QtGui.QColor(90, 110, 140)))
        # 停车场地面 + 车位
        for pk in d.parking:
            w = pk['stalls'] * pk['sw']
            dep = pk['rows'] * pk['sd'] + pk['aisle']
            polys.append((_quad(pk['cx'] - w / 2, pk['cy'] - dep / 2, pk['cx'] + w / 2, pk['cy'] - dep / 2,
                                dep / 2, 0.04), QtGui.QColor(70, 70, 74)))
            polys.append((_quad(pk['cx'] - w / 2, pk['cy'] + dep / 2, pk['cx'] + w / 2, pk['cy'] + dep / 2,
                                dep / 2, 0.04), QtGui.QColor(70, 70, 74)))
        # 信号灯杆 + 灯箱（灯球颜色动态，见 paintEvent）
        for tl in d.lights:
            for f in _box_faces(tl['x'], tl['y'], 1.5, 0.2, 0.2, 3.0):
                polys.append((f, QtGui.QColor(60, 60, 65)))
            for f in _box_faces(tl['x'], tl['y'], 3.4, 0.5, 0.5, 1.2):
                polys.append((f, QtGui.QColor(30, 30, 32)))
        return polys

    # ---------------- 交互 ----------------
    def mousePressEvent(self, e):  # noqa: N802
        self._drag_pos = e.pos()
        self._pan_btn = e.button()

    def mouseMoveEvent(self, e):  # noqa: N802
        if self._drag_pos is None:
            return
        dx = e.x() - self._drag_pos.x()
        dy = e.y() - self._drag_pos.y()
        self._drag_pos = e.pos()
        if self._pan_btn == QtCore.Qt.LeftButton:
            self.yaw += dx * 0.008
            self.pitch = max(_math.radians(5), min(_math.radians(85), self.pitch + dy * 0.008))
        else:  # 平移
            self.target[0] -= dx * 0.05
            self.target[1] += dy * 0.05
        self.update()

    def wheelEvent(self, e):  # noqa: N802
        self.dist = max(20.0, min(400.0, self.dist * (1.0 + e.angleDelta().y() * -0.001)))
        self.update()

    # ---------------- 投影与绘制 ----------------
    def _camera_basis(self):
        cp, sp = _math.cos(self.pitch), _math.sin(self.pitch)
        cy, sy = _math.cos(self.yaw), _math.sin(self.yaw)
        C = (self.target[0] + self.dist * cp * cy,
             self.target[1] + self.dist * cp * sy,
             self.target[2] + self.dist * sp)
        f = (self.target[0] - C[0], self.target[1] - C[1], self.target[2] - C[2])
        fl = _math.sqrt(sum(v * v for v in f)) or 1.0
        f = tuple(v / fl for v in f)
        up = (0.0, 0.0, 1.0)
        r = (f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0])
        rl = _math.sqrt(sum(v * v for v in r)) or 1.0
        r = tuple(v / rl for v in r)
        u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
        return C, f, r, u

    def paintEvent(self, _e):  # noqa: N802
        qp = QtGui.QPainter(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        qp.fillRect(self.rect(), QtGui.QColor(14, 20, 24))
        W, H = self.width(), self.height()
        C, f, r, u = self._camera_basis()
        focal = min(W, H) * 1.15
        cx, cy = W / 2.0, H / 2.0

        # 动态多边形：车辆 + 幽灵 + 信号灯灯球
        dyn = []
        for nm, p in self.dash.vehicles.items():
            for fce in _box_faces(p['x'], p['y'], 0.7, 3.0, 1.6, 1.0, p.get('yaw', 0.0)):
                dyn.append((fce, QtGui.QColor(70, 140, 240)))
            for fce in _box_faces(p['x'], p['y'], 1.5, 1.6, 1.2, 0.7, p.get('yaw', 0.0)):
                dyn.append((fce, QtGui.QColor(120, 180, 250)))
        for nm, p in self.dash.playback.items():
            for fce in _box_faces(p['x'], p['y'], 0.7, 3.0, 1.6, 1.0, p.get('yaw', 0.0)):
                dyn.append((fce, QtGui.QColor(40, 220, 120, 140)))
        for tl in self.dash.lights:
            state = self.dash.light_state.get(tl['group'], 'RED')
            col = LIGHT_COLOR.get(state, QtGui.QColor(120, 120, 120))
            for fce in _box_faces(tl['x'], tl['y'], 3.6, 0.45, 0.45, 0.45):
                dyn.append((fce, col))

        allpolys = self._static + dyn
        NEAR = 0.5

        def to_view(p):
            d = (p[0] - C[0], p[1] - C[1], p[2] - C[2])
            return (d[0] * r[0] + d[1] * r[1] + d[2] * r[2],
                    d[0] * u[0] + d[1] * u[1] + d[2] * u[2],
                    d[0] * f[0] + d[1] * f[1] + d[2] * f[2])

        def clip_near(poly):
            # Sutherland–Hodgman：保留 zv >= NEAR 的部分
            out = []
            n = len(poly)
            for i in range(n):
                a, b = poly[i], poly[(i + 1) % n]
                ain, bin_ = a[2] >= NEAR, b[2] >= NEAR
                if ain:
                    out.append(a)
                if ain != bin_:
                    t = (NEAR - a[2]) / (b[2] - a[2])
                    out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, NEAR))
            return out

        rendered = []
        for pts, col in allpolys:
            view = [to_view(p) for p in pts]
            view = clip_near(view)
            if len(view) < 3:
                continue
            scr = [(cx + v[0] / v[2] * focal, cy - v[1] / v[2] * focal) for v in view]
            rendered.append((sum(v[2] for v in view) / len(view), scr, col))
        rendered.sort(key=lambda t: -t[0])
        for _depth, scr, col in rendered:
            poly = QtGui.QPolygonF([QtCore.QPointF(x, y) for (x, y) in scr])
            qp.setPen(QtGui.QColor(0, 0, 0, 60))
            qp.setBrush(col)
            qp.drawPolygon(poly)

        qp.setPen(QtGui.QColor(255, 255, 255, 200))
        qp.drawText(self.rect().adjusted(10, 6, -10, -6), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
                    f'3D  车辆 {len(self.dash.vehicles)}  幽灵 {len(self.dash.playback)}  '
                    f'左键旋转/滚轮缩放/右键平移')


# ============================================================================
# 实景3D 视口（直接嵌 Gazebo 渲染：驱动场景相机位姿 + 显示其视频流）
# ============================================================================
class SceneCamView(QtWidgets.QWidget):
    """真实 Gazebo 渲染画面。左键拖拽旋转、滚轮缩放、右键平移：
    每帧把轨道参数换算成相机位姿经 /set_entity_state 下发给 v2x_scene_cam，
    并把订阅到的 /v2x_scene_cam/image_raw 最新帧绘制到视口（含贴图/光照/阴影）。"""

    def __init__(self, dash: DashboardNode, parent=None):
        super().__init__(parent)
        self.dash = dash
        self.setMinimumSize(480, 480)
        self.setStyleSheet('background:#000;')
        d = dash.cam_default or {}
        t = d.get('target', [0.0, -4.0, 1.0])
        self.target = [float(t[0]), float(t[1]), float(t[2])]
        self.yaw = _math.radians(float(d.get('yaw', -60.0)))
        self.pitch = _math.radians(float(d.get('pitch', 22.0)))
        self.dist = float(d.get('dist', 60.0))
        self._drag_pos = None
        self._pan_btn = None

    # ---------------- 交互 ----------------
    def mousePressEvent(self, e):  # noqa: N802
        self._drag_pos = e.pos()
        self._pan_btn = e.button()

    def mouseMoveEvent(self, e):  # noqa: N802
        if self._drag_pos is None:
            return
        dx = e.x() - self._drag_pos.x()
        dy = e.y() - self._drag_pos.y()
        self._drag_pos = e.pos()
        if self._pan_btn == QtCore.Qt.LeftButton:
            self.yaw += dx * 0.008
            self.pitch = max(_math.radians(3), min(_math.radians(85), self.pitch + dy * 0.008))
        else:
            self.target[0] -= dx * 0.05
            self.target[1] += dy * 0.05
        self.drive_camera()

    def wheelEvent(self, e):  # noqa: N802
        self.dist = max(8.0, min(400.0, self.dist * (1.0 + e.angleDelta().y() * -0.001)))
        self.drive_camera()

    # ---------------- 相机位姿解算 ----------------
    def _pose(self):
        cp, sp = _math.cos(self.pitch), _math.sin(self.pitch)
        cy, sy = _math.cos(self.yaw), _math.sin(self.yaw)
        # 相机位置：绕 target 的球面
        C = (self.target[0] + self.dist * cp * cy,
             self.target[1] + self.dist * cp * sy,
             self.target[2] + self.dist * sp)
        # 视线方向 f = normalize(target - C)
        f = (self.target[0] - C[0], self.target[1] - C[1], self.target[2] - C[2])
        fl = _math.sqrt(sum(v * v for v in f)) or 1.0
        f = tuple(v / fl for v in f)
        # Gazebo 相机前向 = 模型 +X，up = +Z → yaw=atan2(fy,fx), pitch=-asin(fz), roll=0
        yaw = _math.atan2(f[1], f[0])
        pitch = -_math.asin(max(-1.0, min(1.0, f[2])))
        c1, s1 = _math.cos(yaw / 2), _math.sin(yaw / 2)
        c2, s2 = _math.cos(pitch / 2), _math.sin(pitch / 2)
        # ZYX 四元数（roll=0）
        qx = -s1 * s2
        qy = c1 * s2
        qz = s1 * c2
        qw = c1 * c2
        return C, (qx, qy, qz, qw)

    def drive_camera(self):
        C, q = self._pose()
        self.dash.cmd_camera(C[0], C[1], C[2], q[0], q[1], q[2], q[3])

    # ---------------- 绘制 ----------------
    def paintEvent(self, _e):  # noqa: N802
        qp = QtGui.QPainter(self)
        img = self.dash.cam_image
        if img is None or img.isNull():
            qp.fillRect(self.rect(), QtGui.QColor(10, 12, 14))
            qp.setPen(QtGui.QColor(180, 200, 220))
            qp.drawText(self.rect(), QtCore.Qt.AlignCenter,
                        '实景3D：等待 Gazebo 场景相机画面…\n'
                        '（需运行仿真世界 + 场景相机图像话题）')
        else:
            scaled = img.scaled(self.size(), QtCore.Qt.KeepAspectRatio,
                                QtCore.Qt.SmoothTransformation)
            ox = (self.width() - scaled.width()) // 2
            oy = (self.height() - scaled.height()) // 2
            qp.fillRect(self.rect(), QtGui.QColor(0, 0, 0))
            qp.drawImage(ox, oy, scaled)
        qp.setPen(QtGui.QColor(255, 255, 255, 210))
        qp.drawText(self.rect().adjusted(10, 6, -10, -6),
                    QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
                    f'实景3D(Gazebo)  帧 {self.dash.cam_frames}  '
                    f'车辆 {len(self.dash.vehicles)}  左键旋转/滚轮缩放/右键平移')


# ============================================================================
# 控制面板
# ============================================================================
class ControlPanel(QtWidgets.QWidget):
    def __init__(self, dash: DashboardNode, log, parent=None):
        super().__init__(parent)
        self.dash = dash
        self.log = log
        self.setFixedWidth(320)
        self._build()

    def _grp(self, title):
        g = QtWidgets.QGroupBox(title)
        g.setStyleSheet('QGroupBox{font-weight:bold;border:1px solid #556;margin-top:8px;}'
                        'QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}')
        return g, QtWidgets.QFormLayout(g)

    def _build(self):
        root = QtWidgets.QVBoxLayout(self)

        # 信号配时
        g, form = self._grp('信号配时 (秒)')
        self.sp_green = QtWidgets.QDoubleSpinBox(); self.sp_green.setRange(1, 120); self.sp_green.setValue(10)
        self.sp_yellow = QtWidgets.QDoubleSpinBox(); self.sp_yellow.setRange(1, 30); self.sp_yellow.setValue(3)
        self.sp_red = QtWidgets.QDoubleSpinBox(); self.sp_red.setRange(1, 120); self.sp_red.setValue(10)
        form.addRow('绿灯', self.sp_green)
        form.addRow('黄灯', self.sp_yellow)
        form.addRow('红灯', self.sp_red)
        btn_apply = QtWidgets.QPushButton('应用配时')
        btn_apply.clicked.connect(self._on_apply_durations)
        form.addRow(btn_apply)
        root.addWidget(g)

        # 预设
        g, form = self._grp('配时方案')
        self.cmb_preset = QtWidgets.QComboBox()
        self.cmb_preset.addItems(self.dash.presets or ['default'])
        form.addRow('预设', self.cmb_preset)
        btn_preset = QtWidgets.QPushButton('切换预设')
        btn_preset.clicked.connect(self._on_preset)
        form.addRow(btn_preset)
        self.chk_emg = QtWidgets.QCheckBox('紧急全绿 (急救优先)')
        self.chk_emg.toggled.connect(self._on_emergency)
        form.addRow(self.chk_emg)
        root.addWidget(g)

        # 轨迹记录
        g, form = self._grp('轨迹记录')
        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton('开始'); b1.clicked.connect(lambda: self.dash.cmd_record('start', self._cb('记录开始')))
        b2 = QtWidgets.QPushButton('停止'); b2.clicked.connect(lambda: self.dash.cmd_record('stop', self._cb('记录停止')))
        b3 = QtWidgets.QPushButton('保存'); b3.clicked.connect(lambda: self.dash.cmd_record('save', self._cb('记录保存')))
        row.addWidget(b1); row.addWidget(b2); row.addWidget(b3)
        form.addRow(row)
        root.addWidget(g)

        # 轨迹回放
        g, form = self._grp('轨迹回放')
        row = QtWidgets.QHBoxLayout()
        p1 = QtWidgets.QPushButton('载入'); p1.clicked.connect(lambda: self.dash.cmd_playback('load', self._cb('回放载入')))
        p2 = QtWidgets.QPushButton('播放'); p2.clicked.connect(lambda: self.dash.cmd_playback('play', self._cb('回放播放')))
        p3 = QtWidgets.QPushButton('停止'); p3.clicked.connect(lambda: self.dash.cmd_playback('stop', self._cb('回放停止')))
        row.addWidget(p1); row.addWidget(p2); row.addWidget(p3)
        form.addRow(row)
        root.addWidget(g)

        root.addStretch(1)
        hint = QtWidgets.QLabel('F11 全屏 · 大屏可用')
        hint.setStyleSheet('color:#8899aa;')
        hint.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(hint)

    def _cb(self, label):
        def f(ok, msg):
            self.log.append(f'[{label}] {"OK" if ok else "FAIL"} {msg}')
        return f

    def _on_apply_durations(self):
        self.dash.cmd_durations(self.sp_green.value(), self.sp_yellow.value(),
                                self.sp_red.value(), self._cb('配时'))

    def _on_preset(self):
        name = self.cmb_preset.currentText()
        self.dash.cmd_preset(name)
        self.log.append(f'[预设] -> {name}')

    def _on_emergency(self, on):
        self.dash.cmd_emergency(on, self._cb('紧急全绿 ON' if on else '紧急全绿 OFF'))


# ============================================================================
# 日志窗口
# ============================================================================
class LogView(QtWidgets.QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(2000)
        self.setStyleSheet('background:#10161a;color:#cfe8cf;font-family:Consolas,monospace;')

    @QtCore.pyqtSlot(object)
    def on_event(self, ev):
        stamp, region_id, vehicle, kind, speed = ev
        self.appendPlainText(f't={stamp:8.2f}  {kind:5s}  {region_id:16s} {vehicle:16s} {speed:5.2f} m/s')


# ============================================================================
# 主窗口
# ============================================================================
class DashboardWindow(QtWidgets.QMainWindow):
    def __init__(self, dash: DashboardNode):
        super().__init__()
        self.dash = dash
        self.setWindowTitle('V2X 智能交通仿真 · 可视化操作界面')
        self.resize(1400, 860)

        self.scene = SceneView(dash)
        self.scene3d = Scene3DView(dash)
        self.scene_cam = SceneCamView(dash)
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.scene3d)     # index 0 = 3D（QPainter，默认）
        self.stack.addWidget(self.scene)       # index 1 = 2D 俯视
        self.stack.addWidget(self.scene_cam)   # index 2 = 实景3D（Gazebo 渲染）
        self.stack.setCurrentIndex(0)
        self.log = LogView()
        self.panel = ControlPanel(dash, self.log)
        dash.event_listeners.append(self.log.on_event)

        # 视图切换工具条
        toolbar = QtWidgets.QToolBar('视图')
        self.btn_cam = QtWidgets.QPushButton('实景3D(Gazebo)')
        self.btn_3d = QtWidgets.QPushButton('3D 场景')
        self.btn_2d = QtWidgets.QPushButton('2D 俯视')
        self.btn_cam.setCheckable(True)
        self.btn_3d.setCheckable(True); self.btn_2d.setCheckable(True)
        self.btn_3d.setChecked(True)
        self.btn_cam.clicked.connect(lambda: self._switch_view(2, self.btn_cam))
        self.btn_3d.clicked.connect(lambda: self._switch_view(0, self.btn_3d))
        self.btn_2d.clicked.connect(lambda: self._switch_view(1, self.btn_2d))
        toolbar.addWidget(self.btn_cam)
        toolbar.addWidget(self.btn_3d)
        toolbar.addWidget(self.btn_2d)
        self.addToolBar(toolbar)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self.stack)
        splitter.addWidget(self.panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplit.addWidget(splitter)
        vsplit.addWidget(self.log)
        vsplit.setStretchFactor(0, 3)
        vsplit.setStretchFactor(1, 1)
        self.setCentralWidget(vsplit)

        self.status = self.statusBar()
        self.status.showMessage('就绪')

        # 泵送 ROS + 刷新画面
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(60)  # ~16 FPS

        self.log.appendPlainText('[界面] V2X Dashboard 启动')
        self.log.appendPlainText(f'[地图] 路口 {len(dash.nodes)} 路段 {len(dash.segments)} '
                                 f'信号灯 {len(dash.lights)} 区域 {len(dash.regions)}')

    def _switch_view(self, idx, btn):
        self.stack.setCurrentIndex(idx)
        for b in (self.btn_cam, self.btn_3d, self.btn_2d):
            b.setChecked(b is btn)
        if idx == 2:
            self.scene_cam.drive_camera()

    def _tick(self):
        try:
            rclpy.spin_once(self.dash, timeout_sec=0)
        except Exception:  # noqa: BLE001
            pass
        cur = self.stack.currentIndex()
        if cur == 0:
            self.scene3d.update()
        elif cur == 1:
            self.scene.update()
        else:
            self.scene_cam.drive_camera()
            self.scene_cam.update()
        ls = ' | '.join(f'{g}:{self.dash.light_state.get(g, "--")}' for g in self.dash.groups)
        self.status.showMessage(f'车辆 {len(self.dash.vehicles)}   信号灯 {ls}')

    def keyPressEvent(self, e):  # noqa: N802
        if e.key() == QtCore.Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        else:
            super().keyPressEvent(e)

    def closeEvent(self, e):  # noqa: N802
        try:
            self.dash.event_listeners.remove(self.log.on_event)
        except ValueError:
            pass
        super().closeEvent(e)


def main(args=None):
    rclpy.init(args=args)
    dash = DashboardNode()
    # 高 DPI 大屏更清晰
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication([])
    win = DashboardWindow(dash)
    win.show()
    try:
        app.exec_()
    finally:
        dash.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
