#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车仿真控制中心 —— Windows GUI 主窗口。

通过 WebSocket 与 WSL2 中的 ROS2 桥接服务通信，提供：
  - 仿真启停控制
  - 车辆实时状态监控（位姿/速度/传感器）
  - 2D 轨迹可视化
  - 手动驾驶控制
  - 故障注入面板
  - 轨迹回放控制
  - 车辆配置编辑
"""
import sys
import os
import json
import time
import subprocess
import threading
from typing import Optional, Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QGroupBox, QLabel, QPushButton, QTextEdit, QTableWidget,
    QTableWidgetItem, QComboBox, QDoubleSpinBox, QCheckBox, QFileDialog,
    QStatusBar, QSplitter, QFrame, QProgressBar, QMessageBox, QHeaderView
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPalette

try:
    import websocket
    HAS_WS = True
except ImportError:
    HAS_WS = False


# ── WSL 接口 ──────────────────────────────────────────────────
class WSLInterface:
    def __init__(self):
        self.sim_running = False
        self.bridge_running = False
        self.bridge_process = None
        self.sim_process = None

    def wsl_cmd(self, cmd: str, timeout=10) -> tuple:
        try:
            r = subprocess.run(
                ['wsl', '-e', 'bash', '-c', cmd],
                capture_output=True, text=True, timeout=timeout,
                cwd=os.path.expanduser('~'))
            return r.returncode, r.stdout, r.stderr
        except subprocess.TimeoutExpired:
            return -1, '', 'timeout'
        except Exception as e:
            return -1, '', str(e)

    def start_bridge(self, port=8765):
        if self.bridge_running:
            return True, '桥接服务已在运行'
        cmd = (f'cd ~/v2x_ws && source install/setup.bash && '
               f'python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/'
               f'smart_car_gui/wsl_bridge/ros2_bridge.py {port}')
        self.bridge_process = subprocess.Popen(
            ['wsl', '-e', 'bash', '-c', cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.bridge_running = True
        time.sleep(1)
        return True, '桥接服务已启动'

    def stop_bridge(self):
        if self.bridge_process:
            self.bridge_process.terminate()
            self.bridge_process = None
        self.bridge_running = False
        return True, '桥接服务已停止'

    def start_simulation(self, gui=False, nav2=False):
        if self.sim_running:
            return True, '仿真已在运行'
        gui_str = 'true' if gui else 'false'
        nav2_str = 'true' if nav2 else 'false'
        cmd = (f'cd ~/v2x_ws && source install/setup.bash && '
               f'DISPLAY=:0 ros2 launch smart_car_sim smart_car_sim.launch.py '
               f'gui:={gui_str} nav2:={nav2_str} record:=true')
        self.sim_process = subprocess.Popen(
            ['wsl', '-e', 'bash', '-c', cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.sim_running = True
        return True, '仿真已启动'

    def stop_simulation(self):
        if self.sim_process:
            self.sim_process.terminate()
            self.sim_process = None
        self.wsl_cmd('pkill -f gzserver; pkill -f smart_car_sim')
        self.sim_running = False
        return True, '仿真已停止'

    def get_wsl_status(self) -> dict:
        rc, out, _ = self.wsl_cmd('pgrep -a gzserver | head -1')
        gz = out.strip() if rc == 0 else ''
        rc2, out2, _ = self.wsl_cmd('pgrep -a ros2_bridge | head -1')
        br = out2.strip() if rc2 == 0 else ''
        return {
            'gzserver': bool(gz),
            'bridge': bool(br) or self.bridge_running,
            'sim': self.sim_running,
        }


# ── WebSocket 客户端线程 ──────────────────────────────────────
class WSClientThread(QThread):
    state_received = pyqtSignal(dict)
    connected = pyqtSignal(bool)
    ack_received = pyqtSignal(dict)

    def __init__(self, url='ws://localhost:8765'):
        super().__init__()
        self.url = url
        self.ws = None
        self.running = False

    def run(self):
        if not HAS_WS:
            self.connected.emit(False)
            return
        self.running = True
        while self.running:
            try:
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                    on_open=self._on_open)
                self.ws.run_forever(ping_interval=5, ping_timeout=3)
            except Exception as e:
                print(f'[ws] 错误: {e}')
            if self.running:
                time.sleep(2)

    def _on_open(self, ws):
        print('[ws] 已连接')
        self.connected.emit(True)

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data.get('type') == 'state':
                self.state_received.emit(data)
            elif data.get('type') == 'ack':
                self.ack_received.emit(data)
        except json.JSONDecodeError:
            pass

    def _on_error(self, ws, error):
        self.connected.emit(False)

    def _on_close(self, ws, *args):
        self.connected.emit(False)

    def send(self, msg: dict):
        if self.ws and self.ws.sock and self.ws.sock.connected:
            self.ws.send(json.dumps(msg))

    def stop(self):
        self.running = False
        if self.ws:
            self.ws.close()


# ── 2D 轨迹可视化组件 ─────────────────────────────────────────
class ArenaView(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(500, 500)
        self.vehicles = {}
        self.arena_size = 50.0
        self.trail_points = {}
        self.show_trails = True
        self.max_trail = 200

    def update_vehicles(self, vehicles: dict):
        self.vehicles = vehicles
        for name, v in vehicles.items():
            if name not in self.trail_points:
                self.trail_points[name] = []
            trail = self.trail_points[name]
            trail.append((v['x'], v['y']))
            if len(trail) > self.max_trail:
                trail.pop(0)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        margin = 40
        scale = min(w - 2*margin, h - 2*margin) / self.arena_size
        cx, cy = w / 2, h / 2

        painter.fillRect(0, 0, w, h, QColor(30, 30, 40))

        half = self.arena_size / 2
        arena_rect = QRectF(cx - half*scale, cy - half*scale,
                            self.arena_size*scale, self.arena_size*scale)
        painter.setPen(QPen(QColor(80, 80, 100), 2))
        painter.drawRect(arena_rect)

        painter.setPen(QPen(QColor(50, 50, 60), 1, Qt.DotLine))
        for i in range(int(-half), int(half)+1, 5):
            x = cx + i * scale
            painter.drawLine(int(x), int(cy - half*scale),
                             int(x), int(cy + half*scale))
            y = cy + i * scale
            painter.drawLine(int(cx - half*scale), int(y),
                             int(cx + half*scale), int(y))

        painter.setPen(QPen(QColor(100, 100, 120), 1))
        painter.setFont(QFont('Consolas', 8))
        painter.drawText(int(cx - 15), int(cy - half*scale - 5), 'N')
        painter.drawText(int(cx - half*scale - 20), int(cy + 5), 'W')
        painter.drawText(int(cx + half*scale + 5), int(cy + 5), 'E')
        painter.drawText(int(cx - 15), int(cy + half*scale + 15), 'S')

        obstacle_rect = QRectF(cx - 1*scale, cy - 1*scale, 2*scale, 2*scale)
        painter.setBrush(QBrush(QColor(120, 60, 60, 150)))
        painter.setPen(QPen(QColor(180, 80, 80), 1))
        painter.drawRect(obstacle_rect)

        colors = [QColor(0, 200, 100), QColor(100, 150, 255),
                  QColor(255, 180, 50), QColor(200, 100, 255),
                  QColor(255, 100, 150)]
        for idx, (name, v) in enumerate(self.vehicles.items()):
            color = colors[idx % len(colors)]
            vx, vy = v['x'], v['y']
            yaw = v.get('yaw', 0)
            px = cx + vx * scale
            py = cy - vy * scale

            if self.show_trails and name in self.trail_points:
                trail = self.trail_points[name]
                painter.setPen(QPen(color, 1, Qt.SolidLine))
                for i in range(1, len(trail)):
                    x1 = cx + trail[i-1][0] * scale
                    y1 = cy - trail[i-1][1] * scale
                    x2 = cx + trail[i][0] * scale
                    y2 = cy - trail[i][1] * scale
                    painter.drawLine(int(x1), int(y1), int(x2), int(y2))

            car_size = 0.4 * scale
            import math
            dx = math.cos(yaw) * car_size
            dy = math.sin(yaw) * car_size
            painter.setPen(QPen(color, 2))
            painter.drawLine(int(px), int(py),
                             int(px + dx), int(py - dy))

            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.darker(120), 1))
            r = 5
            painter.drawEllipse(int(px - r), int(py - r), 2*r, 2*r)

            painter.setPen(QPen(QColor(220, 220, 230), 1))
            painter.setFont(QFont('Consolas', 9, QFont.Bold))
            painter.drawText(int(px + 8), int(py - 8), name)

        painter.end()


# ── 主窗口 ────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('智能小车仿真控制中心 v1.0')
        self.setMinimumSize(1200, 800)

        self.wsl = WSLInterface()
        self.ws_client = WSClientThread()
        self.ws_client.state_received.connect(self.on_state)
        self.ws_client.connected.connect(self.on_ws_connected)
        self.ws_client.ack_received.connect(self.on_ack)

        self.current_state = {}
        self.init_ui()

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(5000)

        self.refresh_status()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        self.tabs.addTab(self.create_sim_tab(), '仿真控制')
        self.tabs.addTab(self.create_monitor_tab(), '车辆监控')
        self.tabs.addTab(self.create_drive_tab(), '手动驾驶')
        self.tabs.addTab(self.create_fault_tab(), '故障模拟')
        self.tabs.addTab(self.create_playback_tab(), '轨迹回放')
        self.tabs.addTab(self.create_config_tab(), '车辆配置')

        self.statusBar().showMessage('就绪')

    def create_sim_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        grp_launch = QGroupBox('仿真启动')
        g_layout = QVBoxLayout()

        row1 = QHBoxLayout()
        self.chk_gui = QCheckBox('启用 Gazebo GUI')
        self.chk_gui.setChecked(False)
        row1.addWidget(self.chk_gui)
        self.chk_nav2 = QCheckBox('启用 Nav2 导航')
        row1.addWidget(self.chk_nav2)
        g_layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.btn_start_sim = QPushButton('启动仿真')
        self.btn_start_sim.setStyleSheet('QPushButton { background: #2d8; color: white; padding: 8px; font-weight: bold; }')
        self.btn_start_sim.clicked.connect(self.on_start_sim)
        row2.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton('停止仿真')
        self.btn_stop_sim.setStyleSheet('QPushButton { background: #d44; color: white; padding: 8px; font-weight: bold; }')
        self.btn_stop_sim.clicked.connect(self.on_stop_sim)
        self.btn_stop_sim.setEnabled(False)
        row2.addWidget(self.btn_stop_sim)
        g_layout.addLayout(row2)

        grp_launch.setLayout(g_layout)
        layout.addWidget(grp_launch)

        grp_bridge = QGroupBox('桥接服务')
        b_layout = QVBoxLayout()
        b_row = QHBoxLayout()
        self.btn_start_bridge = QPushButton('启动桥接')
        self.btn_start_bridge.clicked.connect(self.on_start_bridge)
        b_row.addWidget(self.btn_start_bridge)
        self.btn_stop_bridge = QPushButton('停止桥接')
        self.btn_stop_bridge.clicked.connect(self.on_stop_bridge)
        b_row.addWidget(self.btn_stop_bridge)
        self.lbl_bridge_status = QLabel('未连接')
        self.lbl_bridge_status.setStyleSheet('color: gray;')
        b_row.addWidget(self.lbl_bridge_status)
        b_layout.addLayout(b_row)
        grp_bridge.setLayout(b_layout)
        layout.addWidget(grp_bridge)

        grp_status = QGroupBox('系统状态')
        s_layout = QVBoxLayout()
        self.lbl_gzserver = QLabel('Gazebo Server: 未知')
        self.lbl_sim_status = QLabel('仿真状态: 未知')
        self.lbl_vehicle_count = QLabel('车辆数: 0')
        s_layout.addWidget(self.lbl_gzserver)
        s_layout.addWidget(self.lbl_sim_status)
        s_layout.addWidget(self.lbl_vehicle_count)
        grp_status.setLayout(s_layout)
        layout.addWidget(grp_status)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumHeight(150)
        layout.addWidget(QLabel('日志:'))
        layout.addWidget(self.log_output)

        layout.addStretch()
        return w

    def create_monitor_tab(self) -> QWidget:
        w = QWidget()
        layout = QHBoxLayout(w)

        left = QVBoxLayout()
        self.vehicle_table = QTableWidget()
        self.vehicle_table.setColumnCount(8)
        self.vehicle_table.setHorizontalHeaderLabels(
            ['名称', 'X(m)', 'Y(m)', 'Yaw(°)', 'Vx(m/s)', '最近障碍(m)',
             '里程计X', '里程计Y'])
        self.vehicle_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        left.addWidget(self.vehicle_table)
        layout.addLayout(left, 3)

        right = QVBoxLayout()
        self.arena_view = ArenaView()
        right.addWidget(self.arena_view)
        self.chk_trails = QCheckBox('显示轨迹')
        self.chk_trails.setChecked(True)
        self.chk_trails.stateChanged.connect(
            lambda s: setattr(self.arena_view, 'show_trails', bool(s)))
        right.addWidget(self.chk_trails)
        layout.addLayout(right, 4)

        return w

    def create_drive_tab(self) -> QWidget:
        w = QWidget()
        layout = QHBoxLayout(w)

        left = QVBoxLayout()
        grp_select = QGroupBox('选择车辆')
        gs_layout = QVBoxLayout()
        self.cmb_vehicle = QComboBox()
        gs_layout.addWidget(self.cmb_vehicle)
        grp_select.setLayout(gs_layout)
        left.addWidget(grp_select)

        grp_vel = QGroupBox('速度控制')
        gv_layout = QVBoxLayout()
        row_linear = QHBoxLayout()
        row_linear.addWidget(QLabel('线速度 (m/s):'))
        self.spin_linear = QDoubleSpinBox()
        self.spin_linear.setRange(-1.0, 2.0)
        self.spin_linear.setSingleStep(0.1)
        self.spin_linear.setValue(0.0)
        row_linear.addWidget(self.spin_linear)
        gv_layout.addLayout(row_linear)

        row_angular = QHBoxLayout()
        row_angular.addWidget(QLabel('角速度 (rad/s):'))
        self.spin_angular = QDoubleSpinBox()
        self.spin_angular.setRange(-3.14, 3.14)
        self.spin_angular.setSingleStep(0.1)
        self.spin_angular.setValue(0.0)
        row_angular.addWidget(self.spin_angular)
        gv_layout.addLayout(row_angular)

        btn_row = QHBoxLayout()
        self.btn_send_vel = QPushButton('发送')
        self.btn_send_vel.setStyleSheet('QPushButton { background: #48f; color: white; padding: 6px; }')
        self.btn_send_vel.clicked.connect(self.on_send_cmd_vel)
        btn_row.addWidget(self.btn_send_vel)
        self.btn_stop_vel = QPushButton('停止')
        self.btn_stop_vel.setStyleSheet('QPushButton { background: #d44; color: white; padding: 6px; }')
        self.btn_stop_vel.clicked.connect(self.on_stop_vel)
        btn_row.addWidget(self.btn_stop_vel)
        gv_layout.addLayout(btn_row)

        grp_vel.setLayout(gv_layout)
        left.addWidget(grp_vel)

        grp_presets = QGroupBox('预设动作')
        gp_layout = QVBoxLayout()
        for label, lv, av in [('前进', 0.5, 0), ('后退', -0.3, 0),
                               ('左转', 0.3, 0.5), ('右转', 0.3, -0.5)]:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, l=lv, a=av: self._preset_vel(l, a))
            gp_layout.addWidget(btn)
        grp_presets.setLayout(gp_layout)
        left.addWidget(grp_presets)

        left.addStretch()
        layout.addLayout(left, 1)

        right = QVBoxLayout()
        self.drive_arena = ArenaView()
        right.addWidget(self.drive_arena)
        layout.addLayout(right, 2)

        return w

    def create_fault_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        grp_select = QGroupBox('选择车辆')
        gs_layout = QHBoxLayout()
        gs_layout.addWidget(QLabel('车辆:'))
        self.cmb_fault_vehicle = QComboBox()
        gs_layout.addWidget(self.cmb_fault_vehicle)
        grp_select.setLayout(gs_layout)
        layout.addWidget(grp_select)

        grp_faults = QGroupBox('故障类型')
        gf_layout = QVBoxLayout()
        self.fault_checks = {}
        for fault_id, label, desc in [
            ('lidar_off', '激光雷达关闭', '停止发布 scan 数据'),
            ('camera_off', '相机关闭', '停止发布图像数据'),
            ('imu_drift', 'IMU 漂移', '注入姿态漂移误差'),
            ('odom_slip', '里程计打滑', '里程计与实际位姿偏差'),
        ]:
            row = QHBoxLayout()
            chk = QCheckBox(label)
            self.fault_checks[fault_id] = chk
            row.addWidget(chk)
            row.addWidget(QLabel(desc))
            btn = QPushButton('应用')
            btn.clicked.connect(lambda _, f=fault_id: self.on_apply_fault(f))
            row.addWidget(btn)
            gf_layout.addLayout(row)

        btn_reset = QPushButton('重置所有故障')
        btn_reset.setStyleSheet('QPushButton { background: #2a2; color: white; padding: 6px; }')
        btn_reset.clicked.connect(self.on_reset_faults)
        gf_layout.addWidget(btn_reset)

        grp_faults.setLayout(gf_layout)
        layout.addWidget(grp_faults)

        self.fault_log = QTextEdit()
        self.fault_log.setReadOnly(True)
        self.fault_log.setMaximumHeight(120)
        layout.addWidget(QLabel('故障日志:'))
        layout.addWidget(self.fault_log)

        layout.addStretch()
        return w

    def create_playback_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        grp_ctrl = QGroupBox('回放控制')
        gc_layout = QVBoxLayout()
        row1 = QHBoxLayout()
        self.btn_load_traj = QPushButton('加载轨迹文件')
        self.btn_load_traj.clicked.connect(self.on_load_trajectory)
        row1.addWidget(self.btn_load_traj)
        self.lbl_traj_file = QLabel('未加载')
        row1.addWidget(self.lbl_traj_file)
        gc_layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.btn_play = QPushButton('播放')
        self.btn_play.setStyleSheet('QPushButton { background: #2d8; color: white; padding: 6px; }')
        self.btn_play.clicked.connect(self.on_play_trajectory)
        row2.addWidget(self.btn_play)
        self.btn_pause = QPushButton('暂停')
        self.btn_pause.clicked.connect(self.on_pause_trajectory)
        row2.addWidget(self.btn_pause)
        self.btn_stop_play = QPushButton('停止')
        self.btn_stop_play.clicked.connect(self.on_stop_trajectory)
        row2.addWidget(self.btn_stop_play)
        gc_layout.addLayout(row2)

        grp_ctrl.setLayout(gc_layout)
        layout.addWidget(grp_ctrl)

        self.playback_arena = ArenaView()
        layout.addWidget(self.playback_arena)

        self.trajectory_data = None
        layout.addStretch()
        return w

    def create_config_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        grp_file = QGroupBox('配置文件')
        gf_layout = QHBoxLayout()
        self.lbl_config_path = QLabel('smart_car_config.yaml')
        gf_layout.addWidget(self.lbl_config_path)
        btn_browse = QPushButton('浏览...')
        btn_browse.clicked.connect(self.on_browse_config)
        gf_layout.addWidget(btn_browse)
        btn_reload = QPushButton('重新加载')
        btn_reload.clicked.connect(self.on_reload_config)
        gf_layout.addWidget(btn_reload)
        grp_file.setLayout(gf_layout)
        layout.addWidget(grp_file)

        self.config_table = QTableWidget()
        self.config_table.setColumnCount(6)
        self.config_table.setHorizontalHeaderLabels(
            ['实例名', '预设', 'X', 'Y', '颜色', '启用'])
        self.config_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        layout.addWidget(self.config_table)

        btn_row = QHBoxLayout()
        btn_add = QPushButton('添加车辆')
        btn_add.clicked.connect(self.on_add_vehicle)
        btn_row.addWidget(btn_add)
        btn_save = QPushButton('保存配置')
        btn_save.setStyleSheet('QPushButton { background: #48f; color: white; padding: 6px; }')
        btn_save.clicked.connect(self.on_save_config)
        btn_row.addWidget(btn_save)
        layout.addLayout(btn_row)

        self.config_data = None
        self.config_path = ''
        return w

    # ── 槽函数 ────────────────────────────────────────────────
    def log(self, msg: str):
        ts = time.strftime('%H:%M:%S')
        self.log_output.append(f'[{ts}] {msg}')

    def on_start_sim(self):
        gui = self.chk_gui.isChecked()
        nav2 = self.chk_nav2.isChecked()
        self.log(f'启动仿真 (GUI={gui}, Nav2={nav2})...')
        ok, msg = self.wsl.start_simulation(gui=gui, nav2=nav2)
        self.log(msg)
        self.btn_start_sim.setEnabled(False)
        self.btn_stop_sim.setEnabled(True)
        self.statusBar().showMessage('仿真启动中...')

    def on_stop_sim(self):
        self.log('停止仿真...')
        ok, msg = self.wsl.stop_simulation()
        self.log(msg)
        self.btn_start_sim.setEnabled(True)
        self.btn_stop_sim.setEnabled(False)
        self.statusBar().showMessage('仿真已停止')

    def on_start_bridge(self):
        self.log('启动桥接服务...')
        ok, msg = self.wsl.start_bridge()
        self.log(msg)
        if ok:
            self.ws_client = WSClientThread()
            self.ws_client.state_received.connect(self.on_state)
            self.ws_client.connected.connect(self.on_ws_connected)
            self.ws_client.ack_received.connect(self.on_ack)
            self.ws_client.start()

    def on_stop_bridge(self):
        self.ws_client.stop()
        self.wsl.stop_bridge()
        self.lbl_bridge_status.setText('未连接')
        self.lbl_bridge_status.setStyleSheet('color: gray;')
        self.log('桥接服务已停止')

    def on_ws_connected(self, connected: bool):
        if connected:
            self.lbl_bridge_status.setText('已连接')
            self.lbl_bridge_status.setStyleSheet('color: green; font-weight: bold;')
            self.log('已连接到 ROS2 桥接服务')
            self.statusBar().showMessage('桥接已连接')
        else:
            self.lbl_bridge_status.setText('断开')
            self.lbl_bridge_status.setStyleSheet('color: red;')

    def on_state(self, data: dict):
        self.current_state = data
        vehicles = data.get('vehicles', {})
        self.lbl_vehicle_count.setText(f'车辆数: {len(vehicles)}')

        self.vehicle_table.setRowCount(len(vehicles))
        names = sorted(vehicles.keys())
        for row, name in enumerate(names):
            v = vehicles[name]
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(name))
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(f"{v['x']:.2f}"))
            self.vehicle_table.setItem(row, 2, QTableWidgetItem(f"{v['y']:.2f}"))
            import math
            yaw_deg = math.degrees(v.get('yaw', 0))
            self.vehicle_table.setItem(row, 3, QTableWidgetItem(f"{yaw_deg:.1f}"))
            self.vehicle_table.setItem(row, 4, QTableWidgetItem(f"{v['vx']:.2f}"))
            self.vehicle_table.setItem(row, 5, QTableWidgetItem(f"{v['scan_min_range']:.2f}"))
            self.vehicle_table.setItem(row, 6, QTableWidgetItem(f"{v['odom_x']:.2f}"))
            self.vehicle_table.setItem(row, 7, QTableWidgetItem(f"{v['odom_y']:.2f}"))

        self.arena_view.update_vehicles(vehicles)
        self.drive_arena.update_vehicles(vehicles)

        self.cmb_vehicle.clear()
        self.cmb_fault_vehicle.clear()
        for name in names:
            self.cmb_vehicle.addItem(name)
            self.cmb_fault_vehicle.addItem(name)

    def on_ack(self, data: dict):
        cmd = data.get('cmd', '')
        if cmd == 'cmd_vel':
            self.log(f"速度指令已发送 -> {data.get('vehicle')}")
        elif cmd == 'fault':
            self.log(f"故障指令已发送 -> {data.get('vehicle')}: {data.get('fault')}")
            self.fault_log.append(f"[{time.strftime('%H:%M:%S')}] "
                                  f"{data.get('vehicle')}: {data.get('fault')}")

    def on_send_cmd_vel(self):
        vehicle = self.cmb_vehicle.currentText()
        if not vehicle:
            return
        linear = self.spin_linear.value()
        angular = self.spin_angular.value()
        self.ws_client.send({
            'type': 'cmd_vel',
            'vehicle': vehicle,
            'linear': linear,
            'angular': angular,
        })

    def on_stop_vel(self):
        self.spin_linear.setValue(0)
        self.spin_angular.setValue(0)
        self.on_send_cmd_vel()

    def _preset_vel(self, linear, angular):
        self.spin_linear.setValue(linear)
        self.spin_angular.setValue(angular)
        self.on_send_cmd_vel()

    def on_apply_fault(self, fault_type: str):
        vehicle = self.cmb_fault_vehicle.currentText()
        if not vehicle:
            return
        self.ws_client.send({
            'type': 'fault',
            'vehicle': vehicle,
            'fault_type': fault_type,
        })

    def on_reset_faults(self):
        vehicle = self.cmb_fault_vehicle.currentText()
        if not vehicle:
            return
        self.ws_client.send({
            'type': 'fault',
            'vehicle': vehicle,
            'fault_type': 'reset',
        })
        for chk in self.fault_checks.values():
            chk.setChecked(False)

    def on_load_trajectory(self):
        fpath, _ = QFileDialog.getOpenFileName(
            self, '选择轨迹文件',
            os.path.expanduser('~'),
            'JSON 文件 (*.json)')
        if fpath:
            self.lbl_traj_file.setText(os.path.basename(fpath))
            self.ws_client.send({
                'type': 'playback',
                'action': 'load',
                'file': fpath,
            })
            self.ws_client.send({
                'type': 'get_trajectory',
                'file': fpath,
            })

    def on_play_trajectory(self):
        self.ws_client.send({'type': 'playback', 'action': 'play'})

    def on_pause_trajectory(self):
        self.ws_client.send({'type': 'playback', 'action': 'pause'})

    def on_stop_trajectory(self):
        self.ws_client.send({'type': 'playback', 'action': 'stop'})

    def on_browse_config(self):
        fpath, _ = QFileDialog.getOpenFileName(
            self, '选择配置文件', '', 'YAML 文件 (*.yaml)')
        if fpath:
            self.config_path = fpath
            self.lbl_config_path.setText(os.path.basename(fpath))
            self.on_reload_config()

    def on_reload_config(self):
        if not self.config_path:
            default = os.path.join(os.path.dirname(__file__), '..',
                                   'v2x_traffic_sim', 'src', 'smart_car_sim',
                                   'config', 'smart_car_config.yaml')
            if os.path.exists(default):
                self.config_path = default
        if self.config_path and os.path.exists(self.config_path):
            try:
                import yaml
                with open(self.config_path) as f:
                    self.config_data = yaml.safe_load(f)
                self._populate_config_table()
                self.log(f'已加载配置: {self.config_path}')
            except Exception as e:
                self.log(f'加载配置失败: {e}')

    def _populate_config_table(self):
        if not self.config_data:
            return
        vehicles = self.config_data.get('vehicles', [])
        self.config_table.setRowCount(len(vehicles))
        for row, v in enumerate(vehicles):
            self.config_table.setItem(row, 0, QTableWidgetItem(v.get('name', '')))
            self.config_table.setItem(row, 1, QTableWidgetItem(v.get('preset', '')))
            pose = v.get('spawn_pose', [0, 0, 0])
            self.config_table.setItem(row, 2, QTableWidgetItem(str(pose[0])))
            self.config_table.setItem(row, 3, QTableWidgetItem(str(pose[1])))
            self.config_table.setItem(row, 4, QTableWidgetItem(v.get('color', '')))
            enabled = '是' if v.get('enabled', True) else '否'
            self.config_table.setItem(row, 5, QTableWidgetItem(enabled))

    def on_add_vehicle(self):
        if not self.config_data:
            return
        row = self.config_table.rowCount()
        self.config_table.setRowCount(row + 1)
        new_v = {
            'name': f'smart_car_{row+1}',
            'preset': 'medium_car',
            'spawn_pose': [0, -18, 0],
            'color': '#ff0000',
            'enabled': True,
        }
        self.config_data.setdefault('vehicles', []).append(new_v)
        self._populate_config_table()

    def on_save_config(self):
        if not self.config_path or not self.config_data:
            self.log('没有可保存的配置')
            return
        vehicles = []
        for row in range(self.config_table.rowCount()):
            v = {
                'name': self.config_table.item(row, 0).text(),
                'preset': self.config_table.item(row, 1).text(),
                'spawn_pose': [
                    float(self.config_table.item(row, 2).text()),
                    float(self.config_table.item(row, 3).text()),
                    0,
                ],
                'color': self.config_table.item(row, 4).text(),
                'enabled': self.config_table.item(row, 5).text() == '是',
            }
            vehicles.append(v)
        self.config_data['vehicles'] = vehicles
        try:
            import yaml
            with open(self.config_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.config_data, f, allow_unicode=True,
                          default_flow_style=False)
            self.log(f'配置已保存: {self.config_path}')
            self.ws_client.send({
                'type': 'save_config',
                'file': self.config_path,
                'data': self.config_data,
            })
        except Exception as e:
            self.log(f'保存失败: {e}')

    def refresh_status(self):
        status = self.wsl.get_wsl_status()
        if status['gzserver']:
            self.lbl_gzserver.setText('Gazebo Server: 运行中')
            self.lbl_gzserver.setStyleSheet('color: green;')
        else:
            self.lbl_gzserver.setText('Gazebo Server: 未运行')
            self.lbl_gzserver.setStyleSheet('color: gray;')

        if status['sim'] or status['gzserver']:
            self.lbl_sim_status.setText('仿真状态: 运行中')
            self.lbl_sim_status.setStyleSheet('color: green;')
            self.btn_start_sim.setEnabled(False)
            self.btn_stop_sim.setEnabled(True)
        else:
            self.lbl_sim_status.setText('仿真状态: 已停止')
            self.lbl_sim_status.setStyleSheet('color: gray;')
            self.btn_start_sim.setEnabled(True)
            self.btn_stop_sim.setEnabled(False)

    def closeEvent(self, event):
        self.ws_client.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(35, 35, 35))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
