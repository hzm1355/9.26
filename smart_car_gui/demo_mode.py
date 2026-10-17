#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车仿真控制中心 —— 演示模式（无需 WSL/ROS2）。

用于测试 GUI 界面功能，使用模拟数据。
"""
import sys
import os
import json
import time
import math
import random
import threading
from typing import Dict

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QGroupBox, QLabel, QPushButton, QTextEdit, QTableWidget,
    QTableWidgetItem, QComboBox, QDoubleSpinBox, QCheckBox, QFileDialog,
    QStatusBar, QSplitter, QFrame, QProgressBar, QMessageBox, QHeaderView
)
from PyQt5.QtCore import Qt, QTimer, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPalette


# ── 模拟数据生成器 ────────────────────────────────────────────
class MockSimulator:
    def __init__(self):
        self.vehicles = {
            'smart_car_1': {'x': 0, 'y': -18, 'yaw': 1.57, 'vx': 0, 'vy': 0,
                            'scan_min_range': 5.0, 'odom_x': 0, 'odom_y': 0, 'odom_yaw': 1.57},
            'smart_car_2': {'x': -15, 'y': -18, 'yaw': 0, 'vx': 0, 'vy': 0,
                            'scan_min_range': 8.0, 'odom_x': -15, 'odom_y': -18, 'odom_yaw': 0},
            'smart_car_3': {'x': 15, 'y': -18, 'yaw': 3.14, 'vx': 0, 'vy': 0,
                            'scan_min_range': 3.0, 'odom_x': 15, 'odom_y': -18, 'odom_yaw': 3.14},
            'tb3_1': {'x': -8, 'y': -18, 'yaw': 0.5, 'vx': 0, 'vy': 0,
                      'scan_min_range': 2.0, 'odom_x': -8, 'odom_y': -18, 'odom_yaw': 0.5},
            'tb3_2': {'x': 8, 'y': -18, 'yaw': -0.5, 'vx': 0, 'vy': 0,
                      'scan_min_range': 4.0, 'odom_x': 8, 'odom_y': -18, 'odom_yaw': -0.5},
        }
        self.running = False
        self.time_step = 0

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def update(self):
        if not self.running:
            return
        self.time_step += 1
        for name, v in self.vehicles.items():
            v['vx'] = 0.3 * math.sin(self.time_step * 0.02 + hash(name) % 10)
            v['x'] += v['vx'] * 0.1 * math.cos(v['yaw'])
            v['y'] += v['vx'] * 0.1 * math.sin(v['yaw'])
            v['yaw'] += 0.01 * math.sin(self.time_step * 0.01 + hash(name) % 5)
            v['odom_x'] = v['x']
            v['odom_y'] = v['y']
            v['odom_yaw'] = v['yaw']
            v['scan_min_range'] = 2.0 + 3.0 * abs(math.sin(self.time_step * 0.05 + hash(name) % 7))

    def get_state(self) -> dict:
        return {
            'type': 'state',
            't': time.time(),
            'sim_time': self.time_step * 0.1,
            'vehicles': {n: {**v, 'last_update': time.time()} for n, v in self.vehicles.items()},
            'region_events': [],
            'is_recording': False,
            'is_playing': False,
        }


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


# ── 主窗口（演示模式） ────────────────────────────────────────
class DemoMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('智能小车仿真控制中心 v1.0 [演示模式]')
        self.setMinimumSize(1200, 800)

        self.sim = MockSimulator()
        self.sim.start()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_simulation)
        self.update_timer.start(100)

        self.init_ui()
        self.statusBar().showMessage('演示模式 - 模拟数据运行中')

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

    def create_sim_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        grp_launch = QGroupBox('仿真控制（演示模式）')
        g_layout = QVBoxLayout()

        info = QLabel('当前为演示模式，使用模拟数据。\n'
                      '实际使用时需要 WSL2 + ROS2 环境。')
        info.setStyleSheet('color: #ffa; padding: 10px;')
        g_layout.addWidget(info)

        row2 = QHBoxLayout()
        self.btn_start_sim = QPushButton('启动模拟')
        self.btn_start_sim.setStyleSheet('QPushButton { background: #2d8; color: white; padding: 8px; font-weight: bold; }')
        self.btn_start_sim.clicked.connect(self.on_start_sim)
        row2.addWidget(self.btn_start_sim)

        self.btn_stop_sim = QPushButton('停止模拟')
        self.btn_stop_sim.setStyleSheet('QPushButton { background: #d44; color: white; padding: 8px; font-weight: bold; }')
        self.btn_stop_sim.clicked.connect(self.on_stop_sim)
        row2.addWidget(self.btn_stop_sim)
        g_layout.addLayout(row2)

        grp_launch.setLayout(g_layout)
        layout.addWidget(grp_launch)

        grp_status = QGroupBox('系统状态')
        s_layout = QVBoxLayout()
        self.lbl_sim_status = QLabel('模拟状态: 运行中')
        self.lbl_sim_status.setStyleSheet('color: green;')
        self.lbl_vehicle_count = QLabel('车辆数: 5')
        self.lbl_sim_time = QLabel('仿真时间: 0.0s')
        s_layout.addWidget(self.lbl_sim_status)
        s_layout.addWidget(self.lbl_vehicle_count)
        s_layout.addWidget(self.lbl_sim_time)
        grp_status.setLayout(s_layout)
        layout.addWidget(grp_status)

        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setMaximumHeight(150)
        self.log_output.append('[演示模式] GUI 界面功能测试')
        self.log_output.append('[演示模式] 所有标签页均可操作')
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
        self.cmb_vehicle.addItems(['smart_car_1', 'smart_car_2', 'smart_car_3',
                                    'tb3_1', 'tb3_2'])
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
        self.cmb_fault_vehicle.addItems(['smart_car_1', 'smart_car_2', 'smart_car_3',
                                          'tb3_1', 'tb3_2'])
        gs_layout.addWidget(self.cmb_fault_vehicle)
        grp_select.setLayout(gs_layout)
        layout.addWidget(grp_select)

        grp_faults = QGroupBox('故障类型（演示）')
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

        layout.addStretch()
        return w

    def create_config_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        info = QLabel('车辆配置编辑功能\n\n'
                      '可以添加/删除车辆，修改预设、位置、颜色等参数。\n'
                      '配置保存为 YAML 格式，供仿真启动时读取。')
        info.setStyleSheet('color: #aaf; padding: 10px;')
        layout.addWidget(info)

        self.config_table = QTableWidget()
        self.config_table.setColumnCount(6)
        self.config_table.setHorizontalHeaderLabels(
            ['实例名', '预设', 'X', 'Y', '颜色', '启用'])
        self.config_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)

        demo_data = [
            ('smart_car_1', 'medium_car', '0', '-18', '#ff0000', '是'),
            ('smart_car_2', 'delivery_bot', '-15', '-18', '#00ff00', '是'),
            ('smart_car_3', 'bus', '15', '-18', '#0000ff', '是'),
            ('tb3_1', 'tb3_burger', '-8', '-18', '#ffff00', '是'),
            ('tb3_2', 'tb3_waffle_pi', '8', '-18', '#ff00ff', '是'),
        ]
        self.config_table.setRowCount(len(demo_data))
        for row, data in enumerate(demo_data):
            for col, val in enumerate(data):
                self.config_table.setItem(row, col, QTableWidgetItem(val))

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

        return w

    def update_simulation(self):
        self.sim.update()
        state = self.sim.get_state()
        vehicles = state.get('vehicles', {})

        self.lbl_vehicle_count.setText(f'车辆数: {len(vehicles)}')
        self.lbl_sim_time.setText(f'仿真时间: {state["sim_time"]:.1f}s')

        self.vehicle_table.setRowCount(len(vehicles))
        names = sorted(vehicles.keys())
        for row, name in enumerate(names):
            v = vehicles[name]
            self.vehicle_table.setItem(row, 0, QTableWidgetItem(name))
            self.vehicle_table.setItem(row, 1, QTableWidgetItem(f"{v['x']:.2f}"))
            self.vehicle_table.setItem(row, 2, QTableWidgetItem(f"{v['y']:.2f}"))
            yaw_deg = math.degrees(v.get('yaw', 0))
            self.vehicle_table.setItem(row, 3, QTableWidgetItem(f"{yaw_deg:.1f}"))
            self.vehicle_table.setItem(row, 4, QTableWidgetItem(f"{v['vx']:.2f}"))
            self.vehicle_table.setItem(row, 5, QTableWidgetItem(f"{v['scan_min_range']:.2f}"))
            self.vehicle_table.setItem(row, 6, QTableWidgetItem(f"{v['odom_x']:.2f}"))
            self.vehicle_table.setItem(row, 7, QTableWidgetItem(f"{v['odom_y']:.2f}"))

        self.arena_view.update_vehicles(vehicles)
        self.drive_arena.update_vehicles(vehicles)

    def log(self, msg: str):
        ts = time.strftime('%H:%M:%S')
        self.log_output.append(f'[{ts}] {msg}')

    def on_start_sim(self):
        self.sim.start()
        self.lbl_sim_status.setText('模拟状态: 运行中')
        self.lbl_sim_status.setStyleSheet('color: green;')
        self.log('模拟已启动')

    def on_stop_sim(self):
        self.sim.stop()
        self.lbl_sim_status.setText('模拟状态: 已停止')
        self.lbl_sim_status.setStyleSheet('color: gray;')
        self.log('模拟已停止')

    def on_send_cmd_vel(self):
        vehicle = self.cmb_vehicle.currentText()
        linear = self.spin_linear.value()
        angular = self.spin_angular.value()
        self.log(f'速度指令 -> {vehicle}: linear={linear}, angular={angular}')

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
        self.log(f'故障指令 -> {vehicle}: {fault_type}')
        self.fault_log.append(f"[{time.strftime('%H:%M:%S')}] {vehicle}: {fault_type}")

    def on_reset_faults(self):
        vehicle = self.cmb_fault_vehicle.currentText()
        self.log(f'重置故障 -> {vehicle}')
        for chk in self.fault_checks.values():
            chk.setChecked(False)

    def on_load_trajectory(self):
        self.lbl_traj_file.setText('demo_trajectory.json')
        self.log('加载轨迹文件: demo_trajectory.json')

    def on_play_trajectory(self):
        self.log('开始播放轨迹')

    def on_pause_trajectory(self):
        self.log('暂停播放')

    def on_stop_trajectory(self):
        self.log('停止播放')

    def on_add_vehicle(self):
        row = self.config_table.rowCount()
        self.config_table.setRowCount(row + 1)
        self.config_table.setItem(row, 0, QTableWidgetItem(f'smart_car_{row+1}'))
        self.config_table.setItem(row, 1, QTableWidgetItem('medium_car'))
        self.config_table.setItem(row, 2, QTableWidgetItem('0'))
        self.config_table.setItem(row, 3, QTableWidgetItem('-18'))
        self.config_table.setItem(row, 4, QTableWidgetItem('#ffffff'))
        self.config_table.setItem(row, 5, QTableWidgetItem('是'))
        self.log(f'添加车辆: smart_car_{row+1}')

    def on_save_config(self):
        self.log('配置已保存（演示模式）')


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

    window = DemoMainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
