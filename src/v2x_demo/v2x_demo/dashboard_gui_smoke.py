#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v2x_dashboard GUI 冒烟测试（offscreen）：构造主窗口、注入合成车辆/灯态/回放数据，
触发一次 paintEvent 并渲染到 PNG，验证 SceneView/ControlPanel/LogView 无异常且能出图。
不需真实 Gazebo 或显示器。用法：QT_QPA_PLATFORM=offscreen python3 dashboard_gui_smoke.py [out.png]
"""
import os
import sys
import math

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import rclpy  # noqa: E402
from PyQt5 import QtWidgets, QtGui, QtCore  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from v2x_dashboard import DashboardNode, DashboardWindow  # noqa: E402


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_dashboard_smoke.png'
    live = bool(os.environ.get('V2X_DASH_LIVE'))
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
    dash = DashboardNode()

    app = QtWidgets.QApplication([])

    if not live:
        # 注入合成运行时数据（无需仿真）
        dash.vehicles = {
            'v2x_car_n1': dict(x=1.5, y=-12.0, z=0.2, yaw=math.pi / 2),
            'v2x_car_s1': dict(x=-1.5, y=8.0, z=0.2, yaw=-math.pi / 2),
            'v2x_car_e1': dict(x=-9.0, y=-1.6, z=0.2, yaw=0.0),
            'v2x_prius': dict(x=5.0, y=15.0, z=0.2, yaw=math.pi),
        }
        dash.playback = {'v2x_car_w1': dict(x=12.0, y=1.6, z=0.5, yaw=math.pi)}
        for i, g in enumerate(dash.groups):
            dash.light_state[g] = ['GREEN', 'YELLOW', 'RED'][i % 3]
        # 合成一帧“实景”图像，验证实景3D 视口能绘制视频帧（无需真实 Gazebo）
        fake = QtGui.QImage(320, 200, QtGui.QImage.Format_RGB888)
        fake.fill(QtGui.QColor(20, 30, 50))
        _p = QtGui.QPainter(fake)
        _grad = QtGui.QLinearGradient(0, 0, 320, 200)
        _grad.setColorAt(0.0, QtGui.QColor(40, 90, 150))
        _grad.setColorAt(1.0, QtGui.QColor(180, 200, 120))
        _p.fillRect(fake.rect(), _grad)
        _p.setPen(QtGui.QColor(255, 255, 255))
        _p.drawText(fake.rect(), QtCore.Qt.AlignCenter, 'FAKE GAZEBO FRAME')
        _p.end()
        dash.cam_image = fake
        dash.cam_frames = 1

    win = DashboardWindow(dash)
    win.resize(1200, 780)
    win.show()
    app.processEvents()

    if live:
        # 真实仿真模式：泵送若干秒，让 /model_states 与灯态话题填充进来
        import time
        t_end = time.time() + float(os.environ.get('V2X_DASH_LIVE_SECS', '8'))
        while time.time() < t_end:
            win.scene_cam.drive_camera()   # 持续把实景相机摆到轨道位姿（内部节流）
            win._tick()
            app.processEvents()
            time.sleep(0.03)
    else:
        # 触发一次 tick（spin_once + scene.update），再抓图
        win._tick()
        app.processEvents()

    pix = win.grab()
    ok = pix.save(out, 'PNG')

    # 单独抓 SceneView / Scene3DView 便于检查地图渲染
    scene_pix = win.scene.grab()
    scene_out = out.replace('.png', '_scene.png')
    scene_pix.save(scene_out, 'PNG')
    scene3d_pix = win.scene3d.grab()
    scene3d_out = out.replace('.png', '_scene3d.png')
    scene3d_pix.save(scene3d_out, 'PNG')
    # 实景3D：切到相机视口再抓图
    win._switch_view(2, win.btn_cam)
    app.processEvents()
    cam_pix = win.scene_cam.grab()
    cam_out = out.replace('.png', '_scene_cam.png')
    cam_pix.save(cam_out, 'PNG')

    print(f'[smoke] window saved={ok} -> {out} ({pix.width()}x{pix.height()})')
    print(f'[smoke] scene  saved -> {scene_out} ({scene_pix.width()}x{scene_pix.height()})')
    print(f'[smoke] scene3d saved -> {scene3d_out} ({scene3d_pix.width()}x{scene3d_pix.height()})')
    print(f'[smoke] scene_cam saved -> {cam_out} ({cam_pix.width()}x{cam_pix.height()})')
    print(f'[smoke] status bar: {win.status.currentMessage()}')
    print('[smoke] log lines:')
    print(win.log.toPlainText())

    win.close()
    dash.destroy_node()
    rclpy.shutdown()
    print('GUI_SMOKE_OK' if ok else 'GUI_SMOKE_SAVE_FAILED')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
