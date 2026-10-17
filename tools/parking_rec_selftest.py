#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vid_rec.py 的 --mode parking 面板自测（不启 Gazebo，不发真实相机）。

为什么要单独测一个"只是画画字"的面板：取证视频里的面板没有中间态可读——行溢出下沿、
一行超过面板宽度被右边裁掉、故障分支译名漏一支显示成 "4"，这三种坏法在片子里和
"这段时间没有车"长得一模一样，事后根本查不出来。所以这里把版面本身当数值判据：
逐行取渲染位图的真实像素宽高来比边界，而不是靠人眼回看录像。

两段：
  A 纯逻辑（不碰 DDS）：把 vid_rec 当模块 import（先把它自己的必填 argv 塞好），
    喂最坏情况的合成消息，核对译名覆盖、-1 段不产生负耗时、按显示宽度折行、
    缓冲区截断留最新、以及整屏既不越下沿也不越右边界。
  B 端到端（真录一段）：fork vid_rec 录 10 s，本进程等订阅匹配上后按 5 Hz 发同一批
    合成消息，录完解码回读，用像素统计证明"面板确实画上去了"（只读进程自己打印的
    "已写入 N 帧"只能证明它写了帧，证明不了帧里有内容）。

用法（WSL，要 ~/v2x_ws 里的 v2x_msgs 已编好）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/parking_rec_selftest.py
  --skip-e2e 只跑 A 段（环境没起 DDS 时用）。
退出码 0=PASS 1=FAIL 2=环境不齐。
"""
import os
import re
import subprocess
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.join(HERE, 'vid_rec.py')
OUT = '/tmp/pk_rec_selftest.mp4'
CAP = '/tmp/pk_rec_selftest_caption.txt'
PNG_DIR = '/tmp/v2x_pk_selftest_in'
PNG = os.path.join(PNG_DIR, 'fixture.png')
FAILED = []

# A 段调用 panel_parking 时按默认版面（vid_rec 里 W/H/PANEL/MAIN/HEAD）比对角；
# 先在 import 前给 vid_rec 备好它 argparse 的必填项，否则模块一 import 就 SystemExit。
# 本脚本自己的开关要先抄下来：下面这行会把 sys.argv 整个换成 vid_rec 认的那套。
ARGS = list(sys.argv[1:])
sys.argv = ['vid_rec.py', '--out', OUT, '--dur', '0', '--mode', 'parking']
sys.path.insert(0, HERE)
try:
    import vid_rec as vr                                    # noqa: E402
    from std_msgs.msg import Header                          # noqa: E402
    from v2x_msgs.msg import BarrierState, ParkingAction, ParkingPass  # noqa: E402
except ImportError as e:
    print('环境不齐（缺 rclpy / cv2 / v2x_msgs）：%s' % e)
    raise SystemExit(2)


def check(cond, what, detail=''):
    if cond:
        print('  ok   %s' % what)
    else:
        FAILED.append(what)
        print('  FAIL %s  %s' % (what, detail))


def banner(t):
    print('\n── %s' % t)


def consts(cls, prefix):
    """把 msg 类里的 uint8 常量挑出来（生成代码里就是普通类属性）。"""
    return {k: v for k, v in vars(cls).items()
            if k.startswith(prefix) and isinstance(v, int) and not isinstance(v, bool)}


def st(m, t):
    m.header = Header()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    return m


def mk_gate(gid, state, src, fault, target, angle, eff, occ, veh, rc, lc, el, at):
    m = BarrierState()
    st(m, 1000.0)
    m.gate_id = gid
    m.state = state
    m.source = src
    m.fault = fault
    m.target_rad = target
    m.angle_rad = angle
    m.angle_error_rad = target - angle
    m.at_target = at
    m.angvel_rps = 0.02
    m.effort_nm = eff
    m.action_elapsed_s = el
    m.loop_id = 'PKG-%s-loop' % ('in' if gid == 'gate_in' else 'out')
    m.loop_occupied = occ
    m.vehicle_id = veh
    m.raise_count = rc
    m.lower_count = lc
    return m


def mk_pass(seq, gate, direction, plate, kind, wl, result, reason, ticket,
            arrive, raise_s, clear, lower, img=''):
    m = ParkingPass()
    st(m, arrive)
    m.pass_id = 'P%04d-%s' % (seq, gate)
    m.direction = direction
    m.gate_id = gate
    m.plate_number = plate
    m.recognized = bool(plate)
    m.plate_confidence = 0.93 if plate else 0.0
    m.capture_id = '%s-%04d' % (gate, seq)
    m.image_path = img
    m.vehicle_id = 'car_6'
    m.kind = kind
    m.result = result
    m.reason = reason
    m.ticket_id = ticket
    m.whitelist_hit = wl
    m.arrive_s = arrive
    m.raise_s = raise_s
    m.clear_s = clear
    m.lower_s = lower
    return m


def mk_act(seq, gate, action, src, fault, reached, el, note, veh='car_6', plate='沪A12345'):
    m = ParkingAction()
    st(m, 1200.0 + seq)
    m.action_id = 'A%04d' % seq
    m.action = action
    m.gate_id = gate
    m.vehicle_id = veh
    m.plate_number = plate
    m.source = src
    m.fault = fault
    m.from_rad = 0.0
    m.to_rad = 1.5708 if action in (ParkingAction.ACT_RAISE, ParkingAction.ACT_RAISE_CMD) else 0.0
    m.elapsed_s = el
    m.reached = reached
    m.note = note
    return m


def worst_case_rec():
    """最坏情况：两块闸机（其中一块挂故障）+ 六条通行记录（两条被拒且原因很长）
    + 六条动作日志（三条带注记）+ 一长串摆渡车进度。面板能撑下这一屏就撑得下常态。"""
    r = SimpleNamespace(gates={}, passes=[], acts=[], shut='', n_pass=0, n_act=0)
    r.gates['gate_in'] = mk_gate('gate_in', BarrierState.STATE_OPEN, BarrierState.SRC_AUTO,
                                 BarrierState.FAULT_NONE, 1.5708, 1.5761, 42.0,
                                 True, 'car_6', 6, 5, 0.0, True)
    r.gates['gate_out'] = mk_gate('gate_out', BarrierState.STATE_FAULT, BarrierState.SRC_FAULT,
                                  BarrierState.FAULT_RAISE_FAIL, 1.5708, 0.1892, 300.0,
                                  False, '', 4, 4, 6.0, False)
    r.passes = [
        mk_pass(1, 'gate_in', ParkingPass.DIR_IN, '沪A12345', ParkingPass.KIND_MONTHLY, 1,
                ParkingPass.RESULT_ALLOWED, '', '', 100.0, 102.4, 103.6, 105.1,
                '/tmp/v2x_parking_in/gate_in-0001.png'),
        mk_pass(2, 'gate_in', ParkingPass.DIR_IN, '粤Z1A2B3', ParkingPass.KIND_TEMP, 2,
                ParkingPass.RESULT_ALLOWED, '临时车已发临时卡', 'T-1234', 200.0, 202.8,
                204.0, -1.0),
        mk_pass(3, 'gate_out', ParkingPass.DIR_OUT, '', ParkingPass.KIND_UNKNOWN, 2,
                ParkingPass.RESULT_DENIED,
                '未登记车牌且 unknown_as_temp 关闭，未抬杆，请走人工登记通道', '',
                300.0, -1.0, -1.0, -1.0),
        mk_pass(4, 'gate_out', ParkingPass.DIR_OUT, '浙A66666', ParkingPass.KIND_BLACKLIST, 3,
                ParkingPass.RESULT_DENIED, '命中黑名单，闸机保持落下并上报告警',
                '', 400.0, -1.0, -1.0, -1.0),
        mk_pass(5, 'gate_in', ParkingPass.DIR_IN, '苏E66F88', ParkingPass.KIND_MONTHLY, 1,
                ParkingPass.RESULT_PENDING, '等临时车登记确认', '', 500.0, -1.0, -1.0, -1.0),
        mk_pass(6, 'gate_in', ParkingPass.DIR_IN, '沪A12345', ParkingPass.KIND_MONTHLY, 1,
                ParkingPass.RESULT_ALLOWED, '', '', 600.0, 602.2, 603.3, 604.9),
    ]
    r.n_pass = 20
    r.acts = [
        mk_act(1, 'gate_in', ParkingAction.ACT_RAISE_CMD, BarrierState.SRC_AUTO,
               BarrierState.FAULT_NONE, False, 0.0, ''),
        mk_act(2, 'gate_in', ParkingAction.ACT_RAISE, BarrierState.SRC_AUTO,
               BarrierState.FAULT_NONE, True, 1.52, '核验通过：白名单月租车'),
        mk_act(3, 'gate_in', ParkingAction.ACT_HOLD_OPEN, BarrierState.SRC_AUTO,
               BarrierState.FAULT_NONE, True, 0.41, '杆下仍被占用，防砸重抬'),
        mk_act(4, 'gate_in', ParkingAction.ACT_LOWER, BarrierState.SRC_AUTO,
               BarrierState.FAULT_NONE, True, 1.48, ''),
        mk_act(5, 'gate_out', ParkingAction.ACT_FAULT, BarrierState.SRC_FAULT,
               BarrierState.FAULT_RAISE_FAIL, False, 6.0,
               '抬杆失败：raise_timeout_s 内目标角未到达（实测 10.8 度 / 指令 90.0 度）'),
        mk_act(6, 'gate_out', ParkingAction.ACT_MANUAL, BarrierState.SRC_MANUAL,
               BarrierState.FAULT_NONE, True, 1.71, '手动指令来自 /parking/gate_manual_cmd'),
    ]
    r.n_act = 43
    r.shut = ('phase=hold 3/8 PKG-in-verify#0 目标=(12.00,-11.21) 车头=+0.00 '
              '位置=(12.00,-11.24) 距离=0.03 航向=+0.01 指令 v=0.00 w=+0.00｜第 2 轮'
              '（作废 1 轮）｜停在核验位等抬杆')
    return r


def probe_panel(r):
    """临时替换 R.text，把每一行的 (x, y, 文本, 像素宽, 字号, 颜色) 记下来再照常绘制。"""
    lines = []
    orig = vr.R.text

    def probe(im, x, y, s, sz, col):
        w = vr.R.img(s, sz, col).size[0] if s else 0
        lines.append(SimpleNamespace(x=x, y=y, s=s, w=w, sz=sz, col=col))
        return orig(im, x, y, s, sz, col)

    vr.R.text = probe
    canvas = np.zeros((vr.H, vr.W, 3), np.uint8)
    from PIL import Image
    im = Image.fromarray(canvas)
    try:
        y_end = vr.panel(r, im, vr.HEAD + 10)
    finally:
        vr.R.text = orig
    return lines, y_end


def stage_a():
    banner('A 面板版面：不越下沿、不越右边界')
    check(vr.A.mode == 'parking', '--mode parking 能路由到停车场面板',
          'mode=%s' % vr.A.mode)
    r = worst_case_rec()
    lines, y_end = probe_panel(r)
    panel_lines = [l for l in lines if l.x >= vr.MAIN]
    check(len(panel_lines) > 25, '最坏情况一屏画出了 %d 行面板字' % len(panel_lines))
    over = [l for l in panel_lines if l.y + l.sz + 5 > vr.H]
    check(not over, '整屏不越下沿（末行 y=%.0f，画面高 %d）' % (
        max((l.y for l in panel_lines), default=0), vr.H),
          '越界行：%s' % [l.s[:20] for l in over])
    wide = [l for l in panel_lines if l.x + l.w > vr.W - 2]
    check(not wide, '每行都不越右边界（面板宽 %d）' % vr.PANEL,
          '超宽行：%s' % [(l.s[:24], l.x + l.w) for l in wide])
    check(y_end <= vr.H, 'panel() 返回的推进 y 仍在画面内（%.0f）' % y_end)

    banner('B 译名表覆盖 msg 里的全部常量（漏一支就显示成 "?"，读片时看不出是漏译）')
    for tab, cls, pre, allow_empty in (
            ('BARRIER_STATE', BarrierState, 'STATE_', False),
            ('BARRIER_SRC', BarrierState, 'SRC_', False),
            ('BARRIER_FAULT', BarrierState, 'FAULT_', True),
            ('PASS_KIND', ParkingPass, 'KIND_', False),
            ('PASS_RESULT', ParkingPass, 'RESULT_', False),
            ('PASS_DIR', ParkingPass, 'DIR_', False),
            ('ACT_NAME', ParkingAction, 'ACT_', False)):
        tab_v = getattr(vr, tab)
        missing = [k for k, v in consts(cls, pre).items() if v not in tab_v]
        blank = [v for k, v in consts(cls, pre).items()
                 if not allow_empty and not str(tab_v.get(v, ''))]
        check(not missing and not blank, '%s 覆盖 %s 的 %s 常量' % (tab, cls.__name__, pre),
              '缺=%s 空=%s' % (missing, blank))
    wl_missing = [i for i in range(4) if i not in vr.WL_NAME]
    check(not wl_missing, 'WL_NAME 覆盖 whitelist_hit 的 0..3')

    banner('C 未到达的时段是 -1，既不算差值也不画成负数')
    check(vr.pk_dur(-1.0, 5.0) == -1.0 and vr.pk_dur(5.0, -1.0) == -1.0,
          '任一侧 -1 → 差值也是 -1')
    check(abs(vr.pk_dur(100.0, 102.4) - 2.4) < 1e-9, '两侧都有值时算出差值')
    check(vr.pk_sec(-1.0) == '—' and vr.pk_sec(2.4) == '2.40', '未到达显示破折号')
    rows = [l.s for l in lines if l.s.startswith('P00')]
    check(len(rows) == 6, '面板上给了 6 条通行记录行')
    neg = [x for x in rows if re.search(r'-\d+\.\d+', x)]
    check(not neg, '通行记录行里没有负耗时', '含负数的行：%s' % neg)
    check(any('—' in x for x in rows), '被拒那几条确实带着破折号（不是 0.00 那种假到位）')
    grows = [l.s for l in lines if l.s.strip().startswith(('in ', 'out '))]
    check(len(grows) == 2, '两块闸机各一行：%s' % grows)
    check(any(re.search(r'\s-\d+\.\d+', x) for x in grows),
          '闸机行允许出现负的角误差（实测冲过指令角是真物理量，不能被当成哨兵抹掉）',
          '闸机行：%s' % grows)

    banner('D 折行按显示宽度（CJK 算两个半角位），缓冲区只留最新')
    segs = vr.wrap('中文字符串测试' * 10, 48)
    wd = [sum(1 if ord(c) < 0x2E80 else 2 for c in s) for s in segs]
    check(all(w <= 48 for w in wd), '每段显示宽度 ≤ 48（实测 %s）' % wd)
    check(''.join(segs) == '中文字符串测试' * 10, '折行没有丢字也没有拆字')
    check(len(vr.wrap('abc')) == 1, '短串不折行')
    rr = SimpleNamespace(passes=[], acts=[], n_pass=0, n_act=0)
    for i in range(20):
        vr.Rec.on_pass(rr, mk_pass(i + 1, 'gate_in', ParkingPass.DIR_IN, '沪A12345',
                                   ParkingPass.KIND_MONTHLY, 1, ParkingPass.RESULT_ALLOWED,
                                   '', '', 700.0 + i, -1.0, -1.0, -1.0))
    for i in range(15):
        vr.Rec.on_act(rr, mk_act(i + 1, 'gate_in', ParkingAction.ACT_RAISE,
                                 BarrierState.SRC_AUTO, BarrierState.FAULT_NONE, True,
                                 1.5, ''))
    check(len(rr.passes) == 6 and rr.passes[-1].pass_id.startswith('P0020'),
          '通行记录截到 6 条且留下的是最新的（实测末条 %s）' % rr.passes[-1].pass_id)
    check(len(rr.acts) == 6 and rr.acts[-1].action_id == 'A0015',
          '动作日志截到 6 条且留下的是最新的')
    check(rr.n_pass == 20 and rr.n_act == 15, '计数器数的是收过的总条数，不是缓冲长度')

    banner('E 判色：指令类"未到位"不红，故障才红')
    cmd = [l for l in lines if '下达抬杆' in l.s]
    check(bool(cmd), '动作日志里有"下达抬杆"这一行')
    if cmd:
        check(cmd[0].col == (235, 235, 235), '下达抬杆（未到点、无故障）不标红',
              '实际颜色 %s' % (cmd[0].col,))
    f = [l for l in lines if '抬杆失败' in l.s]
    check(bool(f), '故障注记进了面板（抬杆失败）')


def publish_loop(proc):
    """按 5 Hz 把合成消息发出去，一直发到 vid_rec 自己录完（或硬超时兜底）。

    用"子进程还在不在"当循环条件，不拿墙钟猜录制时长：vid_rec 的 import + DDS 匹配要先
    吃掉几秒，猜早了面板还没收到几帧就结束了，猜晚了这边空转、communicate 也拿不到输出。
    """
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    n = Node('pk_rec_selftest_pub')
    qp = 10
    p_state = n.create_publisher(BarrierState, '/parking/gate_state', qp)
    p_pass = n.create_publisher(ParkingPass, '/parking/pass', qp)
    p_act = n.create_publisher(ParkingAction, '/parking/actions', qp)
    p_shut = n.create_publisher(String, '/parking/shuttle_status', qp)
    t0 = time.time()
    while time.time() - t0 < 25 and p_state.get_subscription_count() == 0 \
            and proc.poll() is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    matched = p_state.get_subscription_count()
    print('  订阅匹配：/parking/gate_state 的订阅者 %d 个（等了 %.1f s）'
          % (matched, time.time() - t0))
    if not matched:
        rclpy.shutdown()
        return 0
    r = worst_case_rec()
    order = list(r.gates.values())
    k = 0
    sent = 0
    while proc.poll() is None and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.0)
        g = order[k % 2]
        g.angle_rad = g.target_rad - 0.01 * (k % 3)
        p_state.publish(g)
        p_shut.publish(String(data=r.shut))
        if k % 5 == 0:
            p_pass.publish(r.passes[(k // 5) % 6])
            p_act.publish(r.acts[k % 6])
        k += 1
        sent += 1
        time.sleep(0.2)
    rclpy.shutdown()
    return sent


def stage_b():
    banner('F 端到端真录一段：帧数 + 像素里有字')
    os.makedirs(PNG_DIR, exist_ok=True)
    # 相机画面用一张纯灰底图顶上的（file: 源取 mtime 最新的一张）：这一屏要证明的是
    # "左画面区非黑、右面板区有亮字"，不是"停车场好看"。
    cv2.imwrite(PNG, np.full((300, 400, 3), 128, np.uint8))
    with open(CAP, 'w', encoding='utf-8') as fh:
        fh.write('自测：car_6 进入核验位，入口闸机抬杆\n')
        fh.write('自测：地感占用 → 落杆，通行记录已入库\n')
    if os.path.exists(OUT):
        os.remove(OUT)
    proc = subprocess.Popen(
        [sys.executable, REC, '--out', OUT, '--dur', '10', '--fps', '5', '--mode', 'parking',
         '--views', 'file:%s/*.png' % PNG_DIR, '--caption', CAP,
         '--title', '停车场面板自测（合成数据，非取证素材）'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    sent = publish_loop(proc)
    out, _ = proc.communicate(timeout=120)
    check(proc.returncode == 0, 'vid_rec 退出码 0', out[-300:])
    frames = re.search(r'（(\d+) 帧', out)
    check(frames is not None and int(frames.group(1)) >= 45,
          'stdout 里有"已写入 N 帧"这行且帧数够（目标 50）',
          '这行靠 main() 结尾的 flush 才留得住：stdout 指到文件时是块缓冲，os._exit 不刷新。'
          '捕获到的输出末尾=%r' % out[-200:])
    cap = cv2.VideoCapture(OUT)
    got, last = 0, None
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        got += 1
        last = fr
    cap.release()
    check(got >= 45, '解码回读得到 %d 帧' % got)
    if last is None:
        check(False, '末帧读不出来', OUT)
        return
    reg = last[vr.HEAD + 8:vr.H - 6, vr.MAIN + 10:vr.W]
    diff = np.any(np.abs(reg.astype(int) - np.array([24, 26, 30])) > 12, axis=2)
    ink = np.min(reg, axis=2) > 110
    check(diff.mean() > 0.01, '面板区改过背景的像素占比 %.1f%%（≥1%% 才算画上去了）'
          % (diff.mean() * 100.0))
    check(ink.sum() > 500, '面板区亮字像素 %d 个（≥500）' % ink.sum())
    left = last[vr.HEAD:vr.H - 130, 0:vr.MAIN]
    check(left.mean() > 60, '左侧画面区不是黑屏（均值 %.1f，证明 file: 源接上了）'
          % left.mean())
    check('!!' not in out, '没有"没等到相机首帧"的警告', out[-200:])
    print('  合成消息发送 %d 拍' % sent)


def main():
    print('vid_rec 停车场面板自测（版面 %d×%d，面板宽 %d）' % (vr.W, vr.H, vr.PANEL))
    stage_a()
    if '--skip-e2e' in ARGS:
        print('\n（按 --skip-e2e 跳过了端到端录制）')
    else:
        stage_b()
    print('\n%s  (%d 项失败)' % ('PASS' if not FAILED else 'FAIL', len(FAILED)))
    return 0 if not FAILED else 1


if __name__ == '__main__':
    raise SystemExit(main())
