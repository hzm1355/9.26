#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/pk_record_audit.py 的合成数据自校：一个正样本 + 三个负样本。

为什么对账器自己也要自测：
  pk_record_audit.py 是"判定别人对不对"的脚本，它自己判据写反的话，产出的是**假通过**
  ——比不过更糟。本轮就真抓到一个：D 段排序方向写反（newest_first=True 时记录器给的是
  降序，判据却要求升序），在真实数据上必然 FAIL，看上去像"记录器排序坏了"。所以每一段
  新加的判据都得有一个"能把它弄挂"的样本，否则它可能恒真。

四个样本：
  正  真 ParkingRecorderNode 在私有 ROS_DOMAIN_ID 上跑起来，另一个节点走**真话题**喂
      5 条通行 + 8 条动作，再用**子进程**跑对账器（它必须自己发现服务、自己发请求），
      期望 rc==0。走话题而不直接调回调，是因为对账器查的是"服务回包 vs 磁盘账本"，
      回调直调会跳过 DDS 与落盘这两段真实路径。
  负1 往 JSONL 追加一条伪记录（内存里没这条）：CSV 是内存投影，必然少一行 ——
      期望 rc!=0 且 FAIL 点名"CSV 数据行"。这条专治"账本被动过但导出看起来正常"。
  负2 空账本：期望 rc!=0 且 FAIL 说"空账本"，防止拿空目录跑出一个绿。
  负3 把账本 mtime 推到 1 小时前（内容照旧）：期望 rc!=0 且 FAIL 说"遗留账本"，
      防止对账器用上一轮的旧账本把 C/D/E/F 全跑绿。
  负4 塞一条没有凭据的"收尾补记"落杆 + 一条重复 pass_id（离线）：期望两条 FAIL 分别点名
      "收尾补记"与"pass_id+gate 唯一"，而导出判据不许被牵连 —— 专治到位行三条来源分类写错。

用法（WSL）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 tools/pk_record_audit_selftest.py
退出码 0=PASS 1=FAIL。全程不碰 /tmp/v2x_parking，产物都在 mkdtemp 里，结束即删。
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUDIT = os.path.join(ROOT, 'tools', 'pk_record_audit.py')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'parking_zones.yaml')
sys.path.insert(0, os.path.join(ROOT, 'src', 'v2x_demo', 'v2x_demo'))

# 私有域要在 import rclpy 之前定：DDS 在 init 时读环境变量，之后再改没用。
# 挑 100..189 这一段，避开取证栈常用的默认域，免得和真在跑的 parking 节点抢服务名。
DOMAIN = str(100 + (os.getpid() % 90))
os.environ['ROS_DOMAIN_ID'] = DOMAIN

FAILED = []


def check(cond, what, detail=''):
    if cond:
        print('  ok   %s' % what)
    else:
        FAILED.append(what)
        print('  FAIL %s  %s' % (what, detail))


def banner(t):
    print('\n── %s' % t)


import rclpy                                   # noqa: E402
from rclpy.executors import SingleThreadedExecutor   # noqa: E402
from rclpy.node import Node                    # noqa: E402
from rclpy.parameter import Parameter          # noqa: E402
from builtin_interfaces.msg import Time        # noqa: E402
from geometry_msgs.msg import Pose, Quaternion  # noqa: E402
from v2x_msgs.msg import BarrierState, ParkingAction, ParkingPass  # noqa: E402

import parking_recorder_node as prn            # noqa: E402

BASE = 1791784800.0    # 与实车仿真钟同量级；刻意取整秒，避免浮点尾差干扰逐列比对


def t_of(sec):
    t = Time()
    t.sec = int(sec)
    t.nanosec = int(round((sec - int(sec)) * 1e9))
    return t


def pose_of(x, y, z=6.70, yaw=1.5708):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = Quaternion(w=math.cos(yaw / 2.0), z=math.sin(yaw / 2.0))
    return p


def pass_msg(sim_ts, pid, plate, direction, gate, kind, result, reason,
             arrive, raise_, clear, lower, vid='car_6', conf=0.942, ticket='',
             hit=1, recognized=True):
    m = ParkingPass()
    m.header.stamp = t_of(sim_ts)
    m.header.frame_id = 'world'
    m.pass_id = pid
    m.direction = direction
    m.gate_id = gate
    m.plate_number = plate
    m.recognized = recognized
    m.plate_confidence = conf
    m.capture_id = 'C-' + pid
    m.image_path = '/tmp/selftest_cap/%s.jpg' % pid
    m.vehicle_id = vid
    m.pose = pose_of(12.0, -11.4)
    m.kind = kind
    m.result = result
    m.reason = reason
    m.ticket_id = ticket
    m.whitelist_hit = hit
    m.arrive_s = arrive
    m.raise_s = raise_
    m.clear_s = clear
    m.lower_s = lower
    return m


def act_msg(sim_ts, aid, act, gate, reached=True, to_rad=1.5708, elapsed=0.85,
            source=BarrierState.SRC_AUTO, fault=BarrierState.FAULT_NONE,
            plate='沪A12345', note=''):
    a = ParkingAction()
    a.header.stamp = t_of(sim_ts)
    a.header.frame_id = 'world'
    a.action_id = aid
    a.action = act
    a.gate_id = gate
    a.vehicle_id = 'car_6'
    a.plate_number = plate
    a.source = source
    a.fault = fault
    a.from_rad = 0.0 if to_rad > 0 else 1.5708
    a.to_rad = to_rad
    a.elapsed_s = elapsed
    a.reached = reached
    a.note = note
    return a


def scenarios():
    """5 条通行要把过滤矩阵的每个轴都铺到：两方向、两闸机、三种结果、四种车型、
    空车牌、含 -1 的时刻（拒绝/挂起那两条不该算出正耗时）。"""
    P = ParkingPass
    return [
        pass_msg(BASE + 0.0, 'P0001', '沪A12345', P.DIR_IN, 'gate_in',
                 P.KIND_MONTHLY, P.RESULT_ALLOWED, '核验通过，抬杆到位',
                 BASE - 1.0, BASE - 0.15, BASE + 1.0, BASE + 1.6, hit=1),
        pass_msg(BASE + 3.0, 'P0002', '京B88990', P.DIR_IN, 'gate_in',
                 P.KIND_TEMP, P.RESULT_ALLOWED, '临时卡登记后放行',
                 BASE + 2.0, BASE + 2.85, BASE + 4.0, BASE + 4.6,
                 ticket='T-0002', hit=2),
        pass_msg(BASE + 6.0, 'P0003', '沪A12345', P.DIR_OUT, 'gate_out',
                 P.KIND_TEMP, P.RESULT_ALLOWED, '缴费出场',
                 BASE + 5.0, BASE + 5.85, BASE + 7.0, BASE + 7.6,
                 vid='car_6', ticket='T-0002', hit=0),
        pass_msg(BASE + 9.0, 'P0004', '粤C66666', P.DIR_IN, 'gate_in',
                 P.KIND_BLACKLIST, P.RESULT_DENIED, '黑名单车辆，拒绝入内并告警',
                 BASE + 8.5, -1.0, -1.0, -1.0, hit=3),
        pass_msg(BASE + 12.0, 'P0005', '', P.DIR_IN, 'gate_in',
                 P.KIND_UNKNOWN, P.RESULT_PENDING, '车牌未定位，挂起等待人工',
                 BASE + 11.5, -1.0, -1.0, -1.0, conf=0.0,
                 hit=0, recognized=False),
    ]


def actions():
    A = ParkingAction
    out = []
    for g, s in (('gate_in', BASE), ('gate_out', BASE + 5.0)):
        out.append(act_msg(s + 0.0, 'A-%s-1' % g, A.ACT_RAISE_CMD, g,
                           to_rad=1.5708, elapsed=0.0))
        out.append(act_msg(s + 0.85, 'A-%s-2' % g, A.ACT_RAISE, g,
                           to_rad=1.5708, elapsed=0.85))
        out.append(act_msg(s + 3.1, 'A-%s-3' % g, A.ACT_LOWER_CMD, g,
                           to_rad=0.0, elapsed=0.0))
        out.append(act_msg(s + 3.7, 'A-%s-4' % g, A.ACT_LOWER, g,
                           to_rad=0.0, elapsed=0.6))
    return out


class Feed(Node):
    """喂数据的节点。独立于记录器，走真话题，才对账器要核对的那条链路。"""

    def __init__(self):
        super().__init__('pk_audit_selftest_feed')
        self.pp = self.create_publisher(ParkingPass, '/parking/pass', 50)
        self.ap = self.create_publisher(ParkingAction, '/parking/actions', 50)

    def wait_discovered(self, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.pp.get_subscription_count() > 0 and self.ap.get_subscription_count() > 0:
                return True
            time.sleep(0.2)
        return False


def jsonl_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8') as f:
        return [json.loads(x) for x in f if x.strip()]


def run_audit(d, prefix, skip_live=False, timeout=240):
    cmd = [sys.executable, AUDIT, '--config', CFG, '--dir', d, '--prefix', prefix]
    if skip_live:
        cmd.append('--skip-live')
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return -1, '（子进程超时 %d s）' % timeout
    out = p.stdout
    if p.stderr.strip():
        out += '\n[stderr]\n' + p.stderr
    return p.returncode, out


def fails_of(out):
    return [ln[len('  FAIL: '):] for ln in out.splitlines()
            if ln.startswith('  FAIL: ')]


def tail(out, n=6):
    lines = [ln for ln in out.splitlines() if ln.strip()]
    return '\n'.join('      ' + ln for ln in lines[-n:])


def main():
    if not os.path.exists(CFG) or not os.path.exists(AUDIT):
        print('找不到 %s 或 %s' % (CFG, AUDIT))
        return 2
    print('私有域 ROS_DOMAIN_ID=%s｜配置 %s' % (DOMAIN, CFG))
    root = tempfile.mkdtemp(prefix='pk_audit_self_')
    rclpy.init()
    rc = 1
    try:
        rc = run_all(root)
    finally:
        rclpy.shutdown()
        shutil.rmtree(root, ignore_errors=True)
    print('\n%s  (%d 项失败)' % ('PASS' if not FAILED else 'FAIL', len(FAILED)))
    return 0 if not FAILED else 1


def run_all(root):
    d = os.path.join(root, 'live')
    os.makedirs(d, exist_ok=True)

    banner('负 2 空账本：必须 FAIL 并点名"空账本"')
    empty = os.path.join(root, 'empty')
    os.makedirs(empty, exist_ok=True)
    code, out = run_audit(empty, 'e', skip_live=True, timeout=60)
    fl = fails_of(out)
    check(code != 0, '空账本 rc!=0（实际 rc=%s）' % code, tail(out))
    check(any('空账本' in f for f in fl), 'FAIL 点名"空账本"', ' | '.join(fl))

    banner('起记录器节点（私有域）+ 话题喂 5 条通行 / 8 条动作')
    node = prn.ParkingRecorderNode(parameter_overrides=[
        Parameter('config_file', Parameter.Type.STRING, CFG),
        Parameter('record_dir', Parameter.Type.STRING, d)])
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    feed = Feed()
    try:
        found = feed.wait_discovered()
        check(found, '喂数据节点发现了记录器的两个订阅（DDS 真通了）')
        if not found:
            return 1
        for m in scenarios():
            feed.pp.publish(m)
            time.sleep(0.08)
        for a in actions():
            feed.ap.publish(a)
            time.sleep(0.08)
        # 生效要用效果复核：看磁盘行数到没到位，而不是"发出去了"。
        p_path = os.path.join(d, 'parking_pass.jsonl')
        a_path = os.path.join(d, 'barrier_actions.jsonl')
        t0 = time.time()
        while time.time() - t0 < 30:
            if len(jsonl_rows(p_path)) >= 5 and len(jsonl_rows(a_path)) >= 8:
                break
            time.sleep(0.2)
        np_, na = len(jsonl_rows(p_path)), len(jsonl_rows(a_path))
        check(np_ == 5 and na == 8, '落盘 5 条通行 / 8 条动作（实际 %d / %d）' % (np_, na))

        banner('正样本：子进程在环对账，期望 rc==0')
        code, out = run_audit(d, 'selftest')
        fl = fails_of(out)
        check(code == 0, '对账器 rc==0（实际 %s）' % code, tail(out, 30))
        # "子进程崩了"和"判据判挂了"必须分开看：本轮就撞到过一次 ——local() 参数名
        # 和 srv 字段不一致，对账器直接 TypeError，rc!=0 看着像"发现了问题"，
        # 其实什么都没查。traceback 单独点一条。
        check('Traceback' not in out, '对账器子进程没有 traceback（崩≠判挂）',
              out[-800:] if 'Traceback' in out else '')
        if code != 0:
            print('    FAIL 明细：\n' + '\n'.join('      - ' + f for f in fl))
        check(not fl, '零条 FAIL', ' | '.join(fl))
        nd = out.count('\n  ✓')
        check(nd >= 40, '正样本至少跑满 40 条判据（实际 %d 条，太少说明在环分支被跳过了）' % nd)

        banner('负 1：往 JSONL 追加一条伪记录（内存没有），期望 rc!=0 且点名"CSV 数据行"')
        rows = jsonl_rows(p_path)
        fake = dict(rows[-1])
        fake['seq'] = rows[-1]['seq'] + 1
        fake['pass_id'] = 'FAKE-9999'
        fake['image_path'] = rows[-1]['image_path'].replace('P0005', 'FAKE')
        with open(p_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(fake, ensure_ascii=False) + '\n')
        # 换 prefix 是必须的：上一个正样本已经把游标推到最新，再用同一个 prefix 的话
        # 增量导出那几条会先挂，FAIL 一堆反而看不出"CSV 数据行"这条到底咬没咬住。
        code2, out2 = run_audit(d, 'selftest2')
        fl2 = fails_of(out2)
        check(code2 != 0, '注入伪记录后 rc!=0（实际 %s）' % code2, tail(out2, 30))
        check(any('CSV 数据行' in f for f in fl2),
              'FAIL 点名"CSV 数据行"（判据真的会咬账本被动过）', ' | '.join(fl2))

        banner('负 3：账本 mtime 推到 1 小时前，期望 rc!=0 且点名"遗留账本"')
        stale = os.path.join(root, 'stale')
        os.makedirs(stale, exist_ok=True)
        for name in ('parking_pass.jsonl', 'barrier_actions.jsonl'):
            shutil.copyfile(os.path.join(d, name), os.path.join(stale, name))
        old = time.time() - 3600.0
        os.utime(os.path.join(stale, 'parking_pass.jsonl'), (old, old))
        code3, out3 = run_audit(stale, 's', skip_live=True, timeout=60)
        fl3 = fails_of(out3)
        check(code3 != 0, '遗留账本 rc!=0（实际 %s）' % code3, tail(out3))
        check(any('遗留账本' in f for f in fl3), 'FAIL 点名"遗留账本"', ' | '.join(fl3))

        banner('负 4：三条来源分类与撞号 —— 无凭据的收尾落杆 + 重复 pass_id，各咬一条')
        # 这两条是给新判据自己配的负样本。收尾补记那一拍在真机上合法（故障清除/交还自动
        # 之后 manager 补的那一行），但必须数得出凭据；这里塞一条没有任何 故障清除/手动生效
        # 的，判据要挂。pass_id 撞号是管理器重启后从 P0001 重数造成的，账本追加式从不重写。
        l3 = os.path.join(root, 'lineage')
        os.makedirs(l3, exist_ok=True)
        shutil.copyfile(os.path.join(stale, 'barrier_actions.jsonl'),
                        os.path.join(l3, 'barrier_actions.jsonl'))
        shutil.copyfile(os.path.join(d, 'parking_pass.jsonl'),
                        os.path.join(l3, 'parking_pass.jsonl'))
        arows = jsonl_rows(os.path.join(l3, 'barrier_actions.jsonl'))
        prows = jsonl_rows(os.path.join(l3, 'parking_pass.jsonl'))
        low = [x for x in arows if x['action_name'] == '落杆到位' and x['gate_id'] == 'gate_in']
        check(bool(low), '负样本拿得到一条 gate_in 的落杆到位当模板', '动作 %d 行' % len(arows))
        fake_act = dict(low[-1])
        fake_act['seq'] = arows[-1]['seq'] + 1
        fake_act['sim_ts'] = arows[-1]['sim_ts'] + 5.0
        fake_act['note'] = '无在途通行的落杆（手动或故障恢复后的收尾）'
        dup_pass = dict(prows[-1])
        dup_pass['seq'] = prows[-1]['seq'] + 1
        dup_pass['pass_id'] = prows[0]['pass_id']
        dup_pass['sim_ts'] = prows[-1]['sim_ts'] + 9.0
        with open(os.path.join(l3, 'barrier_actions.jsonl'), 'a', encoding='utf-8') as f:
            f.write(json.dumps(fake_act, ensure_ascii=False) + '\n')
        with open(os.path.join(l3, 'parking_pass.jsonl'), 'a', encoding='utf-8') as f:
            f.write(json.dumps(dup_pass, ensure_ascii=False) + '\n')
        code4, out4 = run_audit(l3, 'l', skip_live=True, timeout=60)
        fl4 = fails_of(out4)
        check(code4 != 0, '注入这两条后 rc!=0（实际 %s）' % code4, tail(out4, 24))
        check(any('收尾补记' in f for f in fl4),
              'FAIL 点名"收尾补记"（无凭据的收尾落杆咬得住）', ' | '.join(fl4))
        check(any('pass_id+gate 唯一' in f for f in fl4),
              'FAIL 点名"pass_id+gate 唯一"（撞号咬得住）', ' | '.join(fl4))
        check(all('CSV 数据行' not in f for f in fl4),
              '负 4 只该咬这两条：导出判据跟着挂说明来源分类写错了', ' | '.join(fl4))
    finally:
        ex.shutdown()
        feed.destroy_node()
        node.destroy_node()
    return 0 if not FAILED else 1


if __name__ == '__main__':
    sys.exit(main())
