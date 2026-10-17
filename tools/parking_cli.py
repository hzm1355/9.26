#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车场管控接口命令行：手动抬落杆、故障注入、白名单/临时卡、记录查询与导出。

对应任务五条目 4（管控逻辑）与条目 5（记录管理）的四个 srv：
  /parking/gate        v2x_msgs/srv/SetParkingGate      手动控制 + 故障注入（manager）
  /parking/whitelist   v2x_msgs/srv/SetParkingWhitelist 白名单/临时卡热改（manager）
  /parking/query       v2x_msgs/srv/ParkingQuery        通行记录过滤查询（recorder）
  /parking/export      v2x_msgs/srv/ParkingExport       导出两份 CSV（recorder）

手动控制另有话题通道 /parking/gate_manual_cmd（std_msgs/String，无回包），协议条目 4
里 srv 与话题并列，所以 `manual` 子命令专门走话题那条，srv 那条走 `gate`。

用法：
  parking_cli.py gate list
  parking_cli.py gate raise --gate gate_in
  parking_cli.py gate raise --gate gate_in --angle 0.7854     # 半抬，演"目标角≠实测角"
  parking_cli.py gate lower --gate gate_in
  parking_cli.py gate auto --gate gate_out
  parking_cli.py gate fault --gate gate_out --kind STUCK
  parking_cli.py gate clear --gate gate_out
  parking_cli.py manual gate_in:1.5708                        # 走话题，演"通过话题手动抬落杆"
  parking_cli.py manual gate_in:AUTO
  parking_cli.py wl list
  parking_cli.py wl add --plate 京B88990 --kind monthly --label 月租车-李四
  parking_cli.py wl remove --plate 京B88990
  parking_cli.py wl reset
  parking_cli.py wl temp --plate 京N12345 --label 访客送货
  parking_cli.py wl revoke --plate 京N12345
  parking_cli.py query --plate 88990 --dir in --result allowed --limit 12
  parking_cli.py export --prefix 任务五_条目5
  parking_cli.py export --only-new

--gate 不给就是"两道都作用"（srv 里 gate_id 空串的含义），auto/clear 这类复位操作
用空 gate 更省事。query 的 --t0/--t1 是仿真秒，<=0 表示不设界。
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import BarrierState
from v2x_msgs.srv import ParkingExport, ParkingQuery, SetParkingGate, SetParkingWhitelist

# argparse 只把 `-12`／`-1.5` 这种整串认成负数选项值；`--plate 京A-1` 之类的值不受影响，
# 但 `--gate gate_in:1.5708` 没问题，`--angle -0.5` 会被当成选项名 —— 一律把紧跟选项的
# 独立 token 粘成 --opt=value，两种写法都能用（同 speed_zones_cli.py 的坑）。
OPTS = ('--gate', '--angle', '--kind', '--plate', '--label', '--dir', '--result',
        '--t0', '--t1', '--limit', '--prefix', '--vehicle')


def normalize(argv):
    out, i = [], 0
    while i < len(argv):
        if argv[i] in OPTS and i + 1 < len(argv):
            out.append('%s=%s' % (argv[i], argv[i + 1]))
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


GATE_OPS = {'list': SetParkingGate.Request.OP_LIST,
            'raise': SetParkingGate.Request.OP_RAISE,
            'lower': SetParkingGate.Request.OP_LOWER,
            'auto': SetParkingGate.Request.OP_AUTO,
            'fault': SetParkingGate.Request.OP_FAULT,
            'clear': SetParkingGate.Request.OP_CLEAR_FAULT}

WL_OPS = {'list': SetParkingWhitelist.Request.OP_LIST,
          'add': SetParkingWhitelist.Request.OP_ADD,
          'remove': SetParkingWhitelist.Request.OP_REMOVE,
          'reset': SetParkingWhitelist.Request.OP_RESET,
          'temp': SetParkingWhitelist.Request.OP_ISSUE_TEMP,
          'revoke': SetParkingWhitelist.Request.OP_REVOKE_TEMP}

# 故障名 -> BarrierState 常量。STUCK/RAISE_FAIL/NO_CLOSE/SENSOR_FAIL 是
# parking_zones.yaml 的 control.fault_kinds 列出的四种，名字与 srv 的 uint8 一一对应。
FAULTS = {'STUCK': BarrierState.FAULT_STUCK,
          'RAISE_FAIL': BarrierState.FAULT_RAISE_FAIL,
          'NO_CLOSE': BarrierState.FAULT_NO_CLOSE,
          'SENSOR_FAIL': BarrierState.FAULT_SENSOR_FAIL}

STATE_ZH = {BarrierState.STATE_CLOSED: '落下', BarrierState.STATE_OPENING: '抬杆中',
            BarrierState.STATE_OPEN: '抬起', BarrierState.STATE_CLOSING: '落杆中',
            BarrierState.STATE_FAULT: '故障挂起'}
SRC_ZH = {BarrierState.SRC_AUTO: '自动', BarrierState.SRC_MANUAL: '手动',
          BarrierState.SRC_FAULT: '故障令'}

DIR_Q = {'any': ParkingQuery.Request.DIR_ANY, 'in': ParkingQuery.Request.DIR_IN,
         'out': ParkingQuery.Request.DIR_OUT}
RES_Q = {'any': ParkingQuery.Request.RESULT_ANY,
         'allowed': ParkingQuery.Request.RESULT_ALLOWED,
         'denied': ParkingQuery.Request.RESULT_DENIED,
         'pending': ParkingQuery.Request.RESULT_PENDING}

KIND_INT = {'monthly': 0, 'temp': 1}          # ParkingPass.KIND_MONTHLY / KIND_TEMP


def call(node, srv_type, name, req, timeout_sec=30.0):
    """同步调一个服务。两层坑都要如实报，不能把"没拿到回包"演成"操作成功"：
      1) 刚重启的节点 DDS 发现没完成 —— wait_for_service 超时直接退出；
      2) 服务端已经干了、回包却死在传输层。2026-10-14 实测：`wl remove` 那一条在
         manager 侧真的生效了（随后 wl list 读回就是删掉的名单），但 rclpy 报了
         "failed to send response (timeout): client will not receive response"，
         而原来这个循环没有截止，CLI 就挂在那里 6 分钟，把取证脚本整段拖死。
    超时时按失败退出，并点名"副作用可能已生效"，让下游用只读 op 复核而不是猜。"""
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=15.0):
        print('%s 服务不可用（parking_manager / parking_recorder 没起来？）' % name)
        sys.exit(2)
    fut = cli.call_async(req)
    waited = 0.0
    while rclpy.ok() and not fut.done() and waited < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
    if not fut.done():
        print('%s 在 %.0f s 内没有回包：这条指令的副作用**可能已经生效**'
              '（服务端已产出响应、响应在传输层超时丢弃），要用只读 op（list/query）'
              '读回确认，别把这一行当成"没执行"' % (name, timeout_sec))
        sys.exit(3)
    return fut.result()


def do_gate(node, a):
    req = SetParkingGate.Request()
    req.op = GATE_OPS[a.op]
    req.gate_id = a.gate
    # -999 = 用 YAML 的 open_angle（srv 注释里约定的哨兵值），别传 0，0 是"落到底"。
    req.angle_rad = a.angle if a.angle is not None else -999.0
    req.fault_kind = FAULTS.get(a.kind or 'STUCK', BarrierState.FAULT_STUCK)
    resp = call(node, SetParkingGate, '/parking/gate', req)
    print('success=%s  %s' % (resp.success, resp.message))
    for s, line in zip(resp.states, resp.lines):
        print('  · ' + line)
    return 0 if resp.success else 1


def do_wl(node, a):
    req = SetParkingWhitelist.Request()
    req.op = WL_OPS[a.op]
    req.plate_numbers = [p for p in a.plate.split(',') if p]
    req.kinds = [KIND_INT[a.kind]] * len(req.plate_numbers)
    # 空备注干脆不传：manager 的 OP_ISSUE_TEMP 是 `labels[0] if labels else '人工登记'`，
    # 传 [''] 会把兜底文案顶成一个空字符串，字幕上就看不出这条临时卡是人工发的还是自动发的。
    req.labels = [a.label] if a.label else []
    resp = call(node, SetParkingWhitelist, '/parking/whitelist', req)
    print('success=%s  %s' % (resp.success, resp.message))
    for p, k, lb in zip(resp.whitelist, resp.whitelist_kinds, resp.whitelist_labels):
        print('  · %-10s %s  %s' % (p, '临时' if k == 1 else '月租', lb))
    if resp.tickets:
        print('  在途临时卡：%s' % ', '.join(resp.tickets))
    return 0 if resp.success else 1


def do_query(node, a):
    req = ParkingQuery.Request()
    req.plate_number = a.plate
    req.vehicle_id = a.vehicle
    req.gate_id = a.gate
    req.direction = DIR_Q[a.dir]
    req.result = RES_Q[a.result]
    req.t0 = a.t0
    req.t1 = a.t1
    req.limit = a.limit
    req.newest_first = not a.oldest
    resp = call(node, ParkingQuery, '/parking/query', req)
    print('success=%s  %s' % (resp.success, resp.message))
    print('  内存留存 %d 条｜命中 %d 条｜本次返回 %d 条'
          % (resp.stored, resp.matched, resp.returned))
    for line in resp.lines:
        print('  ' + line)
    return 0 if resp.success else 1


def do_export(node, a):
    req = ParkingExport.Request()
    req.prefix = a.prefix
    req.only_new = a.only_new
    resp = call(node, ParkingExport, '/parking/export', req)
    print('success=%s  %s' % (resp.success, resp.message))
    print('  通行 CSV   %s（%d 行）' % (resp.pass_csv, resp.pass_rows))
    print('  动作 CSV   %s（%d 行）' % (resp.action_csv, resp.action_rows))
    print('  上游 JSONL %s' % resp.pass_jsonl)
    print('  上游 JSONL %s' % resp.action_jsonl)
    return 0 if resp.success else 1


def do_manual(node, payload):
    """话题通道：发一条 gate_id:rad 或 gate_id:AUTO。

    只发一条、发前必须等到订阅者匹配上：manager 每收到一条就落一行 ACT_MANUAL 动作日志，
    为了让"一定送达"而连发三次，读记录的人会数出三次手动抬杆，与画面上的"我按了一次"
    对不上账。get_subscription_count() 已经 >0 时 RELIABLE 的一次发布就有重传保证。
    """
    pub = node.create_publisher(String, '/parking/gate_manual_cmd', 10)
    waited, sub = 0.0, 0
    while waited < 15.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
        sub = pub.get_subscription_count()
        if sub > 0:
            break
    if sub == 0:
        print('没有订阅者（等了 15 s）：/parking/gate_manual_cmd 没人听，'
              'parking_manager 没起来？不发这条指令，免得演成"发了但没生效"')
        return 2
    pub.publish(String(data=payload))
    # 发布是异步的：立刻退出会让这条消息停在 DDS 写队列里，进程没了就没了。
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    print('已发布 gate_manual_cmd=%r（订阅者 %d 个，等 %.1f s 匹配）' % (payload, sub, waited))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='grp', required=True)

    g = sub.add_parser('gate')
    g.add_argument('op', choices=sorted(GATE_OPS))
    g.add_argument('--gate', default='')
    g.add_argument('--angle', type=float, default=None)
    g.add_argument('--kind', default=None, choices=sorted(FAULTS))

    m = sub.add_parser('manual')
    m.add_argument('payload')

    w = sub.add_parser('wl')
    w.add_argument('op', choices=sorted(WL_OPS))
    w.add_argument('--plate', default='')
    w.add_argument('--kind', default='monthly', choices=sorted(KIND_INT))
    w.add_argument('--label', default='')

    q = sub.add_parser('query')
    q.add_argument('--plate', default='')
    q.add_argument('--vehicle', default='')
    q.add_argument('--gate', default='')
    q.add_argument('--dir', default='any', choices=sorted(DIR_Q))
    q.add_argument('--result', default='any', choices=sorted(RES_Q))
    q.add_argument('--t0', type=float, default=-1.0)
    q.add_argument('--t1', type=float, default=-1.0)
    q.add_argument('--limit', type=int, default=12)
    q.add_argument('--oldest', action='store_true')

    e = sub.add_parser('export')
    e.add_argument('--prefix', default='')
    e.add_argument('--only-new', dest='only_new', action='store_true')

    a = ap.parse_args(normalize(sys.argv[1:]))

    if a.grp == 'gate' and a.op == 'fault' and not a.kind:
        print('gate fault 必须带 --kind（STUCK/RAISE_FAIL/NO_CLOSE/SENSOR_FAIL）')
        sys.exit(2)
    if a.grp == 'gate' and a.op == 'raise' and a.angle is not None and a.gate == '':
        print('--angle 只对单道闸有意义：不给 --gate 时两道都会被抬，各用各的 open_angle')
        sys.exit(2)
    if a.grp == 'wl' and a.op in ('add', 'remove', 'temp', 'revoke') and not a.plate:
        print('%s 必须带 --plate（多个用逗号分隔）' % a.op)
        sys.exit(2)

    rclpy.init()
    node = Node('parking_cli')
    try:
        if a.grp == 'manual':
            rc = do_manual(node, a.payload)
        elif a.grp == 'gate':
            rc = do_gate(node, a)
        elif a.grp == 'wl':
            rc = do_wl(node, a)
        elif a.grp == 'query':
            rc = do_query(node, a)
        else:
            rc = do_export(node, a)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
