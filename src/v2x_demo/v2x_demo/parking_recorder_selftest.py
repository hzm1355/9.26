#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车场记录器合成数据自测（不需要 Gazebo，也不 spin，不发 ROS 消息）。

做法：直接拿 ParkingPass / ParkingAction 消息喂节点的 on_pass / on_action 回调，
再直接调 on_query / on_export 两个服务处理函数，然后**读回落盘的 JSONL 与 CSV**做断言。
读文件而不是只读内存是重点 —— 条目 5 交付的是"能查能导的档案"，内存里对得上而磁盘上
缺一行、多一行表头、中文乱码，都是现场才会暴露的失败。

覆盖：
  A 三条通行 + 五条动作全部落盘，CSV 与 JSONL 逐字段一致（含中文车牌/原因）。
  B 整段耗时派生列：放行那条四段相加等于全程；拒绝那条四个 -1 不能算出负耗时。
  C 查询过滤矩阵（车牌子串/方向/结果/闸机/时段/limit/排序），matched 不受 limit 影响，
    回包里的 ParkingPass 与 lines 行必须指同一次通行（防两个列表索引错位）。
    ParkingQuery 的 DIR_IN=1 / RESULT_ALLOWED=1 比 ParkingPass 的 0 大一档，
    这里专门用"只查放行"和"只查进场"两个断言把映射钉住。
  D only_new 增量导出：游标推进、第二次 0 行、再喂一条只多一行，且 CSV 中间不插表头。
  E 环形上限挤出最旧记录：JSONL 仍全，内存与 CSV 少，导出回包三个口径同时报出来。
  F 开机轮转遗留 JSONL：旧文件改名保留，新文件从 0 行开始（否则 pass_id 会重复计数）。
  G prefix 带目录分隔符 → 拒绝，且不落任何文件。
  H 六张枚举名字典覆盖 msg 里全部常量（新增常量时先在这里炸，而不是报告里出现 ?4）。

用法（WSL）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 src/v2x_demo/v2x_demo/parking_recorder_selftest.py [parking_zones.yaml]
退出码 0=PASS 1=FAIL。
"""
import csv
import json
import math
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Quaternion
from rclpy.parameter import Parameter
from v2x_msgs.msg import BarrierState, ParkingAction, ParkingPass
from v2x_msgs.srv import ParkingExport, ParkingQuery

import parking_recorder_node as prn

FAILED = []


def check(cond, what, detail=''):
    if cond:
        print('  ok   %s' % what)
    else:
        FAILED.append(what)
        print('  FAIL %s  %s' % (what, detail))


def banner(t):
    print('\n── %s' % t)


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
    m.image_path = '/tmp/v2x_parking_in/%s.jpg' % pid
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


def act_msg(sim_ts, aid, act, gate='gate_in', fault=BarrierState.FAULT_NONE,
            source=BarrierState.SRC_AUTO, reached=True, to_rad=1.5708,
            elapsed=0.85, vid='car_6', plate='沪A12345', note=''):
    a = ParkingAction()
    a.header.stamp = t_of(sim_ts)
    a.header.frame_id = 'world'
    a.action_id = aid
    a.action = act
    a.gate_id = gate
    a.vehicle_id = vid
    a.plate_number = plate
    a.source = source
    a.fault = fault
    a.from_rad = 0.0
    a.to_rad = to_rad
    a.elapsed_s = elapsed
    a.reached = reached
    a.note = note
    return a


def make_node(cfg, outdir):
    return prn.ParkingRecorderNode(parameter_overrides=[
        Parameter('config_file', Parameter.Type.STRING, cfg),
        Parameter('record_dir', Parameter.Type.STRING, outdir)])


def read_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8') as f:
        return [json.loads(x) for x in f if x.strip()]


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def q(node, **kw):
    req = ParkingQuery.Request()
    req.plate_number = kw.get('plate', '')
    req.vehicle_id = kw.get('vid', '')
    req.gate_id = kw.get('gate', '')
    req.direction = kw.get('direction', ParkingQuery.Request.DIR_ANY)
    req.result = kw.get('result', ParkingQuery.Request.RESULT_ANY)
    req.t0 = kw.get('t0', 0.0)
    req.t1 = kw.get('t1', 0.0)
    req.limit = kw.get('limit', 0)
    req.newest_first = kw.get('newest', True)
    return node.on_query(req, ParkingQuery.Response())


def exp(node, prefix='', only_new=False):
    req = ParkingExport.Request()
    req.prefix = prefix
    req.only_new = only_new
    return node.on_export(req, ParkingExport.Response())


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config',
        'parking_zones.yaml')
    if not os.path.exists(cfg):
        print('找不到配置 %s' % cfg)
        return 2
    out = tempfile.mkdtemp(prefix='parkrec_')
    rclpy.init()
    try:
        run(cfg, out)
    finally:
        rclpy.shutdown()
        shutil.rmtree(out, ignore_errors=True)
    print('\n%s  (%d 项失败)' % ('PASS' if not FAILED else 'FAIL', len(FAILED)))
    return 0 if not FAILED else 1


def run(cfg, out):
    banner('A 三条通行 + 五条动作落盘，CSV 与 JSONL 逐字段一致')
    d = os.path.join(out, 'a')
    node = make_node(cfg, d)
    node.on_pass(pass_msg(100.50, 'P0001-gate_in', '沪A12345', ParkingPass.DIR_IN,
                          'gate_in', ParkingPass.KIND_MONTHLY,
                          ParkingPass.RESULT_ALLOWED, '白名单月租放行',
                          97.20, 98.05, 99.60, 100.50))
    node.on_pass(pass_msg(140.25, 'P0002-gate_out', '沪A12345', ParkingPass.DIR_OUT,
                          'gate_out', ParkingPass.KIND_MONTHLY,
                          ParkingPass.RESULT_ALLOWED, '月租车出场放行',
                          136.00, 137.10, 138.30, 140.25))
    node.on_pass(pass_msg(200.00, 'P0003-gate_in', '粤X00001', ParkingPass.DIR_IN,
                          'gate_in', ParkingPass.KIND_UNKNOWN,
                          ParkingPass.RESULT_DENIED, '未登记且未开临时车放行',
                          198.40, -1.0, -1.0, -1.0, hit=2, ticket=''))
    for i, (ts, aid, act, note) in enumerate((
            (97.30, 'A00001-gate_in', ParkingAction.ACT_RAISE_CMD, '下发抬杆目标角'),
            (98.05, 'A00002-gate_in', ParkingAction.ACT_RAISE, '抬杆到位'),
            (99.60, 'A00003-gate_in', ParkingAction.ACT_LOWER_CMD, '地感释放，落杆'),
            (100.50, 'A00004-gate_in', ParkingAction.ACT_LOWER, '落杆到位'),
            (200.10, 'A00005-gate_in', ParkingAction.ACT_REFUSE, '名单未命中'))):
        node.on_action(act_msg(ts, aid, act, to_rad=(1.5708 if act in (
            ParkingAction.ACT_RAISE_CMD, ParkingAction.ACT_RAISE) else 0.0),
            reached=(act != ParkingAction.ACT_REFUSE), note=note))

    pj, aj = read_jsonl(node.pass_path), read_jsonl(node.act_path)
    check(len(pj) == 3 and len(aj) == 5, 'JSONL 行数 3/5',
          '实际 %d/%d' % (len(pj), len(aj)))
    check(len(node.passes) == 3 and len(node.pass_msgs) == 3, '内存通行环形表 3 条')
    r = exp(node)
    check(r.success and r.pass_rows == 3 and r.action_rows == 5,
          '全量导出回包 3/5 行', r.message)
    pc = read_csv(r.pass_csv)
    ac = read_csv(r.action_csv)
    check(len(pc) == 3 and len(ac) == 5, 'CSV 行数 3/5', '%d/%d' % (len(pc), len(ac)))
    same = all(str(json.dumps(pj[i]['plate_number'], ensure_ascii=False).strip('"')) and
               pc[i]['plate_number'] == pj[i]['plate_number'] and
               pc[i]['pass_id'] == pj[i]['pass_id'] and
               abs(float(pc[i]['raise_dur_s']) - pj[i]['raise_dur_s']) < 1e-9
               for i in range(3))
    check(same, 'CSV 与 JSONL 的 pass_id/车牌/派生耗时逐行一致')
    check(pc[0]['reason'] == '白名单月租放行', 'CSV 中文列不乱码（utf-8-sig）',
          repr(pc[0]['reason']))
    check(ac[1]['action_name'] == '抬杆到位' and ac[4]['action_name'] == '拒绝动作',
          '动作名映射写进 CSV')
    check(abs(float(pc[0]['yaw_deg']) - 90.0) < 0.1, '位姿 yaw 还原成度',
          pc[0]['yaw_deg'])

    banner('B 整段耗时派生列')
    p0 = pc[0]
    segs = [float(p0['raise_dur_s']), float(p0['pass_dur_s']), float(p0['lower_dur_s'])]
    check(all(s > 0 for s in segs), '抬/行/落三段都是正数', str(segs))
    check(abs(sum(segs) - float(p0['cycle_s'])) < 0.01,
          '三段相加 == 全程 %.2f s' % float(p0['cycle_s']),
          '%.3f vs %s' % (sum(segs), p0['cycle_s']))
    check(float(p0['raise_dur_s']) > 0.8 and float(p0['raise_dur_s']) < 0.9,
          '抬杆段 = 实测 0.85 s 量级（不是节点自己数的定时器）', p0['raise_dur_s'])
    p2 = pc[2]
    check([p2['raise_s'], p2['clear_s'], p2['lower_s']] == ['-1.0', '-1.0', '-1.0'],
          '拒绝那条后三段仍是 -1（没被填成 0）', str([p2['raise_s'], p2['clear_s'], p2['lower_s']]))
    # -1 是 msg 里写明的"这一刻没发生"哨兵，合法；要抓的是它被拿去做减法后流出来的
    # 别的负数（-1 减某个真实时刻 = -13.4，看着像"提前抬杆"）。
    bad = [float(p2[k]) for k in ('raise_dur_s', 'pass_dur_s', 'lower_dur_s', 'cycle_s')
           if float(p2[k]) < 0.0 and abs(float(p2[k]) + 1.0) > 1e-9]
    check(not bad, '未发生的段只出 -1 哨兵，不产生其它负耗时', str(bad))

    banner('C 查询过滤矩阵与常量映射')
    check(q(node).matched == 3, '无条件命中 3 条')
    check(q(node, plate='12345').matched == 2, '车牌子串"12345"命中 2 条')
    check(q(node, plate='沪A').returned == 2, '车牌子串"沪A"返回 2 条')
    r = q(node, direction=ParkingQuery.Request.DIR_IN)
    check(r.matched == 2 and all(p.direction == ParkingPass.DIR_IN for p in r.passes),
          'DIR_IN=1 正确映射到消息的 0（进场 2 条）',
          'matched=%d 方向=%s' % (r.matched, [p.direction for p in r.passes]))
    r = q(node, direction=ParkingQuery.Request.DIR_OUT)
    check(r.matched == 1 and r.passes[0].direction == ParkingPass.DIR_OUT,
          'DIR_OUT=2 映射到 1（出场 1 条）', 'matched=%d' % r.matched)
    r = q(node, result=ParkingQuery.Request.RESULT_DENIED)
    check(r.matched == 1 and r.passes[0].result == ParkingPass.RESULT_DENIED,
          'RESULT_DENIED=2 映射到 1（拒绝 1 条）', 'matched=%d' % r.matched)
    r = q(node, result=ParkingQuery.Request.RESULT_ALLOWED, gate='gate_in')
    check(r.matched == 1 and r.passes[0].pass_id == 'P0001-gate_in',
          '放行 + 闸机 gate_in 只剩 1 条', 'matched=%d' % r.matched)
    check(q(node, vid='car_6').matched == 3 and q(node, vid='car_9').matched == 0,
          '车辆编号过滤命中/落空都对')
    r = q(node, t0=101.0, t1=250.0)
    check(r.matched == 2, '时段 [101,250] 按 sim_ts 命中 2 条（把 P0001 排除）',
          'matched=%d ids=%s' % (r.matched, [p.pass_id for p in r.passes]))
    r = q(node, limit=2)
    check(r.matched == 3 and r.returned == 2 and len(r.passes) == 2,
          'limit 截返回、matched 保持全量（对账口径）', r.message)
    check('内存保留' in r.message and '命中 3' in r.message, '回包点名统计口径',
          r.message)
    rn = q(node, limit=2, newest=True)
    rf = q(node, limit=2, newest=False)
    check([p.pass_id for p in rn.passes] == ['P0003-gate_in', 'P0002-gate_out'],
          'newest_first 倒序取最近两条', str([p.pass_id for p in rn.passes]))
    check([p.pass_id for p in rf.passes] == ['P0001-gate_in', 'P0002-gate_out'],
          '正序取最早两条')
    check(rn.lines[0].split()[0] == rn.passes[0].pass_id and
          len(rn.lines) == rn.returned,
          'lines 与 passes 指同一次通行（两列表没错位）',
          '%s vs %s' % (rn.lines[0][:14], rn.passes[0].pass_id))
    check(rn.stored == 3, '回包 stored=内存条数')
    check('到场=' in rn.lines[0] and '全程' in rn.lines[0], '人读行含四时刻与全程')

    banner('D only_new 增量导出')
    r2 = exp(node, only_new=True)
    check(r2.pass_rows == 0 and r2.action_rows == 0, '刚导完再增量=0 行', r2.message)
    node.on_pass(pass_msg(300.00, 'P0004-gate_in', '京B88990', ParkingPass.DIR_IN,
                          'gate_in', ParkingPass.KIND_BLACKLIST,
                          ParkingPass.RESULT_DENIED, '命中黑名单，拒绝抬杆',
                          298.0, -1.0, -1.0, -1.0, hit=3))
    r3 = exp(node, only_new=True)
    check(r3.pass_rows == 1 and r3.action_rows == 0, '再喂一条后增量只有 1 行', r3.message)
    pc2 = read_csv(r3.pass_csv)
    check(len(pc2) == 4, '追加后 CSV 累计 4 行数据', str(len(pc2)))
    check([x['pass_id'] for x in pc2] == ['P0001-gate_in', 'P0002-gate_out',
                                         'P0003-gate_in', 'P0004-gate_in'],
          '增量写在文件尾部，顺序正确', str([x['pass_id'] for x in pc2]))
    body = open(r3.pass_csv, encoding='utf-8-sig').read().splitlines()
    check(sum(1 for x in body if x.startswith('seq,')) == 1,
          '增量追加没有再插一行表头', '表头出现 %d 次' % sum(1 for x in body if x.startswith('seq,')))
    r4 = exp(node, only_new=False)
    check(r4.pass_rows == 4, '全量导出重数一遍是 4 行', r4.message)

    banner('E 环形上限挤出最旧记录')
    d5 = os.path.join(out, 'keep')
    node2 = make_node(cfg, d5)
    node2.keep = 2                      # 真值 5000 测不出截断，这里只把上限调小
    for i in range(4):
        node2.on_pass(pass_msg(400.0 + i, 'P000%d-gate_in' % (i + 1), '沪A12345',
                               ParkingPass.DIR_IN, 'gate_in', ParkingPass.KIND_MONTHLY,
                               ParkingPass.RESULT_ALLOWED, '', 399.0 + i, -1.0, -1.0,
                               -1.0))
    check(len(node2.passes) == 2 and len(node2.pass_msgs) == 2 and node2.drop_pass == 2,
          '内存留 2 条、丢弃计数 2 条')
    check(len(read_jsonl(node2.pass_path)) == 4, 'JSONL 四条一条不少')
    check([p['pass_id'] for p in read_jsonl(node2.pass_path)] ==
          ['P0001-gate_in', 'P0002-gate_in', 'P0003-gate_in', 'P0004-gate_in'],
          'JSONL 顺序与编号完整')
    r5 = exp(node2)
    check(r5.pass_rows == 2 and 'JSONL 行数 通行 4' in r5.message and '丢弃 2' in r5.message,
          '导出回包同时报 CSV 2 / 内存 2 / JSONL 4 / 丢弃 2', r5.message)
    check(len(read_csv(r5.pass_csv)) == 2, '全量 CSV 只有内存里那 2 条（口径已在回包说明）')
    node2.destroy_node()

    banner('F 开机轮转遗留 JSONL')
    d6 = os.path.join(out, 'rotate')
    os.makedirs(d6)
    import yaml
    rec_cfg = yaml.safe_load(open(cfg, encoding='utf-8'))['parking']['records']
    stale = os.path.join(d6, rec_cfg['pass_file'])
    with open(stale, 'w', encoding='utf-8') as f:
        f.write(json.dumps({'pass_id': 'P0001-gate_in'}, ensure_ascii=False) + '\n')
    node3 = make_node(cfg, d6)
    check(len(read_jsonl(stale)) == 0, '当前 JSONL 从 0 行开始', str(read_jsonl(stale)))
    baks = [x for x in os.listdir(d6) if x.startswith(rec_cfg['pass_file'] + '.')]
    check(len(baks) == 1, '旧文件被改名保留而不是覆盖', str(baks))
    with open(os.path.join(d6, baks[0]), encoding='utf-8') as f:
        check(f.read().strip().endswith('}'), '备份内容就是那一行旧记录')
    node3.destroy_node()

    banner('G prefix 校验')
    node4 = make_node(cfg, os.path.join(out, 'prefix'))
    r7 = exp(node4, prefix='run1')
    check(r7.success and os.path.basename(r7.pass_csv) == 'run1_parking_pass.csv',
          '合法前缀进文件名', r7.pass_csv)
    r8 = exp(node4, prefix='../evil')
    check(not r8.success and not os.path.exists(os.path.join(out, 'evil_parking_pass.csv')),
          '带目录分隔符的 prefix 被拒且没落文件', r8.message)
    node4.destroy_node()

    banner('H 枚举名字典覆盖 msg 常量')
    groups = [('DIRECTION_NAME', prn.DIRECTION_NAME,
               [('DIR_IN', ParkingPass.DIR_IN), ('DIR_OUT', ParkingPass.DIR_OUT)]),
              ('KIND_NAME', prn.KIND_NAME,
               [('KIND_MONTHLY', ParkingPass.KIND_MONTHLY),
                ('KIND_TEMP', ParkingPass.KIND_TEMP),
                ('KIND_UNKNOWN', ParkingPass.KIND_UNKNOWN),
                ('KIND_BLACKLIST', ParkingPass.KIND_BLACKLIST)]),
              ('RESULT_NAME', prn.RESULT_NAME,
               [('RESULT_ALLOWED', ParkingPass.RESULT_ALLOWED),
                ('RESULT_DENIED', ParkingPass.RESULT_DENIED),
                ('RESULT_PENDING', ParkingPass.RESULT_PENDING)]),
              ('ACT_NAME', prn.ACT_NAME,
               [('ACT_RAISE', ParkingAction.ACT_RAISE),
                ('ACT_LOWER', ParkingAction.ACT_LOWER),
                ('ACT_RAISE_CMD', ParkingAction.ACT_RAISE_CMD),
                ('ACT_LOWER_CMD', ParkingAction.ACT_LOWER_CMD),
                ('ACT_HOLD_OPEN', ParkingAction.ACT_HOLD_OPEN),
                ('ACT_REFUSE', ParkingAction.ACT_REFUSE),
                ('ACT_FAULT', ParkingAction.ACT_FAULT),
                ('ACT_FAULT_CLEAR', ParkingAction.ACT_FAULT_CLEAR),
                ('ACT_MANUAL', ParkingAction.ACT_MANUAL),
                ('ACT_DEGRADE', ParkingAction.ACT_DEGRADE)]),
              ('SRC_NAME', prn.SRC_NAME,
               [('SRC_AUTO', BarrierState.SRC_AUTO), ('SRC_MANUAL', BarrierState.SRC_MANUAL),
                ('SRC_FAULT', BarrierState.SRC_FAULT)]),
              ('FAULT_NAME', prn.FAULT_NAME,
               [('FAULT_NONE', BarrierState.FAULT_NONE),
                ('FAULT_STUCK', BarrierState.FAULT_STUCK),
                ('FAULT_RAISE_FAIL', BarrierState.FAULT_RAISE_FAIL),
                ('FAULT_NO_CLOSE', BarrierState.FAULT_NO_CLOSE),
                ('FAULT_SENSOR_FAIL', BarrierState.FAULT_SENSOR_FAIL)])]
    miss = ['%s.%s=%d' % (n, attr, val) for n, dic, items in groups
            for attr, val in items if val not in dic]
    check(not miss, '六张表覆盖全部常量', str(miss))
    check(sorted(prn.HIT_NAME) == [0, 1, 2, 3], 'whitelist_hit 四档都有名字')
    check(prn.PASS_FIELDS == [k for k in read_jsonl(node.pass_path)[0]],
          'CSV 列顺序 = JSONL 键顺序（导出没漏列）')

    node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
