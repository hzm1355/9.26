#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务五（五）条目 5：停车场通行记录与闸机动作日志的落盘、查询、导出。

  输入  /parking/pass      v2x_msgs/ParkingPass       一次通行一条（进场/出场各一条）
  输入  /parking/actions   v2x_msgs/ParkingAction     每一次抬/落/保持/故障/手动一条
  服务  /parking/query     v2x_msgs/srv/ParkingQuery   按车牌/方向/闸机/结果/时段过滤
  服务  /parking/export    v2x_msgs/srv/ParkingExport  两份 CSV + 行数与 JSONL 路径

为什么要独立一个记录器，而不是让 parking_manager 自己写文件：
  manager 是管控逻辑的权威，它一旦兼任写盘，导出服务就要和 0.1 s 的状态机 tick 抢
  同一个执行器 —— 一次 CSV 重写几百毫秒，够把"地感释放后 0.5 s 落杆"的确认窗口吃掉。
  拆开后 manager 只管发，记录器只管收，两边的节拍互不干扰；也符合本仓库既有形态
  （speed_recorder_node / plate_recorder_node 同样是纯订阅+落盘）。

JSONL 是真相，CSV 是投影：
  每条记录**先追加进 JSONL 再进内存环形表**（见 append），所以节点崩了、重启了，
  已经发生过的通行不会丢。反过来说，内存环形表只有"本次启动以来"的量，上限
  records.max_records_in_memory，超了就丢最旧的 —— 导出回包必须同时报 CSV 行数和
  JSONL 行数，两个数只有在没溢出时才相等，报出来才对得上账。

开机轮转旧 JSONL：
  追加式文件会跨重启增长，而 pass_id 的序号在 manager 重启后从 P0001 重新数。两者
  混在同一个文件里，事后读表会出现两条同名的 P0001-gate_in，查询与导出全都对不上。
  所以启动时把已有文件改名成 `<名>.<月日_时分秒>` 再建空文件，本轮的 JSONL 行数
  恒等于本轮内存条数（未溢出时）。轮转了什么、多大，写进启动日志，不静默丢弃。
"""
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from v2x_msgs.msg import BarrierState, ParkingAction, ParkingPass
from v2x_msgs.srv import ParkingExport, ParkingQuery

# CSV 列 = JSONL 键。写盘用同一份字典，导出只是给它套个头，不做二次加工 ——
# 这样"CSV 里那一行"和"JSONL 里那一行"是同一个事实，不会出现两份口径。
PASS_FIELDS = ['seq', 'ts', 'sim_ts', 'pass_id', 'direction', 'direction_name',
               'gate_id', 'plate_number', 'recognized', 'plate_confidence',
               'capture_id', 'image_path', 'vehicle_id', 'x', 'y', 'z', 'yaw_deg',
               'kind', 'kind_name', 'result', 'result_name', 'reason', 'ticket_id',
               'whitelist_hit', 'whitelist_hit_name',
               'arrive_s', 'raise_s', 'clear_s', 'lower_s',
               'raise_dur_s', 'pass_dur_s', 'lower_dur_s', 'cycle_s']
ACT_FIELDS = ['seq', 'ts', 'sim_ts', 'action_id', 'action', 'action_name', 'gate_id',
              'vehicle_id', 'plate_number', 'source', 'source_name', 'fault',
              'fault_name', 'from_rad', 'to_rad', 'elapsed_s', 'reached', 'note']

DIRECTION_NAME = {ParkingPass.DIR_IN: '进场', ParkingPass.DIR_OUT: '出场'}
KIND_NAME = {ParkingPass.KIND_MONTHLY: '月租', ParkingPass.KIND_TEMP: '临时',
             ParkingPass.KIND_UNKNOWN: '未登记', ParkingPass.KIND_BLACKLIST: '黑名单'}
RESULT_NAME = {ParkingPass.RESULT_ALLOWED: '放行', ParkingPass.RESULT_DENIED: '拒绝',
               ParkingPass.RESULT_PENDING: '挂起'}
HIT_NAME = {0: '未查', 1: '命中', 2: '未命中', 3: '黑名单'}
ACT_NAME = {ParkingAction.ACT_RAISE: '抬杆到位', ParkingAction.ACT_LOWER: '落杆到位',
            ParkingAction.ACT_RAISE_CMD: '下发抬杆', ParkingAction.ACT_LOWER_CMD: '下发落杆',
            ParkingAction.ACT_HOLD_OPEN: '防砸重抬', ParkingAction.ACT_REFUSE: '拒绝动作',
            ParkingAction.ACT_FAULT: '故障生效', ParkingAction.ACT_FAULT_CLEAR: '故障清除',
            ParkingAction.ACT_MANUAL: '手动生效', ParkingAction.ACT_DEGRADE: '地感降级'}
SRC_NAME = {BarrierState.SRC_AUTO: '自动', BarrierState.SRC_MANUAL: '手动',
            BarrierState.SRC_FAULT: '故障'}
FAULT_NAME = {BarrierState.FAULT_NONE: 'NONE', BarrierState.FAULT_STUCK: 'STUCK',
              BarrierState.FAULT_RAISE_FAIL: 'RAISE_FAIL',
              BarrierState.FAULT_NO_CLOSE: 'NO_CLOSE',
              BarrierState.FAULT_SENSOR_FAIL: 'SENSOR_FAIL'}


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def yaw_of(q):
    return math.degrees(math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def jsonl_lines(path):
    """数 JSONL 的行。导出回包要拿它跟 CSV 行数对账，所以单独封一个：读不到就返回
    None，让回包如实写"未知"，而不是把"数不出来"报成 0。"""
    try:
        with open(path, encoding='utf-8') as f:
            return sum(1 for _ in f)
    except OSError:
        return None


def dur(a, b):
    """两段时刻之差。ParkingPass 里未到那一刻的字段是 -1（见 msg 注释），
    任何一侧是 -1 就返回 -1，不要算出 -13.4 这种看着像"提前抬杆"的负数。"""
    if a is None or b is None or a < 0.0 or b < 0.0:
        return -1.0
    return round(b - a, 3)


class ParkingRecorderNode(Node):
    def __init__(self, **kw):
        # **kw 透传 parameter_overrides：parking_recorder_selftest 要直接吃源码树里的
        # parking_zones.yaml 和一个临时 outdir，不等 colcon 装 share。
        super().__init__('parking_recorder', **kw)
        self.declare_parameter('config_file', '')
        self.declare_parameter('record_dir', '')
        path = self.get_parameter('config_file').value or os.path.join(
            get_package_share_directory('v2x_demo'), 'config', 'parking_zones.yaml')
        cfg = self.load_config(path)
        rec = cfg['records']
        ctl = cfg['control']
        # record_dir 非空时覆盖 yaml：自测和取证要把产物关进临时目录，不能写 /tmp 公共区
        self.dir = self.get_parameter('record_dir').value or rec['dir']
        self.keep = int(rec.get('max_records_in_memory', 5000))
        self.encoding = rec.get('csv_encoding', 'utf-8-sig')
        self.pass_path = os.path.join(self.dir, rec['pass_file'])
        self.act_path = os.path.join(self.dir, rec['action_file'])
        os.makedirs(self.dir, exist_ok=True)
        self.rotate()

        self.passes = []          # [rec_dict]，最新在末尾，与 pass_msgs 同增同删
        self.pass_msgs = []
        self.acts = []
        self.pass_seen = 0        # 累计追加过多少条（截断后仍单调，导出游标用它）
        self.act_seen = 0
        self.drop_pass = 0        # 被环形上限挤掉的条数，导出时要报名
        self.drop_act = 0
        self.exp_pass = 0         # 上次导出时的 pass_seen 游标
        self.exp_act = 0

        qos = 50
        self.create_subscription(ParkingPass, ctl['pass_topic'], self.on_pass, qos)
        self.create_subscription(ParkingAction, ctl['action_topic'], self.on_action, qos)
        self.create_service(ParkingQuery, '/parking/query', self.on_query)
        # 服务名沿用 records.export_topic 那个键：yaml 注释里写明了它"实为 srv"，
        # 这里就别再造一个同名话题，两处都注册会让看板以为有两条导出通道。
        self.create_service(ParkingExport, rec['export_topic'], self.on_export)

        self.get_logger().info(
            '[park-rec] 记录目录 %s｜通行 %s、动作 %s｜内存上限 %d 条｜'
            '服务 /parking/query、%s' % (
                self.dir, os.path.basename(self.pass_path),
                os.path.basename(self.act_path), self.keep, rec['export_topic']))

    @staticmethod
    def load_config(path):
        import yaml
        with open(path, encoding='utf-8') as fh:
            return yaml.safe_load(fh)['parking']

    # ── 开机轮转 ──────────────────────────────────────────────────────────
    def rotate(self):
        for p in (self.pass_path, self.act_path):
            try:
                size = os.path.getsize(p)
            except OSError:
                continue
            if size <= 0:
                continue
            bak = '%s.%s' % (p, time.strftime('%m%d_%H%M%S'))
            os.rename(p, bak)
            self.get_logger().info(
                '[park-rec] 上一轮遗留 %s（%.1f KB）已轮转为 %s' % (
                    os.path.basename(p), size / 1024.0, os.path.basename(bak)))

    # ── 落盘 ──────────────────────────────────────────────────────────────
    def append(self, lst, path, rec, para=None, obj=None):
        # 顺序有讲究：先写文件再进内存。反过来写的话，磁盘写失败异常一抛，内存里已经
        # 多了一条，导出的 CSV 就会比 JSONL 多一行 —— 事后没法解释。
        with open(path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        lst.append(rec)
        # para 是原始消息的另一份账：查询回包里 ParkingPass[] 必须给真消息，
        # 不能从字典反向拼（拼出来会丢字段，而且 header 是时间戳对象不是浮点）。
        # 它和 lst 必须同增同删，否则索引错位，查到的那条和显示的那条不是同一次通行。
        if para is not None:
            para.append(obj)
        dropped = len(lst) > self.keep
        if dropped:
            lst.pop(0)
            if para is not None:
                para.pop(0)
        return dropped

    def on_pass(self, m):
        self.pass_seen += 1
        rec = {
            'seq': self.pass_seen,
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
            'pass_id': m.pass_id,
            'direction': int(m.direction),
            'direction_name': DIRECTION_NAME.get(m.direction, '?%d' % m.direction),
            'gate_id': m.gate_id,
            'plate_number': m.plate_number,
            'recognized': bool(m.recognized),
            'plate_confidence': round(float(m.plate_confidence), 4),
            'capture_id': m.capture_id,
            'image_path': m.image_path,
            'vehicle_id': m.vehicle_id,
            'x': round(float(m.pose.position.x), 3),
            'y': round(float(m.pose.position.y), 3),
            'z': round(float(m.pose.position.z), 3),
            'yaw_deg': round(yaw_of(m.pose.orientation), 2),
            'kind': int(m.kind),
            'kind_name': KIND_NAME.get(m.kind, '?%d' % m.kind),
            'result': int(m.result),
            'result_name': RESULT_NAME.get(m.result, '?%d' % m.result),
            'reason': m.reason,
            'ticket_id': m.ticket_id,
            'whitelist_hit': int(m.whitelist_hit),
            'whitelist_hit_name': HIT_NAME.get(int(m.whitelist_hit), '?%d' % m.whitelist_hit),
            'arrive_s': round(float(m.arrive_s), 3),
            'raise_s': round(float(m.raise_s), 3),
            'clear_s': round(float(m.clear_s), 3),
            'lower_s': round(float(m.lower_s), 3),
            # 整段耗时：条目 5 要"闸机抬杆→车辆通行→落杆"逐段可查，四个原始时刻相减
            # 就够了，别等到看板上再算，算错了没人知道。
            'raise_dur_s': dur(m.arrive_s, m.raise_s),
            'pass_dur_s': dur(m.raise_s, m.clear_s),
            'lower_dur_s': dur(m.clear_s, m.lower_s),
            'cycle_s': dur(m.arrive_s, m.lower_s),
        }
        dropped = self.append(self.passes, self.pass_path, rec, self.pass_msgs, m)
        if dropped:
            self.drop_pass += 1
        self.get_logger().info(
            '[park-rec] %s %s %s 牌=%s %s/%s%s｜抬 %.2f 行 %.2f 落 %.2f 全程 %.2f s%s' % (
                rec['pass_id'], rec['direction_name'], rec['gate_id'],
                rec['plate_number'] or '无', rec['kind_name'], rec['result_name'],
                ('（卡 %s）' % rec['ticket_id']) if rec['ticket_id'] else '',
                rec['raise_dur_s'], rec['pass_dur_s'], rec['lower_dur_s'],
                rec['cycle_s'], '' if rec['reason'] == '' else '｜' + rec['reason']))

    def on_action(self, m):
        self.act_seen += 1
        rec = {
            'seq': self.act_seen,
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
            'action_id': m.action_id,
            'action': int(m.action),
            'action_name': ACT_NAME.get(m.action, '?%d' % m.action),
            'gate_id': m.gate_id,
            'vehicle_id': m.vehicle_id,
            'plate_number': m.plate_number,
            'source': int(m.source),
            'source_name': SRC_NAME.get(m.source, '?%d' % m.source),
            'fault': int(m.fault),
            'fault_name': FAULT_NAME.get(m.fault, '?%d' % m.fault),
            'from_rad': round(float(m.from_rad), 4),
            'to_rad': round(float(m.to_rad), 4),
            'elapsed_s': round(float(m.elapsed_s), 3),
            'reached': bool(m.reached),
            'note': m.note,
        }
        if self.append(self.acts, self.act_path, rec):
            self.drop_act += 1
        self.get_logger().info(
            '[park-rec] 动作 %s %s %s %s→%.2f rad %.2f s %s%s' % (
                rec['action_id'], rec['gate_id'], rec['action_name'],
                rec['source_name'], rec['to_rad'], rec['elapsed_s'],
                '到位' if rec['reached'] else '未到位',
                '' if rec['fault_name'] == 'NONE' else ' 故障=' + rec['fault_name']))

    # ── 查询 ──────────────────────────────────────────────────────────────
    def on_query(self, req, resp):
        # ParkingQuery 的 DIR_*/RESULT_* 常量比 ParkingPass 大 1（0 留给 ANY），
        # 所以过滤前先映射回消息里的取值；漏了这一步，"只查放行"会筛出拒绝。
        want_dir = {req.DIR_IN: ParkingPass.DIR_IN,
                    req.DIR_OUT: ParkingPass.DIR_OUT}.get(req.direction)
        want_res = {req.RESULT_ALLOWED: ParkingPass.RESULT_ALLOWED,
                    req.RESULT_DENIED: ParkingPass.RESULT_DENIED,
                    req.RESULT_PENDING: ParkingPass.RESULT_PENDING}.get(req.result)
        plate = (req.plate_number or '').strip()
        vid = (req.vehicle_id or '').strip()
        gid = (req.gate_id or '').strip()

        hits = []
        for i, rec in enumerate(self.passes):
            if plate and plate not in rec['plate_number']:
                continue
            if vid and vid != rec['vehicle_id']:
                continue
            if gid and gid != rec['gate_id']:
                continue
            if want_dir is not None and rec['direction'] != want_dir:
                continue
            if want_res is not None and rec['result'] != want_res:
                continue
            # 时段过滤吃的是 sim_ts（= 本条记录成立那一刻，放行帧），不是 arrive_s。
            # 两者对同一辆车差的就是核验+抬杆那几秒，报告里要点名用的是哪个。
            if req.t0 > 0 and rec['sim_ts'] < req.t0:
                continue
            if req.t1 > 0 and rec['sim_ts'] > req.t1:
                continue
            hits.append(i)

        hits.sort(key=lambda i: self.passes[i]['sim_ts'], reverse=bool(req.newest_first))
        matched = len(hits)
        take = hits if req.limit <= 0 else hits[:req.limit]
        resp.success = True
        resp.passes = [self.pass_msgs[i] for i in take]
        resp.lines = [self.pass_line(self.passes[i]) for i in take]
        resp.matched = matched
        resp.returned = len(take)
        resp.stored = len(self.passes)
        resp.message = ('命中 %d 条，返回 %d 条（limit=%d，%s序，按 sim_ts 排序）；'
                        '内存保留 %d 条（上限 %d，本次启动累计 %d 条，因上限丢弃 %d 条）'
                        '；过滤 牌含%r 车=%r 闸=%r 方向=%s 结果=%s 时段[%s, %s]' % (
                            matched, len(take), req.limit,
                            '倒' if req.newest_first else '正',
                            len(self.passes), self.keep, self.pass_seen, self.drop_pass,
                            plate or '*', vid or '*', gid or '*',
                            {0: '不限', 1: '进', 2: '出'}.get(req.direction, '?'),
                            {0: '不限', 1: '放行', 2: '拒绝', 3: '挂起'}.get(req.result, '?'),
                            '%.2f' % req.t0 if req.t0 > 0 else '-inf',
                            '%.2f' % req.t1 if req.t1 > 0 else '+inf'))
        return resp

    @staticmethod
    def pass_line(rec):
        """人读一行。宽度按取证时直接贴进报告排版来的：定长数字、竖线分栏。"""
        return ('%-14s %-4s %-9s 牌=%-10s %s/%s 命中=%s 识别=%.2f '
                '到场=%8.2f 抬到位=%8.2f 离场=%8.2f 落到位=%8.2f '
                '抬%.2f 行%.2f 落%.2f 全程%.2f s %s' % (
                    rec['pass_id'], rec['direction_name'], rec['gate_id'],
                    rec['plate_number'] or '无', rec['kind_name'], rec['result_name'],
                    rec['whitelist_hit_name'], rec['plate_confidence'],
                    rec['arrive_s'], rec['raise_s'], rec['clear_s'], rec['lower_s'],
                    rec['raise_dur_s'], rec['pass_dur_s'], rec['lower_dur_s'],
                    rec['cycle_s'], rec['reason']))

    # ── 导出 ──────────────────────────────────────────────────────────────
    def write_csv(self, path, fields, recs, header_only_if_new):
        """utf-8-sig 让 Excel 双击打开中文车牌与表头不乱码（与卡口/测速同规矩）。
        header_only_if_new=True 时是 only_new 的追加写：文件已有内容就不再写头，
        否则第二次导出的 CSV 中间会插一行表头，csv.DictReader 读出一行假记录。"""
        append = header_only_if_new and os.path.exists(path) and os.path.getsize(path) > 0
        with open(path, 'a' if append else 'w', newline='', encoding=self.encoding) as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            if not append:
                w.writeheader()
            w.writerows(recs)
        return path

    def on_export(self, req, resp):
        resp.pass_jsonl = self.pass_path
        resp.action_jsonl = self.act_path
        prefix = req.prefix or ''
        if os.path.dirname(prefix) or '\\' in prefix:
            # prefix 只当文件名前缀用。带路径就拒掉：这个 srv 的调用方是看板和命令行，
            # 没有"导到别处"的需求，接受它等于让一次调用能往任意目录写文件。
            resp.success = False
            resp.message = ('prefix 只能是文件名前缀，不能带目录分隔符，收到 %r' % prefix)
            return resp
        stem = (prefix + '_') if prefix else ''

        if req.only_new:
            prow = [r for r in self.passes if r['seq'] > self.exp_pass]
            arow = [r for r in self.acts if r['seq'] > self.exp_act]
        else:
            prow, arow = list(self.passes), list(self.acts)
        p_path = os.path.join(self.dir, stem + 'parking_pass.csv')
        a_path = os.path.join(self.dir, stem + 'barrier_actions.csv')
        self.write_csv(p_path, PASS_FIELDS, prow, req.only_new)
        self.write_csv(a_path, ACT_FIELDS, arow, req.only_new)
        self.exp_pass = self.pass_seen
        self.exp_act = self.act_seen

        # 对账三件套：本次 CSV 行数、内存条数、JSONL 行数。only_new 时 CSV 是增量，
        # 与 JSONL 的差就是"此前已导出 + 被环形挤掉"，所以行数字段全报出来，
        # 读的人不需要知道内部游标也能判断这次导出对不对。
        pj, aj = jsonl_lines(self.pass_path), jsonl_lines(self.act_path)
        resp.success = True
        resp.pass_rows = len(prow)
        resp.action_rows = len(arow)
        resp.pass_csv, resp.action_csv = p_path, a_path
        resp.message = ('%s导出 通行 %d 行、动作 %d 行 -> %s / %s；'
                        '内存 通行 %d/累计 %d（丢弃 %d）、动作 %d/累计 %d（丢弃 %d）；'
                        'JSONL 行数 通行 %s、动作 %s；编码 %s' % (
                            '增量' if req.only_new else '全量',
                            len(prow), len(arow), os.path.basename(p_path),
                            os.path.basename(a_path),
                            len(self.passes), self.pass_seen, self.drop_pass,
                            len(self.acts), self.act_seen, self.drop_act,
                            pj, aj, self.encoding))
        self.get_logger().info('[park-rec] ' + resp.message)
        return resp


def main():
    rclpy.init()
    node = ParkingRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
