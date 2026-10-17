#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黑名单仿真取证（两阶段分级版）：拿"当前真的被读出来的那块牌"演完 WARN 与 CRITICAL 两级。

为什么不再按号码预先分级（blacklist_live_test.py 的做法）：
  · 级别是挂在名单条目上的，不是挂在号码上的——同一个号先按 WARN 入黑命中一次、
    再按 CRITICAL 入黑又命中一次，两级就都落地了，不必凑两块都能被读出来的牌；
  · 实测这一时段五车里只有 car_3(京B88990) 在稳定被读出（250 s 里命中 6 次），
    另外四辆车一张牌都没读出来。把 CRITICAL 押在"历史上读得出"的沪A/浙A 上，
    连着两轮各 240/250 s 都只等到 WARN，整段取证就缺一档；
  · 所以先花几十秒看 /plate/result 现在在出哪个号，拿它做两级实验的载体。
代价是只演一辆车，但两级告警、增删改名单、alert_level 落盘全部在片内闭环。

用法：python3 blacklist_two_phase.py [观察秒=90] [每级等待秒=150]
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from v2x_msgs.msg import PlateAlert, PlateResult
from v2x_msgs.srv import SetBlacklist

LEVEL_CN = {0: 'INFO', 1: 'WARN', 2: 'CRITICAL'}
CSV_PATH = '/tmp/v2x_records/plate_records.csv'
PHASES = [(1, '逾期未检'), (2, '肇事逃逸')]


class TwoPhase(Node):
    def __init__(self, obs_s, wait_s):
        super().__init__('blacklist_two_phase')
        self.obs = obs_s
        self.wait = wait_s
        self.reads = []
        self.alerts = []
        self.cli = self.create_client(SetBlacklist, '/plate/blacklist')
        self.exp = self.create_client(Trigger, '/plate/export')
        self.create_subscription(PlateResult, '/plate/result', self.on_result, 50)
        self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 50)

    def on_result(self, m):
        if m.plate_number:
            self.reads.append(m.plate_number)

    def on_alert(self, a):
        self.alerts.append(a)
        print('    %s 告警 车牌=%s 级别=%s 原因=%s 卡口=%s 速度=%.2f 抓拍=%s' % (
            time.strftime('%H:%M:%S'), a.plate_number, LEVEL_CN.get(a.level, a.level),
            a.reason, a.gate_id, a.speed, a.capture_id))
        sys.stdout.flush()

    def call(self, client, req, timeout=20):
        t0 = time.time()
        while not client.wait_for_service(timeout_sec=2.0) and time.time() - t0 < timeout:
            print('等服务 %s …' % client.srv_name)
            sys.stdout.flush()
        if not service_ready(client):
            print('!! 服务 %s 不可用' % client.srv_name)
            return None
        fut = client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        return fut.result()

    def add(self, num, level, reason):
        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_ADD
        req.plate_numbers = [num]
        req.reasons = [reason]
        req.levels = [level]
        r = self.call(self.cli, req)
        print('  入黑 %s 级别=%s 原因=%s -> %s' % (
            num, LEVEL_CN[level], reason, r.message if r else '服务不可用'))
        sys.stdout.flush()

    def run(self):
        if not self.spin_until(lambda: self.count_publishers('/plate/result') > 0, 15):
            print('!! /plate/result 无发布者：识别器没跑。先 bash tools/ensure_recognizer.sh')
            return 1
        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, q)
        print('清空名单: %s' % (r.message if r else '服务不可用'))

        print('观察 %d s：看现在哪块牌真被卡口读出来，用它做两级载体…' % self.obs)
        sys.stdout.flush()
        if not self.spin_until(lambda: len(self.reads) >= 3, self.obs):
            print('!! %d s 内一条号码都没读出来（车队多半卡死了），先跑 tools/reset_fleet.sh' % self.obs)
            return 1
        from collections import Counter
        top, n = Counter(self.reads).most_common(1)[0]
        print('  窗口内读出 %d 条，最稳的号码 = %s（%d 次）' % (len(self.reads), top, n))
        sys.stdout.flush()

        got = set()
        for level, reason in PHASES:
            before = len(self.alerts)
            self.add(top, level, reason)
            t0 = time.time()
            print('  等 %s 级告警，最多 %d s …' % (LEVEL_CN[level], self.wait))
            sys.stdout.flush()
            while rclpy.ok() and time.time() - t0 < self.wait:
                if any(a.level == level for a in self.alerts[before:]):
                    break
                rclpy.spin_once(self, timeout_sec=0.2)
            hit = [a for a in self.alerts[before:] if a.level == level]
            if hit:
                got.add(level)
                print('  %s 已落地（%d s，%d 条）' % (LEVEL_CN[level], int(time.time() - t0), len(hit)))
            else:
                print('  !! %s 在 %d s 内没落地' % (LEVEL_CN[level], self.wait))
            sys.stdout.flush()

        r = self.call(self.exp, Trigger.Request())
        if r is not None:
            print('导出: %s' % r.message)
        rows = [x for x in csv.DictReader(open(CSV_PATH, newline='', encoding='utf-8-sig'))]
        al = [x for x in rows if (x.get('alert_level') or '').strip() not in ('', 'None', '0')]
        print('窗口内实时收到 %d 条告警；导出 CSV 共 %d 行，其中带 alert_level 的 %d 行' % (
            len(self.alerts), len(rows), len(al)))
        by = {}
        for x in al:
            by[x['alert_level']] = by.get(x['alert_level'], 0) + 1
        print('  CSV 按级别: %s' % by)
        for x in al[-4:]:
            print('  CSV 行 %s 号=%s 卡口=%s alert_level=%s' % (
                x['capture_id'], x['plate_number'], x['gate_id'], x['alert_level']))

        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, q)
        print('收尾清空名单: %s' % (r.message if r else '服务不可用'))
        return 0 if got == {1, 2} else 2

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while rclpy.ok() and not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pred()


def service_ready(cli):
    return cli.service_is_ready()


def main():
    a = sys.argv[1:]
    obs = int(a[0]) if len(a) > 0 and a[0].isdigit() else 90
    wait = int(a[1]) if len(a) > 1 and a[1].isdigit() else 150
    rclpy.init()
    node = TwoPhase(obs, wait)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
