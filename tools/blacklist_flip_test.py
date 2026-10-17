#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黑名单仿真取证（翻档循环版）：五副在跑的牌整批入黑，缺哪一档就把整张名单改成哪一档。

四代取证脚本的演化，每一代都是被真实流量打的脸：
  1) targeted：换两块牌挂到"历史取景最稳的车"上——那辆车这轮在 J2 十张里九张空号，200 s 零告警；
  2) live：五副牌整批入黑、按号码预分两级——连着两轮（240 s / 250 s）只有 car_3 的京B88990
     被读出来，CRITICAL 押在沪A/浙A 上永远等不到，片内只剩 WARN；
  3) two-phase：拿"当前真被读出的那块牌"先 WARN 后 CRITICAL——WARN 66 s 就落地了，
     但同一辆车再次过卡口要 100~150 s（实测 5 分钟 12 条记录、单车重复 2~3 次），
     130 s 的单档超时差一口气，CRITICAL 又没演上；
  4) 现在这版：**级别挂在名单条目上，不挂在号码上**，所以不必凑"两辆都能被读出的车"，
     也不必等"同一辆车跑两趟"。整张名单先按 WARN 入黑，任意一辆车被读出来就落一条 WARN；
     收到后立刻把整张名单改成还缺的那一档（OP_ADD 原地覆盖），下一辆车（往往 30~60 s 内）
     过卡口就落 CRITICAL。循环到两级齐或总超时。
顺带演全：op=0 增/改、op=2 清空、名单热更新对下一条识别结果立即生效、alert_level 写回导出 CSV。

用法：python3 blacklist_flip_test.py [总超时秒=420] [单条告警等待秒=150]
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from v2x_msgs.msg import PlateAlert
from v2x_msgs.srv import SetBlacklist

LEVEL_CN = {0: 'INFO', 1: 'WARN', 2: 'CRITICAL'}
REASON = {1: '逾期未检', 2: '肇事逃逸'}
# 路面上默认就挂着的五副牌（apply_default_plates.sh 的结果），整批同级入黑。
PLATES = ['沪A12345', '浙A66666', '京B88990', '粤Z1A2B3', '苏E66F88']
CSV_PATH = '/tmp/v2x_records/plate_records.csv'


class Flip(Node):
    def __init__(self, total_s, per_s):
        super().__init__('blacklist_flip_test')
        self.total = total_s
        self.per = per_s
        self.alerts = []
        self.cli = self.create_client(SetBlacklist, '/plate/blacklist')
        self.exp = self.create_client(Trigger, '/plate/export')
        self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 50)

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
        if not client.service_is_ready():
            print('!! 服务 %s 不可用' % client.srv_name)
            return None
        fut = client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        return fut.result()

    def set_level(self, level, note):
        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_ADD
        req.plate_numbers = PLATES
        req.reasons = [REASON[level]] * len(PLATES)
        req.levels = [level] * len(PLATES)
        r = self.call(self.cli, req)
        print('  %s 整张名单（%d 个号）改为 %s 级：%s -> %s' % (
            note, len(PLATES), LEVEL_CN[level], REASON[level], r.message if r else '服务不可用'))
        sys.stdout.flush()

    def clear(self, note):
        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, req)
        print('%s: %s' % (note, r.message if r else '服务不可用'))
        sys.stdout.flush()

    def run(self):
        # 识别器不活着就永远不会命中名单；判活要 spin 着等 DDS 发现，问一次会读到 0
        if not self.spin_until(lambda: self.count_publishers('/plate/result') > 0, 15):
            print('!! /plate/result 无发布者：识别器没跑。先 bash tools/ensure_recognizer.sh')
            return 1
        self.clear('清空名单')
        self.set_level(1, '起手')
        got = set()
        t_all = time.time()
        n = 0
        while rclpy.ok() and time.time() - t_all < self.total and got != {1, 2}:
            before = len(self.alerts)
            t0 = time.time()
            while rclpy.ok() and len(self.alerts) == before and time.time() - t0 < self.per:
                rclpy.spin_once(self, timeout_sec=0.2)
            new = self.alerts[before:]
            if not new:
                print('  %d s 内没有新告警，继续等（车辆过卡口并被读出号码才会有）' % self.per)
                sys.stdout.flush()
                continue
            n += 1
            got |= {a.level for a in new}
            print('  第 %d 次收到告警，已覆盖级别 %s；已用 %d s' % (
                n, sorted(LEVEL_CN[x] for x in got), int(time.time() - t_all)))
            missing = {1, 2} - got
            if missing:
                self.set_level(max(missing), '翻档')
            sys.stdout.flush()

        r = self.call(self.exp, Trigger.Request())
        if r is not None:
            print('导出: %s' % r.message)
        rows = [x for x in csv.DictReader(open(CSV_PATH, newline='', encoding='utf-8-sig'))]
        al = [x for x in rows if (x.get('alert_level') or '').strip() not in ('', 'None', '0')]
        by = {}
        for x in al:
            by[x['alert_level']] = by.get(x['alert_level'], 0) + 1
        print('窗口 %d s 内实时收到 %d 条告警；导出 CSV 共 %d 行，其中带 alert_level 的 %d 行 %s' % (
            int(time.time() - t_all), len(self.alerts), len(rows), len(al), by))
        for x in al[-6:]:
            print('  CSV 行 %s 号=%s 卡口=%s alert_level=%s' % (
                x['capture_id'], x['plate_number'], x['gate_id'], x['alert_level']))
        self.clear('收尾清空名单')
        return 0 if got == {1, 2} else 2

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while rclpy.ok() and not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pred()


def main():
    a = sys.argv[1:]
    total = int(a[0]) if len(a) > 0 and a[0].isdigit() else 420
    per = int(a[1]) if len(a) > 1 and a[1].isdigit() else 150
    rclpy.init()
    node = Flip(total, per)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
