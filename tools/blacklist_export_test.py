#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三（三）条目 4/5 取证：黑名单命中告警 + 记录导出。

做三件事，一次跑完：
  1. 往 /plate/blacklist 加两条真值车牌（车在环路上跑，早晚会过卡口）；
  2. 挂着听 /plate/alert，命中就打一行，直到收满或超时；
  3. 调 /plate/export，把 recorder 落盘的记录改写成 CSV/JSON 并回统计。

为什么要等车：告警是"识别结果命中黑名单"触发的，而识别只发生在抓拍时刻，
车过抓拍点约 25 s 一圈，所以这个脚本天然是分钟级的，不能催。

用法：python3 blacklist_export_test.py [等待秒数=180]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from v2x_msgs.msg import PlateAlert
from v2x_msgs.srv import SetBlacklist

LEVEL_CN = {0: 'INFO', 1: 'WARN', 2: 'CRITICAL'}
# 用真值车牌，这样命中与否只取决于"车有没有过卡口"，不掺识别错误。
# car_5 浙A66666 与 car_3 京B88990 分属南环/北环，两条能同时覆盖两个卡口。
HITS = [('浙A66666', '涉嫌套牌', 2), ('京B88990', '逾期未检', 1)]


class BlacklistTest(Node):
    def __init__(self, wait_s):
        super().__init__('blacklist_export_test')
        self.alerts = []
        self.wait = wait_s
        self.cli = self.create_client(SetBlacklist, '/plate/blacklist')
        self.exp = self.create_client(Trigger, '/plate/export')
        self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 50)

    def on_alert(self, a):
        self.alerts.append(a)
        print('  ** 告警 %s 车牌=%s 级别=%s 原因=%s 卡口=%s 置信位=%s' % (
            time.strftime('%H:%M:%S'), a.plate_number,
            LEVEL_CN.get(a.level, str(a.level)), a.reason, a.gate_id,
            a.capture_id))

    def call(self, client, req, timeout=15):
        t0 = time.time()
        while not client.wait_for_service(timeout_sec=2.0) and time.time() - t0 < timeout:
            print('等服务 %s …' % client.srv_name)
        if not client.service_is_ready():
            print('!! 服务 %s 不可用' % client.srv_name)
            return None
        fut = client.call_async(req)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
        return fut.result()

    def run(self):
        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_ADD
        req.plate_numbers = [h[0] for h in HITS]
        req.reasons = [h[1] for h in HITS]
        req.levels = [h[2] for h in HITS]
        r = self.call(self.cli, req)
        if r is None:
            return 1
        print('加黑名单: %s -> %s' % (r.message, r.blacklist))

        # 命中要等车过卡口，所以这里只能按时间窗收。每收一条打一行，
        # 收满 HITS 条或超时就去导出。
        print('等车过卡口触发告警，最多 %d s …' % self.wait)
        t0 = time.time()
        want = {h[0] for h in HITS}
        while rclpy.ok() and time.time() - t0 < self.wait \
                and not want.issubset({a.plate_number for a in self.alerts}):
            rclpy.spin_once(self, timeout_sec=0.2)

        r = self.call(self.exp, Trigger.Request())
        if r is not None:
            print('导出: %s' % r.message)

        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_LIST
        r = self.call(self.cli, q)
        if r is not None:
            print('黑名单仍在: %s' % (r.blacklist,))
        print('共收到 %d 条告警，覆盖 %d/%d 个车牌' % (
            len(self.alerts), len({a.plate_number for a in self.alerts}), len(HITS)))
        base = '/tmp/v2x_records'
        for f in ('plate_records.jsonl', 'plate_records.csv', 'plate_records.json'):
            p = os.path.join(base, f)
            print('  %-22s %s' % (f, '%d 字节' % os.path.getsize(p)
                                  if os.path.exists(p) else '不存在'))
        return 0


def main():
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 180
    rclpy.init()
    node = BlacklistTest(wait_s)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
