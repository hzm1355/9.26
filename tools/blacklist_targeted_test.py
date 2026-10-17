#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三（三）条目 5 定向取证：让 CRITICAL 与 WARN 两档告警都在**当前这条时线**上落地。

blacklist_export_test.py 只入黑、然后等路面上的车自己过卡口。实测等不到：
黑名单号 浙A66666 挂在 car_5 上，而 car_5 在南环 J2 的取景一直不合格（这一窗口连 5 张空号），
识别读不出号就永远不会命中黑名单，CRITICAL 那条就只能靠运气。所以这里改成受控注入：
把两个黑名单号挂到"每个卡口取景最稳的那辆车"上（从 /tmp/v2x_records/plate_records.csv
按 车辆×卡口 统计命中率挑的），再等告警。要验证的是告警链路（查表 → 发 /plate/alert →
写回 alert_level），谁载牌不影响这条链路。

用法：python3 blacklist_targeted_test.py [等待秒数=300]
跑完自动把两块牌换回原号。
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from v2x_msgs.msg import PlateAlert
from v2x_msgs.srv import SetBlacklist

LEVEL_CN = {0: 'INFO', 1: 'WARN', 2: 'CRITICAL'}
# (黑名单号, 原因, 级别, 载牌车, 期望卡口)
HITS = [
    ('浙A66666', '涉嫌套牌', 2, 'car_1', 'gate_j3_north'),
    ('京B88990', '逾期未检', 1, 'car_4', 'gate_j2_south'),
]
ORIG = {'car_1': '沪A12345', 'car_4': '苏E66F88'}


class Targeted(Node):
    def __init__(self, wait_s):
        super().__init__('blacklist_targeted_test')
        self.wait = wait_s
        self.alerts = []
        self.req = self.create_publisher(String, '/plate/request', 10)
        self.cli = self.create_client(SetBlacklist, '/plate/blacklist')
        self.exp = self.create_client(Trigger, '/plate/export')
        self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 50)

    def on_alert(self, a):
        self.alerts.append(a)
        print('  ** 告警 %s 车牌=%s 级别=%s 原因=%s 卡口=%s 速度=%.2f 置信位=%s' % (
            time.strftime('%H:%M:%S'), a.plate_number,
            LEVEL_CN.get(a.level, str(a.level)), a.reason, a.gate_id,
            a.speed, a.capture_id))

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

    def set_plate(self, model, number):
        # 车尾/车头都换，否则车头牌还是旧号，相机拍到哪面就不确定
        for pos in ('rear', 'front'):
            m = String()
            m.data = '%s|%s|%s' % (model, pos, number)
            self.req.publish(m)
            rclpy.spin_once(self, timeout_sec=0.1)
        print('换牌 %s -> %s' % (model, number))

    def run(self):
        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, q)
        print('先清名单: %s' % (r.message if r else '服务不可用'))

        for h in HITS:
            self.set_plate(h[3], h[0])
        time.sleep(3.0)

        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_ADD
        req.plate_numbers = [h[0] for h in HITS]
        req.reasons = [h[1] for h in HITS]
        req.levels = [h[2] for h in HITS]
        r = self.call(self.cli, req)
        if r is None:
            return 1
        print('加黑名单: %s -> %s' % (r.message, r.blacklist))

        want = {h[0] for h in HITS}
        print('等告警，最多 %d s（想收满 %s）…' % (self.wait, sorted(want)))
        t0 = time.time()
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
        got = {}
        for a in self.alerts:
            got.setdefault(a.plate_number, a)
        print('收到 %d 条告警，覆盖 %d/%d 个车牌；逐号首条：' % (
            len(self.alerts), len(got), len(HITS)))
        for h in HITS:
            a = got.get(h[0])
            if a is None:
                print('  %-9s 未命中（期望 %s / %s）' % (h[0], LEVEL_CN[h[2]], h[4]))
            else:
                print('  %-9s 级别=%s 卡口=%s 抓拍=%s' % (
                    h[0], LEVEL_CN.get(a.level), a.gate_id, a.capture_id))
        base = '/tmp/v2x_records'
        for f in ('plate_records.jsonl', 'plate_records.csv', 'plate_records.json'):
            p = os.path.join(base, f)
            print('  %-22s %s' % (f, '%d 字节' % os.path.getsize(p)
                                  if os.path.exists(p) else '不存在'))
        return 0 if len(got) == len(HITS) else 2

    def restore(self):
        for model, number in ORIG.items():
            self.set_plate(model, number)
        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        self.call(self.cli, q)
        print('已还原车牌并清空名单')


def main():
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 300
    rclpy.init()
    node = Targeted(wait_s)
    try:
        rc = node.run()
    finally:
        # 名单是内存态，牌是材质态；两者都留在盘上会污染后面的测试，必须还原
        node.restore()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
