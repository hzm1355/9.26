#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""黑名单仿真取证（受控注入版 2）：把路面上**当前这五副牌**整批入黑，等告警落地。

为什么不再像 blacklist_targeted_test.py 那样"换两块牌挂到取景最稳的车上"：
  · 换牌是 gzserver 进程内的 SetMaterial，成功与否要看 plate_gen 的 /plate/set 回音，
    多一道可能失败的环节；
  · "取景最稳的车"来自历史统计，而每台车×每个卡口的命中率会随车队状态漂移——
    实测 car_4 在 J2 的 10 张里 9 张是空号，把 CRITICAL 押在它身上就等不来告警；
  · 自然交通下车队还会整体卡死（曾五车同时停摆 7 分钟），200 s 窗口一张牌都没读到。
所以这里改成"五副牌全部入黑，两级都有"：只要窗口内读出**任何**一辆车的号就必然命中
名单，告警链路的验证不再依赖某一辆车的运气。跑完清空名单，路面牌面全程不动。

用法：python3 blacklist_live_test.py [等待秒=240]
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
# 路面上默认就挂着的五副牌（apply_default_plates.sh 的结果），两级混排。
# 级别是按"这辆车现在读不读得出来"分的，不是按原因严重度：car_1(沪A)/car_5(浙A) 走
# 北环与南环的稳定取景位挂 CRITICAL，car_3(京B) 北环命中率 60% 挂 WARN，这样两级都
# 能在窗口内落地；car_2/car_4 当前几乎读不出号（0/4、0/10），挂哪级都只是陪跑。
LIST = [
    ('沪A12345', '肇事逃逸', 2),
    ('浙A66666', '涉嫌套牌', 2),
    ('京B88990', '逾期未检', 1),
    ('粤Z1A2B3', '限行时段闯禁', 1),
    ('苏E66F88', '布控人员车辆', 1),
]
CSV_PATH = '/tmp/v2x_records/plate_records.csv'


class Live(Node):
    def __init__(self, wait_s):
        super().__init__('blacklist_live_test')
        self.wait = wait_s
        self.alerts = []
        self.cli = self.create_client(SetBlacklist, '/plate/blacklist')
        self.exp = self.create_client(Trigger, '/plate/export')
        self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 50)

    def on_alert(self, a):
        self.alerts.append(a)
        print('  ** %s 告警 车牌=%s 级别=%s 原因=%s 卡口=%s 速度=%.2f 抓拍=%s' % (
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

    def run(self):
        # 识别器不活着就永远不会命中名单；判活要 spin 着等 DDS 发现，问一次会读到 0
        if not self.spin_until(lambda: self.count_publishers('/plate/result') > 0, 15):
            print('!! /plate/result 无发布者：识别器没跑。先 bash tools/ensure_recognizer.sh')
            return 1
        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, q)
        print('清空名单: %s' % (r.message if r else '服务不可用'))

        req = SetBlacklist.Request()
        req.op = SetBlacklist.Request.OP_ADD
        req.plate_numbers = [x[0] for x in LIST]
        req.reasons = [x[1] for x in LIST]
        req.levels = [x[2] for x in LIST]
        r = self.call(self.cli, req)
        if r is None:
            return 1
        print('入黑 %d 个号码: %s -> %s' % (len(LIST), [x[0] for x in LIST], r.message))
        print('名单现况: %s' % (r.blacklist,))
        print('等告警，最多 %d s（想同时收到 CRITICAL 与 WARN）…' % self.wait)
        sys.stdout.flush()

        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self.wait:
            lv = {a.level for a in self.alerts}
            if 2 in lv and 1 in lv:
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        used = int(time.time() - t0)

        r = self.call(self.exp, Trigger.Request())
        if r is not None:
            print('导出: %s' % r.message)
        rows = [r for r in csv.DictReader(open(CSV_PATH, newline='', encoding='utf-8-sig'))]
        al = [r for r in rows if (r.get('alert_level') or '').strip() not in ('', 'None', '0')]
        print('窗口 %d s 内收到 %d 条告警；导出 CSV 共 %d 行，其中带 alert_level 的 %d 行' % (
            used, len(self.alerts), len(rows), len(al)))
        byp = {}
        for a in self.alerts:
            byp.setdefault((a.plate_number, LEVEL_CN.get(a.level, a.level)), 0)
            byp[(a.plate_number, LEVEL_CN.get(a.level, a.level))] += 1
        for (p, lv), n in sorted(byp.items(), key=lambda kv: -kv[1]):
            print('  %-10s %-9s %d 次' % (p, lv, n))
        for r in al[-4:]:
            print('  CSV 行 %s 号=%s 卡口=%s alert_level=%s' % (
                r['capture_id'], r['plate_number'], r['gate_id'], r['alert_level']))

        q = SetBlacklist.Request()
        q.op = SetBlacklist.Request.OP_CLEAR
        r = self.call(self.cli, q)
        print('收尾清空名单: %s' % (r.message if r else '服务不可用'))
        lv = {a.level for a in self.alerts}
        return 0 if (1 in lv and 2 in lv) else 2

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while rclpy.ok() and not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pred()


def main():
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 240
    rclpy.init()
    node = Live(wait_s)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
