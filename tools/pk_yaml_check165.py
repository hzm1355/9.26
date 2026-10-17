#!/usr/bin/env python3
"""恢复 parking_zones.yaml 之后的体检：解析它，并把运行期实测过的值逐项点名比对。

为什么不能只看"文件字节数正常"就宣布恢复完成：这份 yaml 是三处消费者的唯一真值
（gen_parking_world.py 生成 world、manager 判定几何、shuttle 路线），一个数字回到旧版
不会报错，只会让闸机按旧几何抬落杆——而 world 里已经是新几何，两边一错开就是砸车或
永远检不到车。所以判据用"节点今天实际跑出来的那些数"，不用 mtime、不用行数。

期望值基线 = 2026-10-16 在环真值（下面四处各自改过一轮，任何一处回到旧版都必须 FAIL）：
  #79 入口线圈南扩 → 方案甲三修 -10.90/[1.30,1.10]；#99 车位东扩 → lot x1=16.50；
  等待位南移 (12.00,-13.95)（pk184 净空判据）；#122 提速 0.70/0.98（v/ω 比值不变）。
顶层 entry_route/exit_route/reset_route 仍是 car_6 那版的**默认航线**，运行期被
shuttle.fleets.<模型> 整段覆盖 —— 现在在环的是 car_3，所以两段都要体检：默认段证明
条目 3 的标定没漂，fleets 段证明真正在跑的那条线没被改回去。

用法: pk_yaml_check165.py <yaml 路径>
"""
import sys

import yaml

P = sys.argv[1]
with open(P, encoding='utf-8') as fh:
    pk = yaml.safe_load(fh)['parking']

# 左=期望值（全部来自今天/昨天在环实测与片内字幕），右=这份文件里的值
GATES = {g['id']: g for g in pk['gates']}
FAIL = []


def eq(label, got, want):
    same = got == want
    print('  %-46s %-22s %s' % (label, got, '✓' if same else '!! 期望 %s' % (want,)))
    if not same:
        FAIL.append(label)


print('=== 占地/高程（world 生成侧）===')
# x1=16.50 是 #99 东扩（第三个原始车位 13.377..16.453 要纳入场内）；回到 13.00 就等于
# P3 那一列（x=14.915）停在界外，条目 1 的"车位可配置"和条目 5 的在场车就都不是真的。
eq('lot 矩形', [pk['lot']['x0'], pk['lot']['x1'], pk['lot']['y0'], pk['lot']['y1']],
   [5.80, 16.50, -13.80, -5.50])
eq('road_z / floor_z / stripe_z',
   [pk['road_z'], pk['floor_z'], pk['stripe_z']], [6.70, 6.712, 6.715])
eq('车体长宽', [pk['vehicle']['length'], pk['vehicle']['width']], [1.60, 0.80])
eq('axle_ahead（轮轴前移 0.40）', pk['vehicle']['axle_ahead'], 0.40)

print('=== 道闸动力学（条目 3 片内读回过的那一组）===')
b = pk['barrier']
eq('open/tol', [b['open_angle'], b['angle_tolerance']], [1.5708, 0.08])
eq('hold / clearance', [b['hold_time_s'], b['clearance_delay_s']], [1.5, 0.5])
eq('抬/落超时', [b['raise_timeout_s'], b['lower_timeout_s']], [5.0, 5.0])
eq('effort/velocity/damping/friction',
   [b['effort'], b['velocity'], b['damping'], b['friction']], [300.0, 2.0, 0.1, 0.05])
eq('state_rate / jam_effort_ratio', [b['state_rate'], b['jam_effort_ratio']], [20.0, 0.90])

print('=== 入口闸几何（#79 挪线圈、#90 拍车头牌、方案甲加宽的结果必须在）===')
gi = GATES['gate_in']
# 方案甲三修（2026-10-16 上线，pk185 实测把"开到位 3.30 s 就落杆"的病灶治掉）：
# 线圈跨到杆轴以南、y 向 0.80→1.10，让占用框南缘从 -12.35 退到 -12.65，核验位 -12.21
# 的余量 0.14→0.44 m（必须盖住 arrive_radius 0.12 的到位残差）。北缘 -9.15 一字不动。
eq('gate_in loop.center（方案甲 -10.90）', gi['loop']['center'], [12.00, -10.90])
eq('gate_in loop.size（y 向加宽到 1.10）', gi['loop']['size'], [1.30, 1.10])
eq('gate_in queue_point（核验位，实测停在 12.00,-12.21）', gi['queue_point'],
   [12.00, -12.21])
# 等待位南移（pk184 净空判据）：旧位 -12.80 静止离 focus 只 1.59 m，在 near 1.4 圈外但
# 余量太薄，且 reset 末段那个原地转的扫掠最近点只有 0.79 m 会误拍；现在 2.74 m/1.94 m。
eq('gate_in staging_point（等待位 -13.95）', gi['staging_point'], [12.00, -13.95])
eq('gate_in camera.focus（车头牌）', gi['camera']['focus'], [12.00, -11.21, 6.85])
eq('gate_in near/far_radius（1.4 触发 / 2.5 复位）',
   [gi['camera']['near_radius'], gi['camera']['far_radius']], [1.4, 2.5])
eq('gate_in rearm_on_fail / frame_wait',
   [gi['camera']['rearm_on_fail'], gi['camera']['frame_wait']], [True, 3.0])
eq('gate_in capture_dir', gi['camera']['capture_dir'], '/tmp/v2x_parking_in')

print('=== 出口闸几何（#92 把转向点挪出触发圈的结果必须在）===')
go = GATES['gate_out']
eq('gate_out queue_point（实测 6.80,-9.95）', go['queue_point'], [6.80, -9.95])
eq('gate_out camera.focus', go['camera']['focus'], [6.80, -10.95, 6.85])
eq('gate_out loop.center', go['loop']['center'], [6.80, -10.75])
# 出口那道线圈**故意不跟着方案甲加宽**：它的核验位 -9.95 距框北缘 -9.15 有 0.80 m，
# 本来就不紧；若也南扩，reset#0 (-12.55) 会落进框内 0.10 m，落杆会被防砸重抬多演一次
# ——那是新增噪声不是修缺陷。所以这一行是"别动"的守卫，不是漏改。
eq('gate_out loop.size（不随入口南扩，保持 0.80）', go['loop']['size'], [1.30, 0.80])
eq('gate_out staging_point（出口外等候区，随 reset#0 北移到 -12.55）',
   go['staging_point'], [6.80, -12.55])
eq('gate_out capture_dir', go['camera']['capture_dir'], '/tmp/v2x_parking_out')

print('=== 摆渡车路线（运行期日志那句"航点 17 个（进场 7/出场 7/复位 3）"）===')
s = pk['shuttle']
er, xr, rr = s['entry_route'], s['exit_route'], s['reset_route']
eq('航点数 进/出/复位', [len(er), len(xr), len(rr)], [7, 7, 3])
eq('cruise/turn/arrive_radius',
   [s['cruise_speed'], s['turn_speed'], s['arrive_radius']], [0.85, 1.19, 0.12])
# 0.85/1.19 是 #128 第三轮提速（2026-10-17）；0.70/0.98 是 #122 第二轮。三轮都是成倍改，
# 比值 0.714 一字不动 —— 行车支饱和时的最小转弯半径 r=v/ω 就是航线几何本身，单改 v
# 会把 0.71 m 的转弯半径撑大，原来贴着装好的转向点就进不了到位圈。
# 对齐 signal_drive 的 cruise_speed 也是刻意的（那边同批 0.7→0.85）。
eq('v/ω 比值（提速必须成倍改，比值不能变）',
   round(s['cruise_speed'] / s['turn_speed'], 3), 0.714)
eq('loop_period_s', s['loop_period_s'], 120.0)
eq('entry 过杆后那格（#92：必须在 1.4 m 圈外）', er[2], [12.00, -9.55, 1.5708])
eq('entry 原地转成西行那格', er[3], [12.00, -9.55, 3.14159])
eq('entry 终点车位 P1', er[6], [8.40, -7.95, 1.5708])
eq('exit 西行段走 y=-9.00（不是 -9.55）', xr[1], [8.40, -9.00, -1.5708])
eq('exit 核验位', xr[5], [6.80, -9.95, -1.5708])
eq('reset 终点=等待位朝向正北', rr[2], [12.00, -12.80, 1.5708])

print('=== 在环分队航线 shuttle.fleets.car_3（条目 4/5 的驱动就是这一条，不是上面那三段）===')
# 顶层那三段是 car_6 那版的默认航线（模型还在 world 里但不起节点），运行期被
# fleets.<模型> 整段覆盖。2026-10-16 用户指令"只用 car_3 在停车场里、car_2 回归正常
# 行驶赛道"，start_parking_nodes.sh 的 SHUTTLE_MODELS 默认就是 car_3 —— 所以这一段的
# 每一个数都是今天画面里真跑出来的；顶层那几行只证明条目 3 的标定没被改回去。
f3 = s['fleets']['car_3']
fe, fx, fr = f3['entry_route'], f3['exit_route'], f3['reset_route']
eq('car_3 航点数 进/出/复位', [len(fe), len(fx), len(fr)], [7, 7, 3])
eq('car_3 entry#0 等待位（与 gate_in staging_point 同格）', fe[0], gi['staging_point'] + [1.5708])
eq('car_3 entry#1 核验位（与 gate_in queue_point 同格）', fe[1], gi['queue_point'] + [1.5708])
eq('car_3 entry#6 车位 P3（东那位，列取标线中心 14.915）', fe[6], [14.915, -7.95, 1.5708])
eq('car_3 exit#3 西行带 y=-8.90（给入口地感北缘留 0.25 m，防防砸重抬多演）',
   fx[3], [6.80, -8.90, 3.14159])
eq('car_3 exit#5 出场核验位', fx[5], [6.80, -9.95, -1.5708])
eq('car_3 exit#6 过杆驶出=gate_out staging_point', fx[6], go['staging_point'] + [-1.5708])
eq('car_3 reset#0 出口侧转点（2026-10-16 北移 0.25，余量要盖住到位残差）',
   fr[0], [6.80, -12.55, 0.0])
eq('car_3 reset#2 回落到 entry#0（周期闭合，不硬跳序列头）', fr[2], fe[0])
eq('car_2 的分航线仍在（条目 4 要演临时车分支时的串行窗口，默认不启用）',
   len(s['fleets']['car_2']['entry_route']), 8)

print('=== 管控与记录（条目 4/5 的起点状态）===')
c = pk['control']
eq('plates 兜底 car_6', c['plates']['car_6'], '沪A12345')
# 条目 4 的"改身份"动作就打在这一张牌上（car_3 是当下唯一在环的摆渡车），所以它的
# 兜底值必须点名 —— /plate/number 是易失话题，重 spawn 后没重贴牌时这里就是最后一道凭据。
eq('plates 兜底 car_3（条目 4 热摘/黑名单的目标牌）', c['plates']['car_3'], '京B88990')
eq('plates 兜底 car_2（串行窗口才上场，牌在白名单外）', c['plates']['car_2'], '粤Z1A2B3')
eq('白名单条数与内容', sorted(c['whitelist'].keys()), sorted(['沪A12345', '京B88990']))
eq('unknown_as_temp（临时车分支的开关）', c['unknown_as_temp'], True)
eq('blacklist 必须是空基线', c['blacklist'], [])
eq('verify_timeout_s', c['verify_timeout_s'], 8.0)
eq('fault_kinds', c['fault_kinds'], ['STUCK', 'RAISE_FAIL', 'SENSOR_FAIL', 'NO_CLOSE'])
r = pk['records']
eq('records.dir / 两份文件名',
   [r['dir'], r['pass_file'], r['action_file']],
   ['/tmp/v2x_parking', 'parking_pass.jsonl', 'barrier_actions.jsonl'])
eq('csv_encoding / max_records_in_memory',
   [r['csv_encoding'], r['max_records_in_memory']], ['utf-8-sig', 5000])

print('=== 3a/3d 那两条 sed 的锚点（取证脚本按行首 4 空格匹配，必须原样在）===')
with open(P, encoding='utf-8') as fh:
    txt = fh.read()
for pat in ['    blacklist: []', '    unknown_as_temp: true']:
    n = txt.count(pat)
    print('  %-40s 出现 %d 次 %s' % (pat, n, '✓' if n == 1 else '!!'))
    if n != 1:
        FAIL.append('sed 锚点 ' + pat.strip())

print()
if FAIL:
    print('CHECK FAIL：%d 项 -> %s' % (len(FAIL), '、'.join(FAIL)))
    sys.exit(1)
print('CHECK PASS：解析成功，且上面每一项都与运行期实测值对上')
