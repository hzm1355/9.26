#!/usr/bin/env python3
# 读 tools/pk_run308.sh 留下的三态探针日志，判"car_3 到底有没有出力"，
# 据此决定要不要走重 spawn（/delete_entity 是高危动作，只在实测零出力时才做）。
#
# 口径（全部点名，别拿"末态"糊过去）：
#   * 只看**主动给激励**的三段（spin / throttle / spin_back）的**峰值**，不看位移末态；
#     memory「生效要用效果复核」：静止复检不算证据；
#   * 响应阈：轮关节点峰值 >= 0.15 rad/s 或 实测峰值线速 >= 0.10 m/s 或 位移 >= 0.30 m。
#     三个量分别是"轮子真转""车身真动""累计位移"，任一到就算出力；
#   * 探针必须跑到 PROBE308DONE，否则是探针自己崩了/被外层看门狗打断，
#     这种读数不能拿来判死一辆车（memory「rclpy 探针执行器三坑」：外层 timeout
#     会把 spin 打断成 ExternalShutdownException，看着像 DDS 坏了）。
import re
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pk_probe308.log'
# 符号：位移/Δz/峰值都可能带 +（"%+.3f" 打出来的正号），早先只写 [-\d.] 会把
# "+0.000" 那几行整行漏掉 —— 结果 3 段只解析到 1 段，被判成"探针没跑完"退出码 3。
ROW = re.compile(r'\[(\w+)\] 指令 v=([-+\d.]+) ω=([-+\d.]+).*?位移 ([-+\d.]+) m、'
                 r'Δz ([-+\d.]+) m、Δyaw ([-+\d.]+)°、实测峰值线速 ([-+\d.]+) m/s、'
                 r'轮关节点峰值 ([-+\d.]+) rad/s')

try:
    TXT = open(PATH, encoding='utf-8').read()
except OSError as exc:
    print('DECIDE=ABORT 读不到探针日志 %s：%s' % (PATH, exc))
    sys.exit(3)

if 'PROBE308DONE' not in TXT:
    print('DECIDE=ABORT 探针没跑完（日志里没有 PROBE308DONE），这份读数不作数')
    sys.exit(3)

# 本轮实测：探针打完 PROBE308DONE 之后在 rclpy 收尾处抛
# "terminate called without an active exception" 并 dumped core。数据是在那之前打的，
# 所以不影响判据，但必须显式说出来，不然下次会让人以为整轮作废。
if 'dumped core' in TXT or 'terminate called' in TXT:
    print('  注：探针收尾阶段 dump 了 core（在 PROBE308DONE 之后），读数仍按已打印的行算')

rows = [m.groups() for m in ROW.finditer(TXT)]
if len(rows) < 3:
    print('DECIDE=ABORT 只解析到 %d 段激励行（应为 3），日志格式和探针脚本对不上'
          % len(rows))
    sys.exit(3)

responded = []
for tag, v, w, d, dz, dyaw, peak_v, peak_w in rows:
    print('  [%s] v=%s ω=%s -> 位移 %s m、Δz %s m（台面 z≈6.70，掉出沙盘会露出来）、'
          'Δyaw %s°、峰值线速 %s m/s、轮关节点峰值 %s rad/s'
          % (tag, v, w, d, dz, dyaw, peak_v, peak_w))
    if float(peak_w) >= 0.15 or float(peak_v) >= 0.10 or float(d) >= 0.30:
        responded.append(tag)

if responded:
    print('DECIDE=RESPONDED 出力的段：%s —— 插件没卡死，病灶是顶住/几何，不重 spawn'
          % ','.join(responded))
    sys.exit(0)
print('DECIDE=WEDGED 三段激励全零（轮关节点峰值≈0、峰值线速≈0、位移≈0）'
      '—— 这是 memory「gz-diffdrive-instance-wedge」的第二形态"给什么都不转"，'
      '只有重 spawn 该模型能复位')
sys.exit(2)
