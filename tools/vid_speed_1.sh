#!/bin/bash
# 任务四 条目1 视频取证：在虚拟路网中设置测速区间点位，位置可灵活配置。
#
# 这一段为什么这么排：
#   · "可灵活配置"要演成三件事才算数——能新增（OP_ADD）、能改位置（同 ID 再 ADD 就地覆盖）、
#     能删除/回到基线（OP_REMOVE/OP_CLEAR），而且全程不重启检测节点（PID 与运行时长不变）；
#   · 每步之后都等到下一轮 /speed/stats（10 s 一轮）才继续，这样面板上"在册点位"计数的
#     变化与字幕里的 list 回包落在同一帧上，看画面就能对上"哪一步产生了什么"；
#   · 点位坐标一律取自 speed_zones.yaml 注释里的真实车道（Ring A 北环 y=4.6、南环 y=-14.6、
#     西环 x=-19.6），新增点位必须压在车道上，否则零事件会被读成"配置没生效"。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务四_测速路段/条目1_测速点位灵活配置_新增改位删除热生效.mp4"
CLI="python3 $TOOLS/speed_zones_cli.py"

vid_start "$OUT" 290 "$VIEW_MON,$VIEW_INTER" speed \
  "任务四 条目1  测速区间点位灵活配置（世界帧坐标 + /speed/zones 服务热改）"
cap "本段全程不重启检测节点：新增/改位/删除/回基线都走 /speed/zones 的 6 种 op"
capq "pgrep -af 'speed_detect_node.py --ros' | tail -1 | cut -c1-96; echo '^ 检测节点进程（结尾会再读一次，PID 不变即未重启）'"
sleep 4
cap "== 1a 先把现场清回 YAML 基线（清掉上一轮残留的运行期点位与统计高水位），再读基线 =="
capq "$CLI clear | head -2"
sleep 4
capq "$CLI list"
sleep 16
cap "  右栏'在册点位 5'、统计表 5 行 = 基线点位；阈值 2.0/4.0/2.0/2.0/3.0 km/h"
cap "== 1b OP_ADD 新增定点瞬时点位：北环东行车道 y=4.6 的 x=-11.0，雷达杆在路侧 y=2.6 正对来车 =="
capq "$CLI add --zone SZ-VID-inst --mode instant --x -11.0 --y 4.6 --limit 2.0 --radar -11.0,2.6 --beam 0.0 --road 北环东行车道-录制期新增"
sleep 16
cap "  面板'在册点位 6'，新行阈值 2.0、累计从 0 起（点位变更会重置该轮统计）"
cap "== 1c OP_ADD 新增区间点位：南环西行车道 y=-14.6，进入断面 x=-1.0 → 离开断面 x=-11.0（长 10 m）=="
capq "$CLI add --zone SZ-VID-seg --mode segment --entry -1.0,-14.6 --exit -11.0,-14.6 --limit 3.0 --radius 1.6 --road 南环西行车道-录制期新增"
sleep 16
cap "  面板'在册点位 7'；区间行末列'待N' = 已进入断面、尚未离开断面的在途车辆数"
cap "== 1d 回读：list 打印的是服务端当前全部点位（含坐标），不是本地 YAML 文件 =="
capq "$CLI list | tail -9"
sleep 6
cap "== 1e 位置可灵活配置：同 ID 再 ADD 一次就是就地覆盖 —— 把这个区间的两个断面整体挪到西环北行车道 x=-19.6 =="
capq "$CLI add --zone SZ-VID-seg --mode segment --entry -19.6,0.0 --exit -19.6,-10.0 --limit 3.0 --radius 1.6 --road 西环北行车道-改位后"
sleep 4
capq "$CLI list | grep -a VID"
sleep 16
cap "  同一 ID 的断面坐标已从 (-1.0,-14.6)/(-11.0,-14.6) 变成 (-19.6,0.0)/(-19.6,-10.0)"
cap "== 1f OP_REMOVE 删除瞬时点位 =="
capq "$CLI remove --zone SZ-VID-inst"
sleep 16
cap "  面板'在册点位 6'：删掉的点位不再挂在统计表上（不是留一行零值）"
cap "== 1g OP_CLEAR 清掉 YAML 之外的全部运行期点位，回到 5 点基线 =="
capq "$CLI clear"
sleep 16
cap "== 1h 界面侧复核：:8090 看板'测速路段'面板的点位表 =="
capq "python3 $TOOLS/dashboard_speed_dump.py | sed -n '1,8p'"
sleep 6
cap "  看板与 /speed/zones 同源：只剩 5 个在册点位，运行期新增的 SZ-VID-* 不再残留"
cap "== 1i 收尾：再读一次检测节点进程 =="
capq "pgrep -af 'speed_detect_node.py --ros' | tail -1 | cut -c1-96"
cap "  与开头同一 PID、运行时长连续增长 → 以上全部是热改，未重启节点"
vid_wait
echo DONE
