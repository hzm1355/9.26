#!/bin/bash
# 任务二 条目4 视频取证：事件触发（急救车辆优先放行 / 信号灯强制绿通）
#                + 通过 ROS2 话题输出路口通行效率、等待车辆数量等统计。
# 三种触发方式都演：指定方向强制绿通、急救走廊（纵向绿+横向红+行人禁行）、emergency 服务全场放行。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务二_信号灯配时/条目4_事件触发急救优先与强制绿通_统计输出.mp4"
INTER=$VIEW_INTER

vid_start "$OUT" 300 "$INTER" tl \
  "任务二 条目4  事件触发：强制绿通 / 急救优先走廊 / 全场放行 + 统计话题"
cap "右栏两色块=机动车相位/行人相位；模式列 control_mode 会随事件切成 MANUAL 或 EMERGENCY"
cap "== 0 先摘除自适应算法节点：绿通/放行时长必须只由事件决定，不与系数混叠 =="
adap_stop
sleep 6
cap "== 4a 指定方向强制绿通：/traffic_light/forced_green_group = J3_ns =="
pub /traffic_light/forced_green_group J3_ns
sleep 5
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "  预期：J3_ns 恒 GREEN 且模式 MANUAL；J3_ew 与其余四组仍走各自周期"
sleep 14
rel /traffic_light/forced_green_group none
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail | sed -n '1,7p'"
cap "  取消强制后 J3_ns 回到正常周期"
sleep 6
cap "== 4b 急救车辆优先放行走廊：J1_ns/J2_ns/J3_ns 全绿，横向禁行，行人禁行 =="
pub /traffic_light/emergency_groups '["J1_ns", "J2_ns", "J3_ns"]'
sleep 5
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
# 管理器可能是 launch 拉起的（/tmp/signal.log），也可能是取证脚本单独重启的实例（/tmp/tlmgr_vid.log）
capq "grep -ah '急救优先放行' /tmp/signal.log /tmp/tlmgr_vid.log 2>/dev/null | tail -2 | sed 's/.*\]: //'"
capq "timeout -k 3 20 python3 -u $TOOLS/tl_timeline.py 8 | grep -a 行人 | head -6"
cap "  预期：走廊三组 GREEN、横向三组 RED、行人灯全 DONT_WALK"
sleep 10
rel /traffic_light/emergency_groups none
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail | sed -n '1,7p'"
cap "  走廊关闭，恢复各自周期"
sleep 6
cap "== 4c 全场紧急放行：/traffic_light_manager/emergency 服务 SetBool=true =="
cap "   代码语义：self.emergency 为真时，每一组的相位都直接取 GREEN（不看周期、不看走廊名单）"
capq "timeout -k 3 15 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool '{data: true}' | tail -1"
sleep 12
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail"
cap "  预期：六组 vehicle_phase 全 GREEN、control_mode 全 EMERGENCY、行人灯全 DONT_WALK"
sleep 14
capq "timeout -k 3 15 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool '{data: false}' | tail -1"
sleep 12
capq "timeout -k 3 30 python3 -u $TOOLS/tl_detail.py 6 detail | sed -n '1,7p'"
cap "== 4d 统计话题输出：排队车辆数 / 累计通过 / 平均与最大等待 / 通行效率 =="
capq "timeout -k 3 45 python3 -u $TOOLS/tl_detail.py 25 stats"
cap "  口径：queue_length=当前排队等待车辆数；vehicle_count/total_vehicles 全程不清零，是累计通过量"
vid_wait
adap_start
