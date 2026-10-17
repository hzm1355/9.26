#!/bin/bash
# 等条目 5 自然收尾，然后一次跑完成片体检 + 对账结果 + 现场收尾体检。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=$SIM/tools
MP4='/mnt/d/desktop1/9.26.1/视频取证/任务五_停车场闸机/条目5_记录管理_进出场车牌动作日志查询导出.mp4'
LOG=/mnt/d/desktop1/9.26.1/_diag155/vid5_run1.log
PKCLI="python3 $TOOLS/parking_cli.py"
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

for K in $(seq 1 30); do
  sleep 20
  printf '[%04ds] 录制进程 %s  字幕 %s 行\n' "$((K * 20))" \
    "$(pgrep -cf 'vid_parking_[5].sh')" "$(wc -l < /tmp/vid_caption.txt 2>/dev/null)"
  [ "$(pgrep -cf 'vid_parking_[5].sh')" = "0" ] && break
done

echo '=== 1) 成片体检 ==='
python3 $TOOLS/vid_check.py "$MP4" 2>&1 | tr -d '\r' | tail -18
echo '=== 2) 运行日志 !! 计数与尾巴 ==='
grep -ac '!!' $LOG
tail -8 $LOG | tr -d '\r' | cut -c1-170
echo '=== 3) A~H 对账器全文要点（退出码 / 通过数 / FAIL 行）==='
grep -aE '退出码|通过判据' /tmp/vid_caption.txt | tr -d '\r' | cut -c1-170 | head -6
grep -a '✓' /tmp/pk5_audit.txt 2>/dev/null | head -16
grep -aE '✗|汇总|FAIL' /tmp/pk5_audit.txt 2>/dev/null | head -8
echo '=== 4) 导出落盘（CSV 行数 vs 回包条数 vs JSONL 全量）==='
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl 2>&1
wc -l /tmp/v2x_parking/parking_pass.csv /tmp/v2x_parking/barrier_actions.csv 2>&1
echo '=== 5) 收尾现场 ==='
$PKCLI gate list 2>&1 | tr -d '\r' | head -4
for N in parking_manager_node parking_recorder_node parking_shuttle_node; do
  printf '  %-24s 实例 %s\n' "$N" "$(pgrep -cf lib/v2x[_]demo/$N)"
done
printf '  car_6 = '; gz model -m car_6 -p 2>&1 | tr -d '\r'
echo WAIT5_DONE
