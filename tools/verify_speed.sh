#!/bin/bash
# 任务四（三）.(四) 测速路段仿真模块 —— 五条要求逐条取证。
#
# 窗口清零是硬要求：记录器的统计口径是"进程启动以来累积"，不清零就会把上一轮的
# 抓拍/告警混进本片的导出与回读，分不清是不是本片内产生的（任务三条目5 踩过）。
# 预检也是硬要求：检测节点活着 ≠ 有抓拍，车队停摆时任何等待窗口都会空转到超时。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=$SIM/tools
REC=/tmp/v2x_records
# 采样窗口默认 420 s：车队被信号灯拖着跑，一圈 ~108 s，单个区间点位实测两圈才过一次
# （200 s 窗口抓到 0 条区间、4 条瞬时），窗口太短会把"没排到"读成"没实现"。
WIN=${WIN:-420}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

step() { echo; echo "======== $1"; }

# 第 5 步会把全部阈值压到 1.0 取证"全告"。任何一步中途失败都不能把现场留在那儿：
# 下一轮读到的会是上一轮的阈值，超速满地跑看着像实现有问题。所以退出即复位。
RESTORED=0
restore_baseline() {
  RESTORED=1
  bash $TOOLS/start_speed_nodes.sh speed_detect_node.py 2>&1 | tail -3
}
trap 'if [ "$RESTORED" != 1 ]; then restore_baseline >/dev/null 2>&1; fi' EXIT

# 刚重启的节点 DDS 发现没完成，`ros2 topic info` 会瞬时读到 0 个发布者（实测
# /speed/segment 就这样报过 0，而它是检测节点启动时就建好的）。判活要等，别把
# "还没发现"读成"没实现"。
pub_count() {
  local C=0 I
  for I in 1 2 3 4 5; do
    C=$(ros2 topic info "$1" 2>/dev/null | tr -d '\r' | awk '/Publisher count/{print $3}')
    if [ "${C:-0}" -gt 0 ]; then break; fi
    sleep 2
  done
  echo "${C:-0}"
}

step "0 预检：检测/记录节点与话题发布者"
for N in speed_detect_node speed_recorder_node; do
  echo "  $N python 实例 $(pgrep -cf "lib/v2x[_]demo/${N}" || echo 0)"
done
bash $TOOLS/ensure_speed_recorder.sh 2>&1 | tail -2
for T in /speed/capture /speed/segment /speed/stats /speed/alert; do
  echo "  $T 发布者 $(pub_count $T)"
done
ros2 service list 2>/dev/null | grep -qx /speed/zones && echo "  服务 /speed/zones 在" || echo "  服务 /speed/zones 不在！"

step "1 车队体检（<3 辆在动会自动复位）"
bash $TOOLS/fleet_guard.sh 2>&1 | tail -4

step "2 记录窗口清零（归档旧 JSONL 后重启记录器）"
mkdir -p $REC/archive
TS=$(date +%Y%m%d_%H%M%S)
for F in speed_captures speed_segments speed_alerts; do
  [ -f $REC/$F.jsonl ] && mv $REC/$F.jsonl $REC/archive/${F}_$TS.jsonl || true
done
FORCE=1 bash $TOOLS/ensure_speed_recorder.sh 2>&1 | tail -2

step "3 条目1：点位清单与位置可灵活配置（/speed/zones 热改）"
python3 $TOOLS/speed_zones_cli.py list
python3 $TOOLS/speed_zones_cli.py add --zone SZ-VIDEOTEST --mode instant \
  --x -11.0 --y 4.6 --limit 2.0 --radar -11.0,2.4 --beam 0 --road 北环东行车道（运行期新增）
python3 $TOOLS/speed_zones_cli.py add --zone SZ-VIDEOTEST2 --mode segment \
  --entry -19.6,-5.0 --exit -19.6,-13.0 --limit 2.0 --road 西环北行车道（运行期新增）
python3 $TOOLS/speed_zones_cli.py remove --zone SZ-VIDEOTEST2

step "4 条目2+3：两种模式并采 ${WIN}s（雷达读数 vs 位姿对照通道 / 阈值判定）"
timeout -k 3 $((WIN + 40)) python3 -u $TOOLS/speed_probe.py $WIN /tmp/speed_verify_probe.txt \
  2>&1 | grep -avE '^\[统计\]' | tail -40

step "5 条目3：热改阈值看告警开关（先抬到全清，再压到全告）"
# 全点位一起改阈值，所以任一点位过一次车就是证据；窗口按"单点位 ~150 s 一遇"给到 160 s。
python3 $TOOLS/speed_zones_cli.py global --limit 99.0
timeout -k 3 200 python3 -u $TOOLS/speed_probe.py 160 /tmp/speed_verify_high.txt 2>&1 \
  | grep -aE '瞬时|区间' | tail -6 || echo "  （160s 内高阈值窗口无过车，稍后用低阈值窗口出告警）"
python3 $TOOLS/speed_zones_cli.py global --limit 1.0
timeout -k 3 200 python3 -u $TOOLS/speed_probe.py 160 /tmp/speed_verify_low.txt 2>&1 \
  | grep -aE '告警|瞬时|区间' | tail -14

step "6 条目4：流量统计（车流量 / 平均车速 / 超速次数）"
timeout -k 3 40 python3 -u $TOOLS/speed_probe.py 25 /dev/null 2>&1 | grep -a '^\[统计\]' | tail -8

step "7 条目5：导出并体检产物"
ros2 service call /speed/export std_srvs/srv/Trigger "{}" 2>&1 | tr -d '\r' | tail -4
python3 $TOOLS/check_speed_export.py $REC

step "8 回到取证基线：阈值按 YAML 原值（只重启检测节点，不动记录器累积）"
restore_baseline
ls -la $REC/speed_*.csv
