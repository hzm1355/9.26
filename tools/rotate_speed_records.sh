#!/bin/bash
# 把测速三份 JSONL 轮转走，并强制重启 speed_recorder_node，让"落盘全量 = 本轮会话"。
#
# 为什么必须做：整栈重启（restart_clean_stack.sh）后记录器的**内存累积归零**，而
# speed_recorder_node.py:77 打开 JSONL 用的是 'a' 追加模式 —— 磁盘上留着上一会话的
# 2400+ 行，/speed/export 导出的 CSV 却是 'w' 写内存会话（同文件 208 行）。
# 于是"看板累计 / CSV 行数 / JSONL 行数"三个口径会凭空差出一大截，条目 4 的流量统计
# 和条目 5 的"导出原始数据"没法对账闭合（memory「统计数字要点名口径并对账闭合」）。
#
# 顺序不能反：必须先杀记录器再 mv —— 记录器持有的是文件描述符，先 mv 它会继续往
# 改过名的文件里写，新文件反而一行都没有。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
D=/tmp/v2x_records
BK=${D}_rot_$(date +%m%d_%H%M%S)
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '[rotate] 1) 杀掉 speed_recorder 全部实例（TERM 后补 KILL）'
for PID in $(pgrep -f 'lib/v2x[_]demo/speed_recorder_node' || true) \
           $(pgrep -f 'run v2x[_]demo speed[_]recorder[_]node' || true); do
  kill "$PID" 2>/dev/null || true
done
sleep 2
for PID in $(pgrep -f 'lib/v2x[_]demo/speed_recorder_node' || true); do
  kill -9 "$PID" 2>/dev/null || true
done
sleep 1
N=$(pgrep -cf 'lib/v2x[_]demo/speed_recorder_node' || true)
echo "  残留实例：${N:-0}"
[ "${N:-0}" = "0" ] || { echo '!! 没杀干净，停跑（mv 会写进改名的旧文件）'; exit 1; }

echo '[rotate] 2) 三份 JSONL 归档到新目录'
mkdir -p "$BK"
for f in speed_captures speed_segments speed_alerts speed_stats; do
  if [ -f $D/$f.jsonl ]; then
    echo "  $f.jsonl  $(wc -l < $D/$f.jsonl) 行 -> $BK/"
    mv $D/$f.jsonl $BK/
  elif [ -f $D/$f.csv ]; then
    echo "  $f.csv（旧导出）留在原地"
  fi
done
ls $D/*.jsonl 2>/dev/null | sed 's/^/  仍存在：/' || echo "  目录下已无 jsonl"

echo '[rotate] 3) 强制重启记录器'
FORCE=1 bash $SIM/tools/ensure_speed_recorder.sh | tail -3
sleep 12
M=$(pgrep -cf 'lib/v2x[_]demo/speed_recorder_node' || true)
echo "  记录器实例：${M:-0}"
[ "${M:-0}" = "1" ] || { echo '!! 记录器实例数不是 1'; exit 1; }

echo '[rotate] 4) 效果复核：新 JSONL 要从 0 行开始长起来（回读文件而不是看服务回执）'
for i in 1 2 3 4 5 6; do
  sleep 20
  C=$(wc -l < $D/speed_captures.jsonl 2>/dev/null || echo 0)
  G=$(wc -l < $D/speed_segments.jsonl 2>/dev/null || echo 0)
  A=$(wc -l < $D/speed_alerts.jsonl 2>/dev/null || echo 0)
  echo "  +$(( i * 20 )) s  抓拍 $C 行 / 区间 $G 行 / 告警 $A 行"
  [ "$C" -gt 0 ] && break
done
[ "$C" -gt 0 ] || { echo '!! 6 分钟没有新抓拍，车队可能没经过点位，先别录'; exit 1; }
echo '[rotate] 完成，落盘口径已与本轮会话对齐'
