#!/bin/bash
# 只读取证：南口那条"已删但要重启才生效"的隐形墙，是不是车队整体贴到 y≈-18.95 的成因。
#
# 为什么要这三条：T4 静态闸里车流四台被摆回 y=-16.8 的车道后，**驱动节点还没起**就又被
# 挤到 y≈-18.95（pk201 摆前读数 car_1 (-0.34,-18.97)、car_2 (2.43,-18.95) 同一条带）。
# 而 signal_drive_node 的 obstacle_file 缺省是空串（源码 244 行），也就是说 lat_target 恒 0、
# 控制器一直想把车拉回 -16.8 —— 拉不回去还横偏 2.15 m，就正好踩 line 618 的
# off_lane=|lat-lat_target|>lane_guard，连续几次回不来就复位，复位又带 4~8 m/s 弹射。
# 所以先分清两件事：(a) 世界文件里那堵墙到底删没删；(b) 跑着的 gzserver 是哪一刻起的、
# 加载的是哪一份文件（memory「先确认运行期加载的是哪份文件副本」）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
unset ROS_DOMAIN_ID

echo '=== 1) 世界文件里 c130 那一类"南口复位格隐形墙"还在不在（文件=重启后才会生效的那份）'
echo "  文件 mtime：$(stat -c %y "$W" 2>/dev/null | cut -c1-19)　字节 $(stat -c %s "$W" 2>/dev/null)"
echo "  含 c130 的行数：$(grep -ac c130 "$W" 2>/dev/null; true)"
grep -an 'c130' "$W" 2>/dev/null | head -6 | cut -c1-140 | sed 's/^/    /'
echo "  南口一带（y<-16 且 |x|<25）的 collision 名清单，按文件顺序："
grep -aoE '<collision name="c1[0-9][0-9]"' "$W" 2>/dev/null | sort -u | sed 's/^/    /'

echo '=== 2) 跑着的 gzserver：起来多久、cwd 与命令行（决定它加载的是哪一份 world）'
GZPID=$(pgrep -x gzserver | head -1)
echo "  pid=${GZPID:-无}  etime=$(ps -o etime= -p "${GZPID:-0}" 2>/dev/null | tr -d ' ')"
echo "  起量时刻（进程开始时间）：$(ps -o lstart= -p "${GZPID:-0}" 2>/dev/null)"
echo "  cwd=$(sudo readlink /proc/${GZPID:-0}/cwd 2>/dev/null || readlink /proc/${GZPID:-0}/cwd 2>/dev/null)"
echo "  cmdline=$(tr '\0' ' ' < /proc/${GZPID:-0}/cmdline 2>/dev/null | cut -c1-110)"

echo '=== 3) 慢性还是新发：历次驱动日志里「复位/离道/出界」的行数（行数是累计，不看时刻）'
for L in /tmp/signal_drive*.log /tmp/sd*.log; do
  [ -f "$L" ] || continue
  echo "  $(basename "$L") 行数=$(wc -l < "$L") mtime=$(stat -c %y "$L" | cut -c1-19)"
  echo "      复位=$(grep -ac 复位 "$L"; true) 出界=$(grep -ac 出界 "$L"; true) 离道=$(grep -ac 离道 "$L"; true) 重取航段=$(grep -ac 重取航段 "$L"; true)"
done

echo '=== 4) 归档轨迹里"车队到底贴在哪一条 y"（CSV 是驱动节点每 tick 写的，含复位瞬移）'
timeout -k 5 120 python3 "$SIM/tools/pk211_bandscan320.py" \
  /tmp/signal_drive_path.pre_homeall.csv /tmp/signal_drive_path.csv 2>&1 | sed 's/^/  /'
echo 'PK210DONE'
