#!/bin/bash
# #91 重启 parking manager / recorder（上一条脚本被"pgrep 匹配到自己命令行"
# 的自杀陷阱打断，只杀没起）。
#
# 这次的过滤口径：只看 /proc/PID/exe 是 python3 的进程，且 cmdline 里带 install 路径
# 或 "ros2 run" —— 我的这行 bash -c 命令行里同样写着 parking_manager_node.log，
# 按命令行硬匹就会把自己杀掉。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
CFG=$SIM/src/v2x_demo/config/parking_zones.yaml

pids_of() {
  for P in /proc/[0-9]*; do
    PID=${P#/proc/}
    [ "$PID" = "$$" ] && continue
    EXE=$(readlink -f $P/exe 2>/dev/null)
    case "$EXE" in
      *python3|*python3.10) ;;
      *) continue ;;
    esac
    CMD=$(tr '\0' ' ' < $P/cmdline 2>/dev/null)
    case "$CMD" in
      *parking_$1*) echo "$PID $CMD" ;;
    esac
  done
}

echo "=== 1) 现存的 manager / recorder ==="
pids_of manager | sed 's/^/  /'
pids_of recorder | sed 's/^/  /'

for W in manager recorder; do
  for E in $(pids_of $W | cut -d' ' -f1); do
    echo "  kill -9 $E ($W)"
    kill -9 "$E" 2>/dev/null
  done
done
sleep 3
echo "  清理后：manager=$(pids_of manager | wc -l) recorder=$(pids_of recorder | wc -l)（都要 0）"

echo "=== 2) 启动 ==="
[ -f /tmp/parking_manager_node.log ] && mv /tmp/parking_manager_node.log /tmp/parking_manager_node.log.old91
[ -f /tmp/parking_recorder_node.log ] && mv /tmp/parking_recorder_node.log /tmp/parking_recorder_node.log.old91
nohup bash -lc "ros2 run v2x_demo parking_manager_node.py --ros-args -p use_sim_time:=true -p config_file:=$CFG >> /tmp/parking_manager_node.log 2>&1" >/dev/null 2>&1 &
sleep 8
nohup bash -lc "ros2 run v2x_demo parking_recorder_node.py --ros-args -p use_sim_time:=true -p config_file:=$CFG >> /tmp/parking_recorder_node.log 2>&1" >/dev/null 2>&1 &
sleep 12

echo "=== 3) 校验 ==="
echo "  manager 实例=$(pids_of manager | wc -l)（应为 1）"
echo "  recorder 实例=$(pids_of recorder | wc -l)（应为 1）"
echo "  manager 日志:"
head -15 /tmp/parking_manager_node.log 2>&1 | sed 's/^/    /'
echo "  recorder 日志:"
head -6 /tmp/parking_recorder_node.log 2>&1 | sed 's/^/    /'

echo "=== 4) 等 80 s 看新 manager 还会不会自己搬车 ==="
sleep 80
echo "  这 80 s 里含'复位'的行=$(grep -ac '复位' /tmp/parking_manager_node.log 2>/dev/null)"
grep -a '复位' /tmp/parking_manager_node.log 2>/dev/null | head -5 | sed 's/^/    /'
gz model -m car_6 -p 2>&1 | head -1 | sed 's/^/  car_6: /'
