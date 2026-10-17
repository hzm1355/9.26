#!/bin/bash
# 跑 car_3 不动三态探针前的前置闸 + 执行，输出全部 tee 到 /tmp/pk_probe308.log。
# 前置闸只有两道，都是这轮踩过的：
#  1) parking_shuttle_node 必须 0 实例 —— 探针是 /car_3/cmd_vel 的唯一发布者才有意义，
#     shuttle 在跑时两边指令互相盖掉，读数说明不了任何事；
#  2) gzserver 活着且 world 里有 car_3 —— 没这辆车时探针只报"读不到位姿"，
#     那看起来像 DDS 问题，其实是模型没上场。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
LOG=/tmp/pk_probe308.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash

N=$(pgrep -cf "lib/v2x[_]demo/parking_shuttle_node")
echo "shuttle 实例数：${N:-0}（探针要求 0）"
if [ "${N:-0}" != "0" ]; then
  echo "!! 先跑 tools/kill_shuttle308.sh（杀干净并补发零速）再来"
  exit 1
fi

P=$(gz model -m car_3 -p 2>&1 | head -3)
if echo "$P" | grep -aq 'Unable'; then
  echo "!! 运行期 world 里没有 car_3：$(echo "$P" | tr -d '\n' | cut -c1-70)"
  exit 1
fi
echo "car_3 位姿：$(echo "$P" | tr -d '\n' | cut -c1-70)"

cd "$SIM" || exit 1
# 外闸放宽到 240 s：探针本体只要 ~40 s，但前一次 90 s 版把 python 用 SIGTERM 打死了 ——
# rclpy 收到 TERM 会关掉 context，spin 里就抛 ExternalShutdownException，看起来像
# "DDS/上下文坏了"，其实是外层看门狗误杀（这就是那两次的成因）。
T0=$(date +%s)
timeout -k 10 240 python3 tools/pk_probe308.py car_3 2>&1 | tee "$LOG"
RC=$?
echo "退出码 $RC，用时 $(( $(date +%s) - T0 )) s（外闸 240 s；若 RC=124 才是真超时）"
