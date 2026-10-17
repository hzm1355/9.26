#!/bin/bash
# 整栈冷启动后的现场体检：录视频前一次判掉"画面有没有帧、停车场链路是不是活的、
# 账本是不是本轮的"。写成文件而不是内联命令：Git Bash 会把 bash -c 里的 $变量 吃掉，
# 实测把 printf 的参数抹成空串，看起来像"话题没有发布者"，是假故障。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=$SIM/tools
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '--- 1) 进程实例数 ---'
for PAT in 'gzserv[e]r' 'gzclien[t]' \
           'lib/v2x[_]demo/parking[_]manager[_]node' \
           'lib/v2x[_]demo/parking[_]recorder[_]node' \
           'lib/v2x[_]demo/parking[_]shuttle[_]node' \
           'lib/v2x[_]demo/plate[_]gen[_]node' \
           'lib/v2x[_]demo/plate[_]recognizer[_]node' \
           'lib/v2x[_]demo/plate[_]recorder[_]node' \
           'lib/v2x[_]demo/speed[_]detect[_]node' \
           'lib/v2x[_]demo/signal[_]drive[_]node' \
           'traffic_light_manage[r]' \
           'python3 -u /tmp/multi_viewserve[r]' \
           'python3 -u /tmp/live_viewserve[r]'; do
  printf '  %-46s %s\n' "$PAT" "$(pgrep -cf "$PAT")"
done

echo '--- 2) :8080 四路回显（503=还没匹配上帧，要等 DDS 发现，别当故障）---'
for N in car1 speed inter monitor; do
  printf '  %-8s ' "$N"
  curl -s -o /dev/null -w 'HTTP %{http_code}  %{size_download} B\n' --max-time 10 \
      "http://localhost:8080/cam/$N"
done

echo '--- 3) 相机话题发布者与帧率 ---'
for T in /car_1/camera/image_raw /speed_cam/speed_cam/image_raw \
         /intersection_cam/intersection_cam/image_raw /monitor_cam/monitor_cam/image_raw \
         /parking/gate_in/image_raw /parking/gate_out/image_raw; do
  printf '  %-46s ' "$T"
  timeout -k 3 15 ros2 topic info "$T" 2>&1 | tr -d '\r' | grep -a 'Publisher count'
done

echo '--- 4) 停车场话题/服务 ---'
for T in /parking/gate_state /parking/pass /parking/actions /parking/shuttle_status \
         /plate/capture /plate/result; do
  printf '  %-26s ' "$T"
  timeout -k 3 15 ros2 topic info "$T" 2>&1 | tr -d '\r' | grep -a 'Publisher count'
done
ros2 service list 2>/dev/null | grep -aE '^/parking/(gate|whitelist|query|export)$' | sed 's/^/  srv /'

echo '--- 5) 本轮账本（应为刚轮转出来的小文件）---'
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl 2>&1 | tr -d '\r'
ls -1 /tmp/v2x_parking_in/*.png /tmp/v2x_parking_out/*.png 2>/dev/null | wc -l | sed 's/^/  抓拍 png 累计 /'
echo '  最新轮转副本：'
ls -1t /tmp/v2x_parking/parking_pass.jsonl.* 2>/dev/null | head -1

echo '--- 6) car_6 与仿真时钟 ---'
gz model -m car_6 -p 2>&1 | head -2 | tr -d '\r' | sed 's/^/  /'
timeout -k 3 15 ros2 topic echo /clock --qos-profile sensor_data --no-daemon --once 2>&1 \
  | tr -d '\r' | grep -a '^sec' | sed 's/^/  /'

echo '--- 7) 车队是否在动（8 s 位移）---'
timeout -k 5 40 python3 $TOOLS/fleet_peek.py 8 2>&1 | tail -9 | tr -d '\r'
echo CHECK_DONE
