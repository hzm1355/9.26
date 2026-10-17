#!/bin/bash
# ① 把改过的识别器同步到运行期副本并三处 md5 复核；
# ② 给 car_6 贴**前牌**（gate_in 相机拍到的是车头，plate_front 至今还是占位灰牌）；
# ③ 列出闸机相机的真实话题名，供下一步抓活画面用。
# 不重启 gzserver、不重启 manager/shuttle —— 这两件事都只是运行期可改的。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
WS=~/v2x_ws/src/v2x_demo/v2x_demo
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

F=plate_recognizer_node.py
cp -f "$SIM/src/v2x_demo/v2x_demo/$F" "$WS/$F"
echo "--- md5（三份应一致：D 盘源 / home src / install 侧解析到的真身）---"
md5sum "$SIM/src/v2x_demo/v2x_demo/$F" "$WS/$F" \
  ~/v2x_ws/install/v2x_demo/lib/v2x_demo/$F 2>&1

echo "--- 运行期识别器里门槛的实际值 ---"
grep -n 'PLATE_RATIO_MAX = ' ~/v2x_ws/install/v2x_demo/lib/v2x_demo/$F

echo "--- 闸机相机话题 ---"
ros2 topic list 2>/dev/null | grep -aiE 'gate_camera|/gate_in|/gate_out|plate/' | sed 's/^/  /'

echo "--- 给 car_6 贴前牌（当前 /plate/set 与 /plate/number 的尾部）---"
timeout -k 3 12 ros2 topic pub --once /plate/request std_msgs/msg/String \
  "{data: 'car_6|front|沪A12345'}" >/dev/null 2>&1
echo "  发布返回码 $?"
sleep 3
echo "  /plate/set   : $(timeout -k 3 12 ros2 topic echo /plate/set --once --no-daemon 2>&1 | tr -d '\r' | head -3 | tr '\n' ' ')"
echo "  /plate/number: $(timeout -k 3 12 ros2 topic echo /plate/number --once --no-daemon 2>&1 | tr -d '\r' | head -3 | tr '\n' ' ')"
echo "--- 贴牌插件日志尾（要找 car_6 那一行，不是别的车）---"
tail -6 /tmp/plate_gen.log /tmp/*plate*.log 2>/dev/null | cut -c1-170 | sed 's/^/  /'
exit 0
