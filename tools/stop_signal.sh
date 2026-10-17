#!/bin/bash
# 只停止信号灯子系统节点（不重启 Gazebo）
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
# 顺序很关键：驱动节点在以 10 Hz 持续发 cmd_vel，先刹后杀等于白刹——
# 杀掉之前它又覆盖了几十条巡航速度，diff_drive 插件缓存最后一条指令继续把车开出去。
#
# 清理模式必须逐具名节点列，不能按包目录整片 pkill。整片式 '/install/v2x_demo/lib/v2x_demo/'
# 会把同目录里的 ETC 三节点、车牌三件套、停车场三节点一起带走，而且 pkill 默认发 SIGTERM，
# rclpy 把 TERM 只当 shutdown 标志，节点会留下 ExternalShutdownException 的 traceback 后"体面"退出，
# 事后读日志极易误判成回调异常（10-15 17:03 的 etc_recorder 就是这么死的）。
# 名字取自 launch/traffic_signal_full.launch.py 的 executable，改了那边要同步改这里。
for N in 'signal[_]drive_node.py' 'traffic[_]light[_]manager.py' 'adaptive[_]timing_node.py' \
         'traffic[_]light[_]recorder.py' 'websocket[_]bridge.py'; do
  pkill -f "/install/v2x_demo/lib/v2x_demo/$N" 2>/dev/null || true
done
pkill -f 'traffic_signal_full.launch' 2>/dev/null || true
sleep 2
# 车队名单要和 signal_drive 实际驱动的车型对上（car_2/car_3 已 drive:false 归摆渡车，
# 但 kill 掉发布者后同样要补零速，否则 diff_drive 缓存的最后一条指令继续推车）。
for c in car_1 car_2 car_3 car_4 car_5; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1
done
sleep 1
echo "remaining v2x_demo nodes: $(pgrep -cf 'install/v2x[_]demo/lib' || true)"
