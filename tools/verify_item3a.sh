#!/bin/bash
# 任务二 条目3a 复验：四套预设方案（default/peak/offpeak/night）逐一套切换并读回生效配时。
# 与 verify_presets.sh 的唯一区别：预设指令改走 tools/tl_pub.py（先等订阅者、再连发三次）。
#   上一轮 `ros2 topic pub --once /traffic_light/preset ... peak` 的那一条被 DDS 发现竞态吃掉
#   （管理器日志里没有 'preset -> peak'），导致 peak 读回的还是 default 的 10/3/10。
# 全程 enable_adaptive:=false：读到的是"预设方案本身的时长"，不与自适应系数混叠。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

echo "########## 以「关闭自适应」方式重启信号栈 ##########"
bash $TOOLS/stop_signal.sh > /dev/null 2>&1
setsid nohup ros2 launch v2x_demo traffic_signal_full.launch.py \
  enable_adaptive:=false enable_recorder:=false > /tmp/signal_np.log 2>&1 &
sleep 14
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
grep -E 'preset=default' /tmp/signal_np.log | head -1 | sed 's/^/  /'

echo
echo "########## 四套预设切换：每套下发后等一个完整周期再读 ##########"
for p in default peak offpeak night; do
  timeout -k 3 20 python3 -u $TOOLS/tl_pub.py /traffic_light/preset "$p" 3
  sleep 26
  echo "  --- preset=$p ---"
  timeout -k 3 25 python3 -u $TOOLS/tl_detail.py 5 detail | sed -n '1,7p' | sed 's/^  /    /'
done

echo
echo "########## 管理器侧预设切换日志（每条都该有一行 'preset -> xxx'） ##########"
grep -aE 'preset ->|时段自动切换|手动预设切换' /tmp/signal_np.log | sed 's/.*\]: //; s/^/  /'

echo
echo "########## 恢复带自适应的标准信号栈 ##########"
bash $TOOLS/stop_signal.sh > /dev/null 2>&1
bash $TOOLS/start_signal.sh > /dev/null 2>&1
sleep 8
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
