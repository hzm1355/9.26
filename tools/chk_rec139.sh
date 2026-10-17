#!/bin/bash
# 确认识别器只有一个实例在跑。
# 为什么单独一发：ensure_recognizer.sh 报"发布者 2"，两种解释各有后果 ——
#  ① DDS 发现滞后：kill -9 之后发布者计数要 ~21 s 才归零（memory 里踩过），等一等就好；
#  ② 真有第二个实例：同一条 /plate/capture 会被两份进程各答一次，manager 收到重复产出
#     会把一次通行判成两次，账本直接翻倍。
# 判据不能只看 ros2 计数，要数进程本体（方括号防 pgrep 自匹配）。
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/v2x_ws/install/setup.bash 2>/dev/null
unset ROS_DOMAIN_ID

echo "=== 识别器 python 实例（应为 1）==="
pgrep -af 'v2x[_]demo/plate[_]recognizer' | cut -c1-110
echo "  计数: $(pgrep -cf 'v2x[_]demo/plate[_]recognizer')"

echo "=== 等 25 s 让 DDS 发现收敛后重数发布者 ==="
sleep 25
for i in 1 2 3; do
  echo "  /plate/result: $(ros2 topic info /plate/result 2>&1 | tr -d '\r' | grep -a 'Publisher count')"
  echo "  /plate/capture: $(ros2 topic info /plate/capture 2>&1 | tr -d '\r' | grep -a 'Subscriber count')"
  [ "$i" = 1 ] && sleep 12
done

echo "=== 新识别器日志尾（要看到就绪行，门槛 7.0 不打印但崩溃会在这里）==="
tail -6 /tmp/plate_rec.log 2>/dev/null | cut -c1-170
exit 0
