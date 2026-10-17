#!/bin/bash
# 把"抓拍到了但识别无结果"这一环的现场捞全：图、识别器真实日志、它挂的是哪台相机。
#
# 为什么不能只看 manager 那行 WARN 就下结论：它只说 algorithm=builtin_opencv 没结果，
# 到底是"图里根本没牌"、"有牌但形状门槛按高速卡口的安装几何卡的（#90）"、还是
# "识别器压根没订到 /parking/gate_in/plate_event"，三种病因的修法完全不同。
# 上一版这里 tail 了 /tmp/plate_recognizer_node.log，输出空 —— 那个文件名是我猜的，
# 真实日志路径要从进程的 fd/1 读，不能靠猜。
set +e
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
D=/mnt/d/desktop1/9.26.1/_live142

echo "== 1) 识别器进程：命令行与真实日志路径 =="
for PID in $(pgrep -f "lib/v2x[_]demo/plate_recognizer_node"); do
  echo "  pid $PID"
  echo -n "  cmdline: "; tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null; echo
  echo -n "  cwd:     "; readlink /proc/$PID/cwd 2>/dev/null; echo
  echo -n "  stdout:  "; readlink /proc/$PID/fd/1 2>/dev/null; echo
done

echo "== 2) 入口闸抓拍目录最新 6 张 =="
ls -la --time-style=+%m-%d_%H:%M /tmp/v2x_parking_in/ 2>/dev/null | tail -6 | sed 's/^/  /'

echo "== 3) 拷 gate_in-14 到 D 盘看 =="
for F in gate_in-14.png gate_in-13.png; do
  if [ -f /tmp/v2x_parking_in/$F ]; then
    cp /tmp/v2x_parking_in/$F $D/$F && echo "  已拷 $F ($(stat -c %s $D/$F) 字节)"
  else
    echo "  $F 不存在"
  fi
done

echo "== 4) 识别器最近读过到牌没有（按它自己的措辞找，不猜关键词）=="
LOG=$(readlink /proc/$(pgrep -f "lib/v2x[_]demo/plate_recognizer_node" | head -1)/fd/1 2>/dev/null)
if [ -n "$LOG" ] && [ -f "$LOG" ]; then
  echo "  日志文件: $LOG，行数 $(wc -l < "$LOG")"
  tail -14 "$LOG" | cut -c1-175 | sed 's/^/  | /'
else
  echo "  !! 拿不到日志文件（可能是 stdout 直接进了 launch 的聚合日志）"
  ls -la /tmp/*.log 2>/dev/null | grep -aiE ' recog|park' | sed 's/^/  /'
fi

echo "== 5) 入口闸相机的话题与图像是否在流 =="
timeout -k 3 15 ros2 topic info /parking/gate_in/image_raw 2>&1 | tr -d '\r' | sed 's/^/  /'
timeout -k 3 15 ros2 topic bw /parking/gate_in/image_raw 2>&1 | tr -d '\r' | head -3 | sed 's/^/  /'
echo DONE
