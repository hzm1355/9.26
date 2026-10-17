#!/bin/bash
# 查"GUI 到底是谁、连的是哪个 X"。
#
# 为什么要绕这么一圈：刚才 pgrep -f 'gz[_]client' 一行都没有，但 ss 数到 7 条 :6001 的
# tcp 连接，而 PIL 抓 :1 直接 X connection failed。这说明图形客户端确实活着，只是它既不
# 叫 gzclient（名字匹配不到），也不在 :1 上（我猜的显示号是错的）。按 memory「证据命令
# 别用猜的过滤式」，一律先落盘再取，并且用"谁持有 :6001 连接"这种客观口径来认进程。
set +e
LOG=/tmp/probe_x145.txt
: > "$LOG"

echo "--- 1) 全景进程（凡带 gazebo/gz 的都列，不猜名字）---" | tee -a "$LOG"
ps -eo pid,ppid,etime,rss,args | grep -aiE 'gazebo|gzclient|gzserver|gz gui' \
  | grep -av 'grep -aiE' | cut -c1-160 | tee -a "$LOG"
echo "  （空表=没有名字里带 gazebo 的进程，见第 3 步的连接归属）" | tee -a "$LOG"

echo "--- 2) 各自的 DISPLAY / GAZEBO_MASTER_URI ---" | tee -a "$LOG"
for P in $(ps -eo pid,args | grep -aiE 'gazebo|gzclient|gzserver' | grep -av grep | awk '{print $1}'); do
  D=$(tr '\0' '\n' < /proc/$P/environ 2>/dev/null | grep -aE '^(DISPLAY|WAYLAND_DISPLAY|GAZEBO_MASTER_URI)=' | tr '\n' ' ')
  echo "  pid $P：$D" | tee -a "$LOG"
done

echo "--- 3) 谁持有到 VcXsrv(:6001) 的连接 ---" | tee -a "$LOG"
ss -tnp 2>/dev/null | grep -a ':6001' | cut -c1-200 | tee -a "$LOG"
echo "  合计：$(ss -tnp 2>/dev/null | grep -ac ':6001') 条" | tee -a "$LOG"

echo "--- 4) 本机 X 显示号候选 ---" | tee -a "$LOG"
ls /tmp/.X11-unix/ 2>/dev/null | tr '\n' ' ' | sed 's/^/  unix socket：X/' | tee -a "$LOG"
ss -tln 2>/dev/null | grep -aE ':60[0-9][0-9]' | tee -a "$LOG"
