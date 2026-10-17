#!/bin/bash
# 摊开所有和 Gazebo 图形界面有关的进程：进程名（comm）和完整命令行一起看。
# 为什么不用 pgrep -x gzclient：#142 实测旧客户端杀不掉，说明它的 comm 不是 "gzclient"
# （Gazebo 11 的 GUI 可能以 gzclient-11 / gz gui / Qt 应用名注册），只按精确名匹配就漏了。
set +e
echo "== comm 里带 gz / gazebo 的进程 =="
ps -eo pid,ppid,etime,rss,comm,args 2>/dev/null | tr -d '\r' \
  | grep -aiE 'gzclient|gzserver|gazebo|gz gui' | grep -av grep | cut -c1-190
echo "== 端口 6001（X）连接归属 =="
ss -tnp 2>/dev/null | tr -d '\r' | grep -a ':6001' | cut -c1-150 | head -20
