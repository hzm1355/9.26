#!/bin/bash
# 上一轮 triage 里 pgrep -x gzserver 与 gz model 都空回，但 ros2 service list 能看到
# /get_model_list —— 两者矛盾，必须先把"gzserver 到底活在哪、ROS 插件通不通"钉死，
# 否则后面所有"复位 car_6"的判断都建在沙上。纯只读。
set +e
echo '== 所有名字含 gz 的进程（字符类防自匹配，不用 [_] 后缀式）=='
pgrep -af 'g[z]' | cut -c1-140
echo '== 所有 python/ros 相关常驻 =='
pgrep -af 'ros2|launch|_node' | cut -c1-140 | head -20
echo '== /usr/bin/gzserver 的命令行（从 /proc 找，不看名字）=='
for P in $(ls /proc | grep -E '^[0-9]+$'); do
  EXE=$(readlink -f /proc/$P/exe 2>/dev/null)
  case "$EXE" in
    *gzserver*|*gzclient*) echo "  pid $P exe $EXE"; tr '\0' ' ' < /proc/$P/cmdline | cut -c1-160; echo;;
  esac
done
echo '== 端口/共享内存层面：gz 的 transport socket =='
ls -la /tmp/gz* 2>/dev/null | head
echo DONE
