#!/usr/bin/env bash
# 全停仿真栈（bringup gzserver / Nav2 / slam / rviz）。
# 用 [x] 字符类正则避免 pkill -f 匹配到本脚本自身命令行。
pkill -f '[g]zserver' 2>/dev/null
pkill -f '[g]zmaster' 2>/dev/null
pkill -f '[s]lam_toolbox' 2>/dev/null
pkill -f '[b]t_navigator' 2>/dev/null
pkill -f '[c]ontroller_server' 2>/dev/null
pkill -f '[p]lanner_server' 2>/dev/null
pkill -f '[b]ehavior_server' 2>/dev/null
pkill -f '[m]ap_server' 2>/dev/null
pkill -f '[a]mcl' 2>/dev/null
pkill -f '[l]ifecycle_manager' 2>/dev/null
pkill -f '[v]elocity_smoother' 2>/dev/null
pkill -f '[r]viz2' 2>/dev/null
pkill -f '[t]raffic_light_' 2>/dev/null
pkill -f '[r]egion_trigger' 2>/dev/null
pkill -f 'ros2 launch [v]2x_demo' 2>/dev/null
sleep 2
echo '[stop] 残留进程检查：'
pgrep -a -f 'gzserver|gzmaster|rviz2|slam_toolbox|bt_navigator' | head -5 || echo '[stop] 全部已停'
