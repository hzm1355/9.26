#!/usr/bin/env bash
# 重启 :8090 看板。它一直从 /tmp/live_view_server.py 这份副本运行（当年是为了
# 不改动 D 盘上的原文件才这么摆的），所以改完仓库里的版本必须整份拷过去再重启，
# 否则浏览器看到的还是旧页面。
set -e
D=/mnt/d/desktop1/9.26.1
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

cp $D/live_view_server.py /tmp/live_view_server.py
sed -i 's/\r//g' /tmp/live_view_server.py

# 同名实例要一次杀光：`pgrep | head -1` 只杀一个，留下的旧进程会继续占着 8090 或
# 与新实例同时喂 STATE，页面数据看着正常其实是两份叠加（任务三踩过同款）。
for PID in $(pgrep -f 'python3 -u /tmp/live_view_server.py' || true); do
  # 必须先 TERM 再 KILL：rclpy.init() 给 SIGTERM 装了处理器，只把 context 置成
  # shutdown，而主线程卡在 serve_forever() 里根本不检查它——单发 TERM 杀不掉。
  echo "停掉旧看板 pid $PID"
  kill "$PID" 2>/dev/null || true
  sleep 1
  kill -9 "$PID" 2>/dev/null || true
  for i in 1 2 3 4 5 6 7 8 9 10; do
    ss -tln 2>/dev/null | grep -q ':8090 ' || break
    sleep 1
  done
done
ss -tln 2>/dev/null | grep -q ':8090 ' && { echo '!! 8090 仍被占用，先手工处理'; exit 1; }
setsid nohup python3 -u /tmp/live_view_server.py 8090 > /tmp/live_view.log 2>&1 < /dev/null &
sleep 6
echo "看板实例 $(pgrep -cf 'python3 -u /tmp/live_view_server.py' || true)（必须=1）"
echo "-- state.json 的 speed 字段 --"
curl -s -m 5 http://127.0.0.1:8090/state.json | python3 -c 'import json,sys; s=json.load(sys.stdin).get("speed",{}); print("点位%d 最近行%d 告警%d" % (len(s.get("zones",{})), len(s.get("rows",[])), len(s.get("alerts",[]))))'
echo "-- 首页前 3 行 --"
curl -s -m 5 http://127.0.0.1:8090/ | head -3
echo "-- state.json 的 gates 字段 --"
curl -s -m 5 http://127.0.0.1:8090/state.json | python3 -c 'import json,sys; d=json.load(sys.stdin); print(json.dumps(d.get("gates", {}), ensure_ascii=False)[:400])'
echo "[restart_dashboard] DONE"
