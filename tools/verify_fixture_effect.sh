#!/bin/bash
# 等效盒上线后的效果复核：采 MIN 分钟，然后看三件事
#   1) 车队还在正常跑（RTF、每车单发布者、速度范围）
#   2) 横偏与"轨迹外接盒与实体碰撞盒相交"——/tmp/world_collisions.json 现在含
#      sandbox_fixtures_coll 的 96 个新盒，相交即真撞
#   3) 驱动节点的复位/脱困/钉停计数（新碰撞最容易引发的副作用就是楔住）
# 用法：bash tools/verify_fixture_effect.sh [分钟]
set -e
WS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
MIN=${1:-15}
cd "$WS"
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

echo "== 采样 ${MIN} 分钟 =="
sleep $((MIN * 60))

echo "== 进程 =="
ps -eo pid,etime,rss,comm | grep -wE 'gzserver|gzclient' || true
echo "  驱动节点实例=$(pgrep -cf 'lib/v2x[_]demo/signal[_]drive[_]node' || true)"

echo "== probe_fleet（RTF / 每车 cmd 帧数 / 速度范围）=="
timeout 60 python3 tools/probe_fleet.py 2>&1 | tail -22 || true

echo "== 驱动节点日志护栏计数 =="
for F in /tmp/signal.log /tmp/signal_drive_fix.log; do
  [ -f "$F" ] || continue
  echo "  --- $F ---"
  for P in 已复位 倒车脱困 打转钉停 重取航段 拦停; do
    printf '    %-10s %s\n' "$P" "$(grep -ac -- "$P" "$F" 2>/dev/null || true)"
  done
done

echo "== 轨迹样本 =="
wc -l /tmp/signal_drive_path.csv
tail -1 /tmp/signal_drive_path.csv

echo "== 偏差与碰撞审计 =="
# 审计脚本读的是 /tmp/world_collisions.json 这份静态快照，它由 world 的 XML 解析而来。
# 等效盒是新写进 world 的，快照不重生成就会拿"补盒之前"的占据去判相交，96 个新盒全被
# 当成空气，"相交即真撞"这条判据直接失效。必须在审计前重跑一次清点。
timeout -k 5 60 python3 -u tools/world_collision_inventory.py > /tmp/world_coll_dump.txt 2>&1 || true
grep -a 'sandbox_fixtures_coll' /tmp/world_coll_dump.txt | head -3 || true
grep -ac 'sandbox_fixtures_coll' /tmp/world_coll_dump.txt || true
python3 tools/analyze_deviation.py 1.0 > /tmp/fixture_audit.txt 2>&1 || true
tail -75 /tmp/fixture_audit.txt
cp /tmp/fixture_audit.txt "$WS/../_fixture_audit.txt" 2>/dev/null || true
cp /tmp/signal_drive_path.csv /tmp/signal_drive_path.with_fixtures.csv || true
echo DONE
