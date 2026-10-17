#!/bin/bash
# apply 第 9 步 colcon build 失败的收尾：我改的是 D 盘 CMakeLists，而 colcon 编译的是
# ~/v2x_ws/src/v2x_demo 那一份独立副本（两边只差行尾符，内容一致）。
# 这一步把同一规则落到 home 副本上、清掉指向空气的 install 符号链接、重建、再走就绪闸，
# 最后按用户要求打开 GUI。
#
# 本脚本还会老实报一件我之前用猜测式过滤造成的假警：step 10 里 "websocket_bridge 0"
# 那句是我凭空写的模式 —— 树上 :8080 由 /tmp/multi_view_server.py、:8090 由
# /tmp/live_view_server.py 服务，websocket_bridge.py 在 install 里存在但本来就没在跑，
# 不是这次隔离弄坏的。这次改成按端口持有者点名。
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
WS=/home/hzm/v2x_ws
OUT=/tmp/quarantine_parketc_307
RUN=$OUT/finish_$(date +%m%d_%H%M%S).log
exec >>"$RUN" 2>&1
echo "日志=$RUN"

echo "===== 1) 前置：两份 CMakeLists 都应已无 parking/etc 安装项 ====="
for F in "$D/src/v2x_demo/CMakeLists.txt" "$WS/src/v2x_demo/CMakeLists.txt"; do
  echo "  $(grep -c -E 'parking_|etc_' "$F") 处残留｜$F"
done

echo "===== 2) 清掉 install 里指向已移动文件的符号链接（只删断链且名字含 parking_/etc_ 的）====="
N=0
for L in $(find "$WS/install/v2x_demo" -type l 2>/dev/null); do
  if [ ! -e "$L" ]; then
    case "$(basename "$L")" in
      parking_*|etc_*) rm "$L"; N=$((N+1)); echo "  删断链：${L#$WS/}";;
    esac
  fi
done
echo "  共删 $N 条断链（其余断链一条没动）"

echo "===== 3) colcon build 硬闸 ====="
cd "$WS"
source /opt/ros/humble/setup.bash
if colcon build --packages-select v2x_demo v2x_msgs > "$OUT/build2.log" 2>&1; then
  echo "  BUILD_OK"
else
  echo "  !! BUILD_FAIL —— 尾部原样："
fi
tail -14 "$OUT/build2.log" | tr -d '\r' | sed 's|^|    |'

echo "===== 4) 安装产物点名：lib 与 share/config 里不该再有 parking_/etc_ ====="
echo "  install/lib/v2x_demo：$(ls "$WS/install/v2x_demo/lib/v2x_demo" 2>/dev/null | grep -cE '^(parking_|etc_)') 项（应 0）"
echo "  install/share/config：$(ls "$WS/install/v2x_demo/share/v2x_demo/config" 2>/dev/null | grep -cE '^(parking_zones|etc_zones)') 项（应 0）"
echo "  还在的节点数：$(ls "$WS/install/v2x_demo/lib/v2x_demo" | wc -l)"

echo "===== 5) 就绪闸：任务一~四九路节点 + 已隔离两路 + 车队在动 + 两个端口 ====="
for PAT in 'signal[_]drive_node' 'traffic[_]light[_]manager' 'adaptive[_]timing_node' \
           'traffic[_]light[_]recorder' 'plate[_]gen[_]node' 'plate[_]recognizer[_]node' \
           'plate[_]recorder[_]node' 'speed[_]detect[_]node' 'speed[_]recorder[_]node'; do
  echo "  $(printf '%-26s' "$PAT") $(pgrep -cf "lib/v2x[_]demo/$PAT" || true)"
done
echo "  已隔离：parking $(pgrep -cf 'lib/v2x[_]demo/parking[_]' || true)｜etc $(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)（应 0）"
timeout -k 4 20 python3 "$D/tools/fleet_moving_guard303.py" /tmp/signal_drive_path.csv 20 2>&1 | tail -5 | sed 's|^|  |' || true
for P in 8080 8090; do
  PID=$(sudo -n fuser "${P}/tcp" 2>/dev/null | tr -d ' ' || true)
  [ -z "$PID" ] && PID=$(ss -tlnp 2>/dev/null | grep -a ":$P " | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
  echo "  :$P -> $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:$P/ || true)｜持有者 pid=${PID:-未知} $( [ -n "${PID:-}" ] && tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-70)"
done
echo "  gzserver=$(pgrep -x gzserver | head -1)｜gzclient=$(pgrep -x gzclient | head -1 || echo 无)"

echo "===== 6) 打开 GUI（用户点名）====="
bash "$D/tools/open_gui77.sh" 2>&1 | tr -d '\r' | tail -14 | sed 's|^|    |' || true
sleep 25
echo "  gzclient=$(pgrep -x gzclient | head -1 || echo 无)｜gzserver=$(pgrep -x gzserver | head -1)"

echo "===== 7) 结论 ====="
echo "  归档与移动：$D/存档_弃用任务五六_1016（tar 263 项 + 三处补丁原件备份 + home 侧副本）"
echo "  /tmp/live_view_server.py（:8090 看板）与 /tmp/multi_view_server.py（:8080 回显）是 /tmp 下的"
echo "  运行期脚本、不在工程树清单里；若它们的停车场/ETC 卡片读不到数据，那是节点没了的正常表现。"
echo "FINISH307DONE"
