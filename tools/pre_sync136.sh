#!/bin/bash
# 复验前必查：运行期加载的是 ~/v2x_ws/install 里的软链目标，而 ~/v2x_ws/src/v2x_demo 是
# D 盘的**独立拷贝**（不是软链），所以 D 盘改了不同步就等于没改（#135 踩过）。
# --strip-trailing-cr：D 盘文件带 CRLF，直接 diff 会全文标红。
set +e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo
W=$HOME/v2x_ws/src/v2x_demo/v2x_demo

echo '== 运行期实际加载的是哪份（查 /proc/PID/cwd + cmdline，别猜）=='
for N in parking_manager_node parking_shuttle_node parking_recorder_node; do
  P=$(pgrep -f "lib/v2x[_]demo/${N}" | head -1)
  if [ -n "$P" ]; then
    echo "  ${N}: pid $P -> $(readlink -f /proc/$P/cwd)$(tr '\0' ' ' < /proc/$P/cmdline | grep -o 'lib/v2x_demo/[^ ]*')"
  else
    echo "  ${N}: 没在跑"
  fi
done

echo '== 包内 .py 差异（只列文件名）=='
diff -rq --strip-trailing-cr "$D" "$W" 2>&1 | grep -av '__pycache__' | sed 's/^/  /'

echo '== 配置 yaml 是否也从 D 盘直读 ==  '
grep -ao 'config_file:=[^ ]*' /tmp/parking_manager_node.log 2>/dev/null | tail -1 | sed 's/^/  manager 日志: /'
pgrep -af 'lib/v2x[_]demo/parking_manager_node' | tr -d '\r' | grep -ao '\-p config_file:=.*' | sed 's/^/  现进程参数: /'
echo DONE
