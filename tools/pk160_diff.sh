#!/bin/bash
# 量化"运行期副本 vs 仓库源码"的差：停车场模块每个 py 都 diff 一次，只看差几行、差在哪。
set +e
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo
WS=$HOME/v2x_ws/src/v2x_demo/v2x_demo

for F in parking_manager_node.py parking_recorder_node.py parking_shuttle_node.py \
         parking_manager_selftest.py parking_recorder_selftest.py \
         parking_shuttle_selftest.py plate_recognizer_node.py; do
  if [ ! -e "$WS/$F" ]; then
    printf '%-32s 工作区里没有这个文件\n' "$F"; continue
  fi
  D=$(diff "$WS/$F" "$SRC/$F" | grep -ac '^[<>]')
  printf '%-32s 差异行数=%-5s 仓库字节=%-6s 工作区字节=%-6s\n' "$F" "$D" \
    "$(stat -c %s "$SRC/$F")" "$(stat -c %s "$WS/$F")"
done

echo "=== manager 的差异明细（工作区缺什么）==="
diff "$WS/parking_manager_node.py" "$SRC/parking_manager_node.py" | head -60

echo "=== 其它两个停车场节点的差异摘要 ==="
for F in parking_recorder_node.py parking_shuttle_node.py; do
  echo "## $F"
  diff "$WS/$F" "$SRC/$F" | grep -a '^[<>]' | head -12
done

echo "=== 运行中的 manager/shuttle 到底是哪个 inode（/proc 里看真实路径）==="
MP=$(pgrep -f 'lib/v2x[_]demo/parking_manager_node' | head -1)
SP=$(pgrep -f 'lib/v2x[_]demo/parking_shuttle_node' | head -1)
for P in $MP $SP; do
  printf 'pid %-8s cwd=%s exe=%s\n' "$P" "$(readlink /proc/$P/cwd)" "$(readlink /proc/$P/exe)"
  tr '\0' ' ' < /proc/$P/cmdline | cut -c1-140; echo
done
