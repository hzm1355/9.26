#!/bin/bash
# 改完节点代码后先确认"运行期到底加载哪一份"：install 侧是符号链接还是要重新 install。
# 依据：memory「先确认运行期加载的是哪份文件副本」——同名副本 + 相对路径启动会把改动吞掉。
set -e
sed -i 's/\r$//' "$0"
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo/parking_shuttle_node.py
echo "=== 源文件 $SRC"
ls -l "$SRC"
grep -c BRAKE_LAG "$SRC" | sed 's/^/  BRAKE_LAG 出现次数: /'
for d in "$HOME/v2x_ws/install/v2x_demo/lib/python3.10/site-packages/v2x_demo" \
         "$HOME/v2x_ws/build/v2x_demo/lib/v2x_demo"; do
  f="$d/parking_shuttle_node.py"
  echo "=== $f"
  if [ -e "$f" ]; then
    ls -l "$f"
    if [ -L "$f" ]; then echo "  符号链接 -> $(readlink -f "$f")"; fi
    echo -n "  BRAKE_LAG 出现次数: "; grep -c BRAKE_LAG "$f" || true
  else
    echo "  不存在"
  fi
done
echo "=== 正在跑的 shuttle 进程（cwd/cmdline 决定它读的是哪份）"
pgrep -af parking_shuttle_node || echo "  没有 shuttle 进程在跑"
for p in $(pgrep -f parking_shuttle_node || true); do
  echo "  PID $p cwd=$(readlink -f /proc/$p/cwd 2>/dev/null)"
  tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null; echo
done
echo "=== 整栈还活着吗（gzserver / 另两个停车场节点）"
pgrep -af gzserver | head -2 || echo "  gzserver 没在跑"
pgrep -af parking_manager_node || echo "  manager 没在跑"
pgrep -af parking_recorder_node || echo "  recorder 没在跑"
