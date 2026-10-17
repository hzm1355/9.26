#!/bin/bash
# 把 D 盘改好的三个摆渡车文件同步到 WSL 里的运行期副本 ~/v2x_ws/src/v2x_demo/。
#
# 为什么必须专门做这一步：改 D 盘源码树不会自动生效 —— 启动脚本读的是
# ~/v2x_ws/install/v2x_demo/...，而 install 侧是指向 ~/v2x_ws/src 的符号链接，
# ~/v2x_ws/src 又是一份**实体目录**（inode 与 D 盘不同）。所以链条是
#   D 盘源 → (手动 cp) → ~/v2x_ws/src → (symlink) → install → 进程。
# 中间那一跳没人替我们做，漏掉就是"代码改了、日志里还是老行为"。
#
# 只 cp 这三个文件，绝不整片覆盖：两边已经分叉，home 侧另有 .bak135/.bak136，
# D 盘这边 CMakeLists.txt、traffic_light_params.yaml、三个 launch 文件等和 home
# 不一致，全量 rsync 会把这些差异按 D 盘版本盖掉。
set +e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
H=$HOME/v2x_ws/src/v2x_demo
I=$HOME/v2x_ws/install/v2x_demo
FAIL=0

FILES="v2x_demo/parking_shuttle_node.py
v2x_demo/parking_shuttle_selftest.py
config/parking_zones.yaml"

echo '[sync] 0) install 侧到底是符号链接还是实体拷贝（决定 cp 完还要不要 colcon build）'
for F in $FILES; do
  ls -l "$I/$F" 2>&1 | sed 's|^|  |'
done

echo '[sync] 1) cp 前：D 盘 / home src / home install 三份的 md5'
for F in $FILES; do
  echo "  $F"
  md5sum "$D/$F" "$H/$F" "$I/$F" 2>&1 | sed 's|^|    |'
done

echo '[sync] 2) 逐个 cp（先备份 home 侧旧版，带 139 轮次标记）'
for F in $FILES; do
  cp -a "$H/$F" "$H/$F.bak139" 2>&1
  cp "$D/$F" "$H/$F" || FAIL=1
  echo "  cp $F -> $?"
done

echo '[sync] 3) cp 后复核：D 盘与 home src 必须同哈希'
for F in $FILES; do
  A=$(md5sum "$D/$F" 2>/dev/null | cut -d' ' -f1)
  B=$(md5sum "$H/$F" 2>/dev/null | cut -d' ' -f1)
  C=$(md5sum "$I/$F" 2>/dev/null | cut -d' ' -f1)
  if [ "$A" = "$B" ] && [ "$A" = "$C" ]; then
    echo "  ok   $F  三处一致 $A"
  else
    echo "  FAIL $F  D=$A home=$B install=$C"
    FAIL=1
  fi
done

echo '[sync] 4) 进程侧看到的到底是哪一份（/proc/<pid>/cwd + cmdline，当前应无 shuttle 进程）'
for PID in $(pgrep -f 'lib/v2x[_]demo/parking[_]shuttle' 2>/dev/null); do
  echo "  还在跑：PID $PID $(tr '\0' ' ' < /proc/$PID/cmdline | cut -c1-90)"
done
echo "  残留实例数=$(pgrep -cf 'lib/v2x[_]demo/parking[_]shuttle' 2>/dev/null)"

echo "[sync] DONE FAIL=$FAIL"
exit $FAIL
