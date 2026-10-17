#!/bin/bash
# 同步前的差异体检：ws 安装树里那份拷贝相对 D 盘源码树有什么独有内容。
# 目的是确认 `sync_demo.sh` 的 rm -rf + cp 不会丢掉只在 ws 里改过的东西。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
WS=~/v2x_ws/src/v2x_demo
echo "=== 逐字节不同的文件清单（忽略行尾 CR）==="
diff -rq --strip-trailing-cr --exclude=__pycache__ "$WS" "$SIM"
echo "=== 只看内容真的不一样的那几个 ==="
for f in CMakeLists.txt config/traffic_light_params.yaml launch/v2x_bringup.launch.py \
         launch/v2x_nav2.launch.py launch/v2x_teleop.launch.py v2x_demo/__init__.py \
         v2x_demo/traffic_light_controller.py; do
    echo "##### $f"
    diff --strip-trailing-cr "$WS/$f" "$SIM/$f" | head -12
done
echo DONE
