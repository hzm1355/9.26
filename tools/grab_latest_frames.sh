#!/bin/bash
# 把最新的相机实拍帧拷到 D 盘，供 Read 工具直接看图。
#
# 为什么走这条路：WSL 侧没有授权安装 scrot/gnome-screenshot，PIL 自截图那条也被
# 安全策略按"访问显示服务 + 装第三方包"拦过。而卡口的 libv2x_gate 插件本来就在往
# /tmp/v2x_gates 落 JPEG——那就是现成的、渲染开启下的真实相机画面，不需要任何新依赖。
# 判据顺带有了：帧在增长 = gzserver 的渲染引擎活着 = 上一轮"gzserver 起来就段错误"
# 的故障没有复现。
#
# 用法：bash tools/grab_latest_frames.sh [每路取几帧]
set -e
OUT=/mnt/d/desktop1/9.26.1/_frames
N=${1:-3}
mkdir -p $OUT
for D in /tmp/v2x_gates /tmp/v2x_parking_in /tmp/v2x_parking_out; do
  if [ ! -d "$D" ]; then
    echo "$D  目录不存在"
    continue
  fi
  echo "$D  共 $(ls -1 $D 2>/dev/null | wc -l) 项"
  i=0
  for F in $(ls -t "$D" 2>/dev/null | grep -E '\.(jpg|jpeg|png)$' | head -$N); do
    TAG=$(basename $D)
    cp "$D/$F" "$OUT/${TAG}__${F}"
    echo "  -> ${TAG}__${F}"
    i=$((i + 1))
  done
  [ "$i" = "0" ] && echo "  (暂无图像帧)"
done
