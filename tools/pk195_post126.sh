#!/bin/bash
# pk126 收尾复核：world 行尾有没有被生成器整片改成 LF + 补上 yaml 路径重跑布局体检。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
BAK=$(ls "$W".pre_parking_*.bak | tail -1)
cd "$SIM" || exit 1
echo "--- 字节数 ---"
ls -l "$W" "$BAK" | sed 's/^/  /'
echo "--- 行尾 ---"
CR=$(tr -cd '\r' < "$W" | wc -c); LF=$(tr -cd '\n' < "$W" | wc -c)
BCR=$(tr -cd '\r' < "$BAK" | wc -c); BLF=$(tr -cd '\n' < "$BAK" | wc -c)
echo "  当前 CR=$CR LF=$LF     备份 CR=$BCR LF=$BLF"
if [ "$CR" = "0" ] && [ "$BCR" != "0" ]; then
  echo "  !! 生成器把整个 world 重写成 LF 了（备份是 CRLF）。gzserver 能吃 LF，"
  echo "     但后续按行断言的脚本要统一按 LF 读，别再混。"
fi
echo "--- 生成块内部行尾抽样（2271 前后）---"
sed -n '2269,2273p' "$W" | od -c | grep -ao '\\r' | head -1 | sed 's/^/  生成块里有 CR: /'
echo "--- 布局体检（带 yaml 路径）---"
python3 tools/check_parking_layout.py src/v2x_demo/config/parking_zones.yaml 2>&1 | tail -30 | sed 's/^/  /'
echo "  退出码 ${PIPESTATUS[0]}"
echo PK126CHECKDONE
