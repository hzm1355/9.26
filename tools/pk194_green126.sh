#!/bin/bash
# 条目 126：把停车场地坪由黑沥青改绿，落地到 world 并证明"只有颜色变了"。
#
# 为什么每次都要先 cp 一份改前的 world：本脚本的交付物不是"生成成功"，而是
# "生成块与改前逐行对比、差异只在 <material> 那几行"。几何/航线/净空常数一旦跟着
# 动了，#127 的车模替换就分不清是谁改出来的（memory「生效要用效果复核」）。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$SIM/src/smart_car_sim/worlds/v2x_city_scene.world
PRE=/tmp/world.pre_126.world
unset ROS_DOMAIN_ID

echo '=== 0) 存档改前 world ==='
cp -a "$W" "$PRE" || exit 1
md5sum "$PRE" | sed 's/^/  /'

echo '=== 1) 去掉 CRLF（D 盘文件是 \\r\\n，脚本/yaml 解析会带上 \\r）==='
for F in tools/gen_parking_world.py tools/check_parking_layout.py tools/pk188_worldprechk316.py \
         src/v2x_demo/config/parking_zones.yaml; do
  sed -i 's/\r$//' "$SIM/$F" && echo "  已处理 $F"
done

echo '=== 2) 生成器 --print（先看不写，确认色值真的落到 asphalt）==='
cd "$SIM" || exit 1
python3 tools/gen_parking_world.py --print 2>&1 | grep -aE 'C_ASPHALT|0\.07 0\.14|asphalt|ERROR|Traceback' | head -12 | sed 's/^/  /'

echo '=== 3) 写回 world ==='
python3 tools/gen_parking_world.py 2>&1 | tail -8 | sed 's/^/  /'
RC=$?
echo "  退出码 $RC"
[ "$RC" = "0" ] || exit 1

echo '=== 4) 逐行差异：只允许出现在 <material> 这一行（生成块里没有时间戳，任何其它差异都是几何漂移）==='
diff "$PRE" "$W" > /tmp/126.diff 2>&1
echo "  diff 行数 $(wc -l < /tmp/126.diff)，改动行数 $(grep -acE '^[<>]' /tmp/126.diff)"
grep -aE '^[<>]' /tmp/126.diff | sed 's/^/  /' | head -40
OTHER=$(grep -aE '^[<>]' /tmp/126.diff | grep -av '<material>' | grep -acve '^[[:space:]]*$')
echo "  非 <material> 的改动行：$OTHER（要求 0）"
[ "$OTHER" = "0" ] || echo '  !! 有几何行跟着变了，先查生成器再谈 #127'

echo '=== 5) 体检：布局 + world 断言（含出生格互不互穿）==='
python3 tools/check_parking_layout.py 2>&1 | tail -12 | sed 's/^/  /'
echo "  check_parking_layout 退出码 ${PIPESTATUS[0]}"
python3 tools/pk188_worldprechk316.py 2>&1 | tail -8 | sed 's/^/  /'
echo "  pk188 退出码 ${PIPESTATUS[0]}"

echo '=== 6) 生成块世代指纹 ==='
grep -acE 'BEGIN GENERATED: parking' "$W" | sed 's/^/  BEGIN 标记数：/'
grep -a 'asphalt' "$W" | head -3 | sed 's/^/  /'
grep -n '0.07 0.14 0.09 1\|0.18 0.38 0.22 1' "$W" | sed 's/^/  新色值行 /'
echo PK126DONE
