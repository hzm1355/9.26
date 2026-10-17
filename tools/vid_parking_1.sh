#!/bin/bash
# 任务五 条目1 视频取证：停车场场景建模（入口/出口区域、道闸设备、卡口摄像头、
# 地感线圈），点位与尺寸全部可配置。
#
# 这一段要演实"可配置"三件事，缺一条都算没做到：
#   · 每个要素的点位/尺寸都能在 parking_zones.yaml 里点名（1b 的表就是从那读的）；
#   · 几何之间是自洽的，不是抄来的数（1c 跑 check_parking_layout.py 的断言）；
#   · 改一处 yaml，生成的 world SDF 只有对应那几行变（1d 用 --print 做前后 diff，
#     不落盘、不碰正在跑的 world，零风险）。
# 为什么 1d 不直接改 world 再重启：world 侧生效要重启 gzserver，那是整栈动作，
# 会把车队与其它取证一起打断；"配置→SDF"这条链用 --print 就能完整演给读片的人看。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
SIM=$SIM_ROOT     # vid_common.sh 给的是 SIM_ROOT；这里下面全按 $SIM 写路径，不定义会展开成 /src/...
OUT="$VID_ROOT/任务五_停车场闸机/条目1_场景建模_四要素点位尺寸与可配置.mp4"
YAML=$SIM/src/v2x_demo/config/parking_zones.yaml

vid_start "$OUT" 260 "$VIEW_PK,$VIEW_PIN" parking \
  "任务五 条目1  停车场场景建模（入口/出口 + 道闸 + 卡口相机 + 地感，点位尺寸可配置）"
cap "上格：intersection_cam 高位画面裁出停车场那块（红白闸杆/机箱/龙门架/线圈框/车位线都在里面）"
cap "下格：入口卡口相机的实拍画面（同一台虚拟相机拍到的核验位）"
sleep 6
cap "== 1a 现场要素点名（画面里能指认的实体，全部来自 world 的生成块）=="
capq "grep -aoE 'model name=\"parking_[a-z_0-9]+\"' $SIM/src/smart_car_sim/worlds/v2x_city_scene.world | sort | uniq -c"
sleep 6
cap "== 1b 点位与尺寸总表（唯一真值 parking_zones.yaml，节点与 world 生成器共读一份）=="
capq "python3 $TOOLS/parking_points_dump.py --zones $YAML | sed -n '1,14p'"
sleep 10
capq "python3 $TOOLS/parking_points_dump.py --zones $YAML | sed -n '15,28p'"
sleep 10
capq "python3 $TOOLS/parking_points_dump.py --zones $YAML | sed -n '29,42p'"
sleep 10
cap "  入口/出口车道、通道带、中岛机箱位、龙门架横梁与立杆、车位、线圈框 —— 全是配置项"
cap "== 1c 几何自洽体检：杆轴/相机俯角反推链/地感防砸余量/摆渡路线扫掠/车道净空 =="
capq "timeout -k 5 180 python3 $TOOLS/check_parking_layout.py $YAML --obj $SIM/src/smart_car_sim/models/room_33/meshes/room_33.obj 2>&1 | tail -6"
sleep 8
cap "  '体检通过'+'受阻 0 格'= 卡口相机抓拍点确实落在核验位车牌面上，落杆不会压到车，"
cap "  且占地矩形里没有建筑/设施占位（957 格 0.25 m 栅格逐格查）"
cap "== 1d 可配置性实证：只把地感线圈 size 从 1.30x0.80 改成 2.00x1.20，看生成的 SDF 差在哪 =="
capq "cp $YAML /tmp/pk_base.yaml; sed 's/size: \\[1.30, 0.80\\]/size: [2.00, 1.20]/' /tmp/pk_base.yaml > /tmp/pk_mod.yaml; echo \"改动的配置行（这一串在 yaml 里出现两次，两道闸的地感一起改）：\"; grep -n 'size: \\[2.00, 1.20\\]' /tmp/pk_mod.yaml"
sleep 4
capq "python3 $TOOLS/gen_parking_world.py --zones /tmp/pk_base.yaml --print > /tmp/pk_base.sdf 2>&1; python3 $TOOLS/gen_parking_world.py --zones /tmp/pk_mod.yaml --print > /tmp/pk_mod.sdf 2>&1; echo \"基线 $(wc -l < /tmp/pk_base.sdf) 行 / 改后 $(wc -l < /tmp/pk_mod.sdf) 行\""
sleep 6
cap "  两份生成块逐行 diff：改后的行数不变，只有地感那四格的 pose/size 跟着变（前 12 行）"
capq "diff /tmp/pk_base.sdf /tmp/pk_mod.sdf | head -12"
sleep 8
cap "  同一份 yaml 还被三个节点读：闸机判定、地感占用、摆渡路线都跟着这个数走，不存在第二套几何"
cap "== 1e 运行期侧的热配置：闸机角度/白名单走 srv，不必重启（详见条目 4）=="
capq "$PKCLI gate list"
sleep 8
cap "== 1f 画面复核：此刻两道闸的实测状态（插件按 joint 实测发布，不是节点自己声明）=="
capq "timeout -k 3 20 ros2 topic echo /parking/gate_state v2x_msgs/msg/BarrierState --no-daemon --once 2>&1 | grep -aE 'gate_id|state:|target_rad|angle_rad|loop_occupied|vehicle_id|raise_count' | head -14"
sleep 8
cap "== 1g 摆渡车在不在动（条目 1 的场地是活的：有车进出才算建模成功）=="
capq "for M in car_3 car_2; do printf '%-7s ' \$M; gz model -m \$M -p 2>&1 | head -1; done"
capq "timeout -k 3 12 ros2 topic echo /parking/shuttle_status std_msgs/msg/String --no-daemon --once 2>&1 | head -3"
vid_wait
echo DONE
