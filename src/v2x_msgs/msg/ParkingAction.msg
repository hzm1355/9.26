# 闸机动作日志（协议 §三.(五).5）：每一次"抬/落/保持/故障/手动"记一条，
# 由 parking_recorder_node 追加进 barrier_actions.jsonl。通行记录见 ParkingPass，
# 两者用 gate_id + 时刻对齐；一条 ParkingPass 通常对应该闸机的一条 RAISE + 一条 LOWER。
std_msgs/Header header      # stamp = 动作成立时刻（仿真时钟）

string action_id

uint8 ACT_RAISE=0           # 抬杆到位（elapsed_s = 实际耗时）
uint8 ACT_LOWER=1           # 落杆到位
uint8 ACT_RAISE_CMD=2       # 下发抬杆目标角（还没到位）
uint8 ACT_LOWER_CMD=3       # 下发落杆目标角
uint8 ACT_HOLD_OPEN=4       # 防砸车：落杆时发现杆下仍被占用 → 立即重抬
uint8 ACT_REFUSE=5          # 拒绝动作（未核验就要求抬杆 / 故障态下要求落杆）
uint8 ACT_FAULT=6           # 故障生效（STUCK/RAISE_FAIL/NO_CLOSE/SENSOR_FAIL）
uint8 ACT_FAULT_CLEAR=7     # 故障清除，闸机恢复自动
uint8 ACT_MANUAL=8          # 手动控制生效（话题或 srv 都算）
uint8 ACT_DEGRADE=9         # 地感失效降级为计时放行
uint8 action

string gate_id
string vehicle_id           # 与该动作关联的车辆，无则空串
string plate_number         # 同上
uint8 source                # 取 BarrierState 的 SRC_* 常量
uint8 fault                 # 取 BarrierState 的 FAULT_* 常量
float64 from_rad            # 动作开始时实测角
float64 to_rad              # 本次目标角
float64 elapsed_s           # 到本条为止本次动作已耗时；CMD 类为 0
bool reached                # 是否到位（超时未到位=false，配合 fault 说明为什么）
string note                 # 人读补充：命中的名单、故障描述、手动来源
