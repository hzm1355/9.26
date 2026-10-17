# 道闸状态（协议 §三.(五).3/4）：由 v2x_barrier_plugin 按 joint 实测值发布，
# parking_manager_node 把逻辑状态机叠在同一份消息上，所以界面只订一个话题就能看到
# "指令角 vs 实测角"的偏差 —— 抬杆失败、卡滞都是从这条偏差里长出来的，不是节点自己声明的。
std_msgs/Header header      # stamp = 采样时刻（use_sim_time 下即仿真时钟）

string gate_id              # gate_in / gate_out

# ── 逻辑状态机：杆在流程里的角色 ─────────────────────────────────
uint8 STATE_CLOSED=0        # 杆水平，车道关闭
uint8 STATE_OPENING=1       # 抬杆中
uint8 STATE_OPEN=2          # 开到位，允许通行
uint8 STATE_CLOSING=3       # 落杆中
uint8 STATE_FAULT=4         # 故障挂起（安全侧保持抬起，等人工/自动处置）
uint8 state

# ── 本次动作是谁下的令 ─────────────────────────────────────────
uint8 SRC_AUTO=0            # 核验通过/地感离场触发的自动流程
uint8 SRC_MANUAL=1          # /parking/gate_manual_cmd 或 SetParkingGate
uint8 SRC_FAULT=2           # 故障注入/清除引起的强制动作
uint8 source

# ── 故障位（协议 §三.(五).4）─────────────────────────────────────
uint8 FAULT_NONE=0
uint8 FAULT_STUCK=1         # 卡滞：角在动但力矩顶满 / 中途停住
uint8 FAULT_RAISE_FAIL=2    # 抬杆失败：目标抬起角在 raise_timeout_s 内到不了
uint8 FAULT_NO_CLOSE=3      # 落杆失败：目标落下角在 lower_timeout_s 内到不了
uint8 FAULT_SENSOR_FAIL=4   # 地感失效：本闸机降级为"计时放行"，不再自动落杆
uint8 fault

# ── 动力学实测（插件是权威，节点只是转发）───────────────────────
float64 target_rad          # 当前目标角：0=落下，open_angle=抬起
float64 angle_rad           # revolute joint 实测角
float64 angle_error_rad     # target - angle
bool at_target              # |angle_error| <= angle_tolerance
float64 angvel_rps          # 角速度 rad/s
float64 effort_nm           # joint 实测力矩，判堵转用（顶满 effort 且角速度归零 = 压到东西）
float64 action_elapsed_s    # 本次抬/落已耗时，超时判定用

# ── 联动几何：地感 ─────────────────────────────────────────────
string loop_id              # PKG-in-loop / PKG-out-loop
bool loop_occupied          # 地感当前是否被车压住
string vehicle_id           # 压住地感的车辆模型名，空串=没有车

# ── 统计（开机累计，导出与对账用）──────────────────────────────
uint32 raise_count
uint32 lower_count
