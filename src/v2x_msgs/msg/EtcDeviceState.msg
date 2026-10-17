# ETC 设备状态（协议 §三.(六).1/5）：门架 / RSU / 道闸 / 地感四类设备各发一条，
# 是条目 5 要的"设备状态日志"，也是条目 4 那块控制面板的状态来源。
# etc_manager_node 周期发 /etc/device_state（含管理器自己的心跳），
# etc_recorder_node 追加进 etc_devices.jsonl。
std_msgs/Header header      # stamp = 本条采时刻（仿真时钟）

uint8 TYPE_GANTRY=0         # ETC 门架：静态构筑物，状态只有"在位/被移除"
uint8 TYPE_RSU=1            # 虚拟路侧单元：射口几何 + 读写计数
uint8 TYPE_BARRIER=2        # 道闸：角度、目标、故障
uint8 TYPE_LOOP=3           # 地感线圈：占用与检出/释放计数
uint8 type

string device_id            # ETC-G1 / ETC-RSU-1 / etc_gate_1 / ETC-L1
string gate_id              # 归属闸机，门架与 RSU 也填同一个，便于按卡口聚合
string lane_id

uint8 STATUS_ONLINE=0
uint8 STATUS_DEGRADED=1     # 降级仍在服务：如地感失效改计时放行、管理器掉线走 fail_open
uint8 STATUS_OFFLINE=2      # 该设备对应的节点没在发（或插件被卸载）
uint8 STATUS_FAULT=3        # 有生效中的注入故障
uint8 status

# ── 道闸专用（type=TYPE_BARRIER 时才有意义，其余设备填 0/-1）────────
float64 boom_angle          # 实测杆角 rad
float64 boom_target         # 目标杆角 rad
bool at_limit               # 是否已到位（|angle-target| <= angle_tolerance）
bool occupied               # 地感：杆下是否被占用（TYPE_LOOP 用它）
uint8 fault                 # 取 BarrierState 的 FAULT_* 常量
string fault_source         # 谁注入的：auto / topic / srv / 面板，空串=无

# ── RSU 专用：识别口径的自证（条目 4 改了参数要在这里读回生效值）────────
float64 identify_limit_m    # 当前生效的识别距离
float64 success_rate        # 当前生效的成功率
bool interference_active    # 干扰 burst 是否正在生效
uint16 ok_count             # 累计识别成功次数
uint16 fail_count           # 累计识别失败次数（含未办理与干扰）

# ── 管理器在不在场（safety.fail_open 的唯一可观测证据）──────────────
bool manager_alive
float64 manager_age_s       # 距管理器上一次心跳多少秒；> manager_watchdog_s 判掉线
uint16 pass_count           # 管理器累计闭合的通行条数，用于对账
string note
