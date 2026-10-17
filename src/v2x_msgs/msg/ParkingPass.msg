# 停车场通行记录（协议 §三.(五).2/5）：一次"核验→抬杆→通行"落一条，
# 进场与出场各算一条，由 parking_recorder_node 追加进 parking_pass.jsonl。
std_msgs/Header header      # stamp = 本条记录成立时刻（放行的那一帧，不是到场的）

string pass_id              # 全局唯一通行编号，导出/查询/告警都用它回指

uint8 DIR_IN=0              # 进场：入口闸机，读车头牌
uint8 DIR_OUT=1             # 出场：出口闸机，读车尾牌
uint8 direction

string gate_id              # gate_in / gate_out

# ── 识别到的车牌（来自 /plate/result 参考识别链路）──────────────
string plate_number         # 识别文本；识别失败时是空串，且 recognized=false
bool recognized
float64 plate_confidence    # 参考识别的置信度 0..1
string capture_id           # 回指 PlateCapture，图是那张图
string image_path           # 核验用抓拍图绝对路径

# ── 车辆 ─────────────────────────────────────────────────────
string vehicle_id           # Gazebo 模型名，如 car_6
geometry_msgs/Pose pose     # 核验位位姿（world 帧）

# ── 管控判定（协议 §三.(五).4）─────────────────────────────────
uint8 KIND_MONTHLY=0        # 白名单月租车
uint8 KIND_TEMP=1           # 临时车（已发临时卡）
uint8 KIND_UNKNOWN=2        # 未登记且 unknown_as_temp=false → 拒入
uint8 KIND_BLACKLIST=3      # 命中黑名单
uint8 kind

uint8 RESULT_ALLOWED=0      # 放行：已抬杆
uint8 RESULT_DENIED=1       # 拒绝：未抬杆，reason 说明为什么
uint8 RESULT_PENDING=2      # 挂起：等临时车登记/等人工确认，尚未抬杆
uint8 result
string reason               # 拒绝/挂起原因，人读文本

string ticket_id            # 临时卡号；KIND_TEMP 且有值，形如 T-<入场时刻>
uint8 whitelist_hit         # 白名单命中：0=未查/1=命中/2=未命中/3=命中黑名单

# ── 时序（用于"闸机抬杆→车辆通行→落杆"整段耗时取证）──────────
float64 arrive_s            # 到场（进入核验区）时刻，仿真秒
float64 raise_s             # 抬杆到位时刻，未抬杆为 -1
float64 clear_s             # 地感释放（车尾离开杆下）时刻，未到为 -1
float64 lower_s             # 落杆到位时刻，未落为 -1
