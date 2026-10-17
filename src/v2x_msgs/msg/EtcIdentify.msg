# ETC 识别事件（协议 §三.(六).2）：车辆进入 RSU 射口即产生一条，成功与失败都要有，
# 一条都不能少 —— "识别失败"本身是条目 4 要演的场景，只发成功事件就没法证伪。
# 由 etc_rsu_node 发 /etc/identify，etc_recorder_node 追加进 etc_identify.jsonl。
std_msgs/Header header      # stamp = 判定成立那一帧（仿真时钟）

string identify_id          # 全局唯一事件号，通行记录与告警都回指它

# ── 身份：OBU 是卡盒编号，车牌要"绑定"上（条目 2 的核心动作）──────────────
string vehicle_id           # Gazebo 模型名，如 car_1
string obu_id               # 命中的 OBU；读不到 OBU 时是空串
string registered_plate     # 该 OBU 在 etc_zones.yaml accounts 里的登记车牌
string on_board_plate       # 车上实际挂牌（来自 /plate/number 的兜底表运行期覆盖）
bool plate_match            # 登记牌 == 挂牌；不等就是"一车一卡、卡牌不符"那条管控
uint8 vehicle_class         # 车种，决定费率
string card_type            # 记账卡 / 储值卡 / 空串=未办理
string owner                # 持卡人，进导出便于人读

# ── 判定结果 ─────────────────────────────────────────────────────
bool success                # true = 读到且身份可用（不代表已抬杆，抬杆见 EtcPass）
uint8 FAIL_NONE=0
uint8 FAIL_NO_OBU=1         # 未办理 ETC：射口里没有这个车的 OBU 响应
uint8 FAIL_INTERFERENCE=2   # 干扰态下读取失败（条目 4 的"ETC 信号干扰"）
uint8 FAIL_TIMEOUT=3        # 响应超时：读到了但没读全
uint8 FAIL_CARD_INVALID=4   # 卡挂失/欠费超限（accounts.status 与 toll.arrears_limit）
uint8 fail_reason
string reason               # 人读补充，含"卡牌不符"这类 plate_match=false 的说明

# ── 识别几何（条目 4 要"可自定义识别距离"，所以触发时的量都要落进事件里）────
string gate_id              # 本事件归属的闸机（etc_gate_1）
string lane_id              # 车道编号
geometry_msgs/Pose pose     # 判定帧的车位姿（world 帧）
float64 along_m             # 车心沿行进轴到射口顶点的投影 —— 就是与 identify.distance 比的那个数
float64 range_m             # 车心到天线的直线距离
float64 beam_angle_rad      # 连线与 beam_axis 的夹角，与 beam_half_angle 比的那个数
float64 limit_m             # 当时生效的识别距离（可能被 /etc/params 热改过，必须自证）
float64 rate_used           # 当时生效的成功率
uint16 attempts             # 本次通行第几次尝试（重试带见 etc_zones.yaml retry_interval_s）
bool interference_active    # 事件是否落在干扰 burst 内

# ── 通行时间（条目 2 要输出的"通行时间"）──────────────────────────
float64 sim_s               # 识别时刻，仿真秒
string time_text            # 同一时刻的人读文本（HH:MM:SS.sss），导出与看板直接用
