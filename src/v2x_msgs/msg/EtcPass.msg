# ETC 通行记录（协议 §三.(六).3）：一次"识别→抬杆→不停车通行→地感离场→落杆→扣费"
# 落一条，由 etc_manager_node 发 /etc/pass、etc_recorder_node 追加进 etc_pass.jsonl。
# 与 EtcIdentify 用 identify_id 回指；识别失败也有一条本记录（result=DENIED），
# 这样"没放行的车"在账上同样看得见，条目 5 的导出才对得齐。
std_msgs/Header header      # stamp = 本次通行闭合那一帧（落杆到位或拒绝成立）

string pass_id              # 形如 E0001，跨重启不重号（由管理器用启动时刻播种）
string identify_id          # 回指 EtcIdentify；未产生识别事件（如完全没读到）时为空

# ── 身份 ───────────────────────────────────────────────────────
string vehicle_id
string plate_number         # 生效车牌：识别到就用识别到的，没识别到用兜底表
string obu_id               # 空串 = 未办理 ETC
string card_type            # 记账卡 / 储值卡 / 空串
uint8 vehicle_class
string gate_id
string lane_id
uint8 DIR_IN=0              # 沿用停车场那套 in/out 进入语义：车流从 -y 方向进
uint8 DIR_OUT=1
uint8 direction

# ── 管控结果 ───────────────────────────────────────────────────
uint8 RESULT_ALLOWED=0      # 放行：已抬杆，扣费见 fee_*
uint8 RESULT_DENIED=1       # 拒绝：未抬杆（未办理 / 卡无效 / 余额不足 / 卡牌不符）
uint8 RESULT_PENDING=2      # 挂起：等人工或等下一次识别，尚未定论
uint8 RESULT_FOLLOWED=3     # 跟车闯关：杆是被前车抬起来的，本车没独立完成识别
uint8 result
string reason

# ── 扣费（条目 3"可模拟扣费逻辑"）───────────────────────────────
uint8 PAY_NONE=0            # 没扣：拒绝或跟车闯关
uint8 PAY_DEDUCT=1          # 储值卡当场扣余额
uint8 PAY_POST_BILL=2       # 记账卡累计账单（余额允许为负，到 arrears_limit 转拒入）
uint8 pay_mode
float64 fee_cny             # 本次应收（按 vehicle_class 查 fee_by_class）
float64 balance_before
float64 balance_after
string currency

# ── 几何与时序（"不停车通行"的证据就在这几行里：全程没有减速到停）──────────
geometry_msgs/Pose pose     # 过杆位姿（车尾离开杆下那一帧）
float64 identify_s          # 识别成立时刻，未识别为 -1
float64 raise_s             # 抬杆到位时刻，未抬杆为 -1
float64 touch_s             # 车头到达杆轴时刻（抬杆提前量的实测起点）
float64 clear_s             # 地感释放（车尾离开杆下）时刻，未到为 -1
float64 lower_s             # 落杆到位时刻，未落为 -1
float64 speed_min           # 本次通行窗口内实测最小速度 m/s —— 不停车判据：> 0 才算没停
float64 speed_mean

# ── 关联与告警挂钩 ──────────────────────────────────────────────
bool followed_previous      # true = 本次通行时前车抬的杆还没落下（跟车闯关）
string prev_pass_id         # 被跟的那条通行记录
uint8 fault                 # 取 BarrierState 的 FAULT_* 常量，0 表示闸机本身没故障
string note
