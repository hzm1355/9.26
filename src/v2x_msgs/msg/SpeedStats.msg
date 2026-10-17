# 测速路段流量统计（协议 §三.(四).4）：每个测速点位/区间一条，speed_detect_node 周期发布。
std_msgs/Header header

string zone_id
uint8 MODE_INSTANT=0        # 定点瞬时测速点
uint8 MODE_SEGMENT=1        # 区间平均测速段
uint8 mode
string road                 # 点位所在的路段描述（人读，如"南环西行车道"）

float64 limit_kmh
float64 window_s            # 本条统计覆盖的时间窗

uint32 flow                 # 车流量：窗内通过本点位的车次（同一辆车重复通过按次计）
uint32 vehicles             # 窗内出现过的不同车辆数
float64 avg_kmh             # 平均车速（窗内通过车次的算术平均）
float64 max_kmh
float64 min_kmh
uint32 over_count           # 超速次数
float64 over_ratio          # over_count / flow
uint32 total_flow           # 自节点启动以来的累计车流量（不受窗口限制）
uint32 total_over           # 累计超速次数
uint32 pending_segment      # 区间内尚未离开的车辆数（只在 MODE_SEGMENT 有意义）
