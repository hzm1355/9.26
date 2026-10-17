# 交通灯统计信息
std_msgs/Header header
string group_id              # 信号组 ID

# 当前周期统计
uint32 vehicle_count         # 当前周期通过车辆数
float64 avg_wait_time        # 平均等待时间（秒）
float64 max_wait_time        # 最大等待时间（秒）
float64 throughput           # 通行效率（辆/分钟）

# 累计统计
uint32 total_vehicles        # 总通过车辆数
float64 total_wait_time      # 总等待时间（秒）
uint32 cycle_count           # 完成的周期数

# 自适应配时指标
float64 current_green_duration  # 当前绿灯时长（自适应调整后）
float64 adaptation_factor       # 自适应调整系数（0.5-2.0）

# 实时排队统计
uint32 queue_length             # 当前红灯等待排队车辆数
