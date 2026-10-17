# 交通信号灯状态（含行人信号）
std_msgs/Header header
string light_id              # 信号灯 ID（如 TL-J1-NE）
string group_id              # 信号组 ID（如 J1_ns）

# 机动车灯态
string vehicle_phase         # GREEN / YELLOW / RED
float64 phase_remaining      # 当前相位剩余时间（秒）

# 行人灯态
string pedestrian_phase      # WALK / DONT_WALK / FLASHING
float64 pedestrian_remaining # 行人相位剩余时间（秒）

# 配时参数（当前生效）
float64 green_duration
float64 yellow_duration
float64 red_duration
float64 walk_duration
float64 dont_walk_duration

# 控制模式
string control_mode          # FIXED / ADAPTIVE / EMERGENCY / MANUAL
