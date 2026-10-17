# 区间平均测速（协议 §三.(四).2）：车辆依次通过区间的进入断面与离开断面各一次，
# 由 speed_detect_node 在离开时发一条。均速 = 区间长度 / 行程时间，与 SUMO 的 e2/e3
# 车道区域检测器同一套语义（区别于 SpeedCapture 的断面瞬时读数）。
std_msgs/Header header      # stamp = 离开断面时刻

string segment_id           # 本次区间通行编号
string zone_id              # 区间 ID

string vehicle_id
string plate_number

float64 entry_ts            # 通过进入断面时刻（仿真秒）
float64 exit_ts             # 通过离开断面时刻（仿真秒）
float64 travel_time_s       # exit_ts - entry_ts
float64 length_m            # 两断面之间的通行长度 m

float64 avg_kmh             # 区间平均车速 = length / travel_time
float64 limit_kmh           # 区间限速阈值
float64 over_kmh            # avg_kmh - limit_kmh
bool over

float64 max_kmh             # 区间内各测速点/采样到的瞬时峰值 km/h（辅助判定是否中途急刹）
float64 entry_speed_kmh     # 进入断面瞬间的雷达读数 km/h
float64 exit_speed_kmh      # 离开断面瞬间的雷达读数 km/h

geometry_msgs/Pose entry_pose
geometry_msgs/Pose exit_pose
