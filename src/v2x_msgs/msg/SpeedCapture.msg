# 定点瞬时测速抓拍（协议 §三.(四).2/3）：车辆通过某个测速点时，由 speed_detect_node 发一条。
# 速度单位一律 km/h —— 仿真里车是 0.7 m/s 级低速跑，阈值按 km/h 写才不会把单位混进判定式。
std_msgs/Header header      # stamp = 触发时刻（use_sim_time 下即仿真时钟）

string capture_id           # 全局唯一测速编号，告警与导出用它回指
string zone_id              # 测速点位 ID（speed_zones.yaml 中定义）

string vehicle_id           # 车辆编号（Gazebo 模型名，如 car_3）
string plate_number         # 车牌（来自 /plate/number 真值，未贴牌时为空串）

float64 speed_kmh           # 本次判定用的雷达读数 km/h
float64 speed_pose_kmh      # 同一时刻由位姿差分算出的速度 km/h（雷达读数的对照通道）
float64 speed_peak_kmh      # 本车在本点作用区间内的雷达峰值 km/h
float64 limit_kmh           # 该点位限速阈值
float64 over_kmh            # speed_kmh - limit_kmh，<=0 表示不超速
bool over

float64 heading_deg         # 车头朝向（度）
float64 beam_angle_deg      # 车头与雷达波束轴的夹角（度）：多普勒读数的余弦来源
float64 radar_range_m       # 触发瞬间车到雷达杆的距离 m
float64 noise_kmh           # 本次注入的雷达测量噪声 km/h（可复算，不是黑箱）

geometry_msgs/Pose pose     # 触发瞬间车辆位姿（world 帧）
