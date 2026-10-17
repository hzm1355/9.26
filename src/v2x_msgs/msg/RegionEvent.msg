# 区域触发事件：车辆进入/离开配置区域（测速/卡口/停车场/ETC 的公共底座）
std_msgs/Header header
string region_id          # 区域 ID（regions.yaml 中定义）
string region_type        # 区域用途：etc_gate / parking_gate / parking_loop / speed_point / checkpoint / custom
string vehicle_id         # 触发车辆的 Gazebo 模型名

uint8 EVENT_ENTER=1
uint8 EVENT_EXIT=2
uint8 event_type

geometry_msgs/Pose pose   # 事件时刻车辆位姿（world 帧）
float64 speed             # 事件时刻线速度 m/s（水平合成，来自 ModelStates twist）
