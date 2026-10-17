# 卡口抓拍原始记录（协议 §三.(三).1/2）：由卡口 SensorPlugin 在车辆通过触发区的瞬间发布。
# 这一层只描述"拍到了什么"，不含任何识别结论，识别结果见 PlateResult。
std_msgs/Header header      # stamp = 抓拍时刻（仿真时间，与图像帧严格同拍）
string capture_id           # 全局唯一抓拍编号，识别结果与告警用它回指
string gate_id              # 卡口 ID：gate_j3_north / gate_j2_south

string vehicle_id           # 被拍车辆的 Gazebo 模型名
geometry_msgs/Pose pose     # 抓拍瞬间车辆位姿（world 帧）
float64 speed               # 抓拍瞬间线速度 m/s（水平合成）

uint16 image_width
uint16 image_height
string image_encoding       # bgr8
string image_path           # 抓拍图落盘绝对路径，供导出与人工复核
