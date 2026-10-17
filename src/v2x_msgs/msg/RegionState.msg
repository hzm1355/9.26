# 区域占用状态（transient_local 发布，晚订阅者可拿到最新快照）
std_msgs/Header header
string region_id
string region_type
bool occupied             # 区域内是否有车
string[] vehicle_ids      # 当前区域内车辆模型名
