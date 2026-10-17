# 黑名单告警（协议 §三.(三).5）：识别结果命中黑名单时发布 /plate/alert。
# 同一次抓拍可命中多条名单，每条各发一告警，用 capture_id 关联原始记录。
std_msgs/Header header      # stamp = 告警触发时刻

uint8 LEVEL_INFO=0
uint8 LEVEL_WARN=1
uint8 LEVEL_CRITICAL=2
uint8 level                 # 告警级别，取自命中名单条目的级别

string plate_number         # 命中的车牌号
string reason               # 入黑原因/备注
string capture_id           # 触发本告警的抓拍编号
string gate_id              # 触发本告警的卡口 ID

geometry_msgs/Pose vehicle_pose   # 车辆位置
float64 speed                     # 车辆速度 m/s
string image_path                 # 抓拍图路径，界面告警弹窗直接显示
