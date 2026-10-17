# 车牌识别结果（协议 §三.(三).2）：内置 OpenCV 参考识别器或外部算法节点发布。
# 需求要求结果含"车牌号码 / 抓拍时间 / 车辆位置"三项，分别为 plate_number / capture_stamp / vehicle_pose。
std_msgs/Header header      # stamp = 识别完成时刻

string capture_id           # 对应 PlateCapture.capture_id
string gate_id              # 抓拍卡口 ID
builtin_interfaces/Time capture_stamp   # 抓拍时刻（透传自 PlateCapture）

string plate_number         # 识别出的车牌号，识别失败为空串
float64 confidence          # 0~1，模板匹配平均相似度
uint16 bbox_x               # 车牌在抓拍图中的外接框（像素）
uint16 bbox_y
uint16 bbox_w
uint16 bbox_h

string algorithm            # 识别来源：builtin_opencv / 外部算法节点自定义名
float64 process_ms          # 本次识别耗时（毫秒）

geometry_msgs/Pose vehicle_pose   # 车辆位置（透传自 PlateCapture）
float64 speed                     # 车辆速度 m/s（透传自 PlateCapture）

string truth_number         # 该车的真值号码（来自 /plate/number），无真值时为空
bool matched                # plate_number 与 truth_number 是否逐字一致（识别率统计用）
