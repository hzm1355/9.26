# 超速告警（协议 §三.(四).3）：瞬时或区间任一判定超限即发 /speed/alert，界面侧据此打告警标记。
std_msgs/Header header

uint8 KIND_INSTANT=0        # 定点瞬时测速超限
uint8 KIND_SEGMENT=1        # 区间平均测速超限
uint8 kind

string ref_id               # 触发本告警的 SpeedCapture.capture_id 或 SpeedSegment.segment_id
string zone_id
string vehicle_id
string plate_number

float64 speed_kmh           # 判定速度（瞬时读数或区间均速）
float64 limit_kmh
float64 over_kmh            # 超限量
float64 ts                  # 超速时刻（仿真秒）

geometry_msgs/Pose pose     # 超速位置：瞬时=触发点，区间=离开断面
string note                 # 备注（如"中途急刹""反向通过已剔除"）
