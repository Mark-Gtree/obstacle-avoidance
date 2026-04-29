# 障碍物告警消息
std_msgs/Header header

bool obstacle_detected              # 是否检测到障碍物
float32 min_distance                # 最近障碍物距离(m)
geometry_msgs/Point closest_point   # 最近障碍物位置
uint8 warning_level                 # 告警等级: 0-无 1-警告 2-危险 3-紧急
uint32 obstacle_count               # 检测到的障碍物数量
uint8 sensor_status                 # 雷达状态: 0-正常 1-故障 2-无数据