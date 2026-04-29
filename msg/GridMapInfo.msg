# 栅格地图信息消息
std_msgs/Header header

float32 resolution            # 栅格分辨率(m)
uint32 width                  # 宽度（栅格数）
uint32 height                 # 高度（栅格数）
geometry_msgs/Pose origin     # 原点位置
float32[] data                # 占据概率数据 [0,1]
uint32 occupied_cells         # 占据格子数
