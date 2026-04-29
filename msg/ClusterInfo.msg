# 聚类信息消息
std_msgs/Header header

uint32 cluster_id                   # 聚类ID
geometry_msgs/Point centroid        # 聚类中心
float32 volume                      # 聚类体积
uint32 point_count                  # 点数量
geometry_msgs/Point min_bound       # 最小边界
geometry_msgs/Point max_bound       # 最大边界
