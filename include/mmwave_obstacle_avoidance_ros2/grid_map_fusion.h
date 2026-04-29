#ifndef MMWAVE_OBSTACLE_AVOIDANCE_ROS2_GRID_MAP_FUSION_H
#define MMWAVE_OBSTACLE_AVOIDANCE_ROS2_GRID_MAP_FUSION_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <Eigen/Dense>
#include "spatial_clustering.h"

namespace mmwave_obstacle_avoidance_ros2 {

/**
 * @brief 栅格单元结构
 */
struct GridCell {
    float occupancy_prob;      // 占据概率 [0,1]
    int update_count;          // 更新次数
    rclcpp::Time last_update;     // 最后更新时间
    bool is_instant;
};

/**
 * @brief L3 局部栅格地图融合类
 * 功能：时序融合 + 概率更新
 */
class GridMapFusion {
public:
    /**
     * @brief 构造函数
     */
    GridMapFusion();

    /**
     * @brief 初始化栅格地图
     * @param resolution 栅格分辨率(m)
     * @param width 地图宽度(m)
     * @param height 地图高度(m)
     */
    void initialize(float resolution, float width, float height);

    /**
     * @brief 设置融合阈值
     * @param threshold 占据概率阈值
     */
    void setOccupancyThreshold(float threshold);

    /**
     * @brief 设置时序衰减参数
     * @param decay_rate 衰减率 [0,1]
     * @param decay_time 衰减时间(s)
     */
    void setTemporalDecay(float decay_rate, float decay_time);

    void clearInstantMask();
    
    /**
     * @brief 更新栅格地图
     * @param clusters 聚类结果
     * @param current_time 当前时间
     */
    void updateGridMap(const std::vector<ClusterResult>& clusters, const rclcpp::Time& current_time);
    void markInstantOccupancy(const std::vector<ClusterResult>& instant_clusters, const rclcpp::Time& current_time);

    /**
     * @brief 获取栅格地图消息
     */
    nav_msgs::msg::OccupancyGrid getOccupancyGridMsg(const std::string& frame_id) const;

    /**
     * @brief 检查指定位置是否有障碍物
     * @param x, y 位置坐标(m)
     * @return 是否占据
     */
    bool isOccupied(float x, float y) const;

    /**
     * @brief 获取指定半径内的障碍物位置
     * @param radius 半径(m)
     * @param obstacles 输出障碍物位置
     */
    void getObstaclesInRadius(float radius, std::vector<Eigen::Vector2f>& obstacles) const;

    /**
     * @brief 清空地图
     */
    void clear();

    /**
     * @brief 获取统计信息
     */
    void getStatistics(int& total_cells, int& occupied_cells, float& avg_occupancy) const;
    const std::vector<GridCell>& getGrid() const { return grid_; }
    int getHeightCells() const { return height_cells_; }
    float getResolution() const { return resolution_; }
    float getWidthMeters() const { return width_meters_; }
    float getHeightMeters() const { return height_meters_; }
    float getOccupancyThreshold() const { return occupancy_threshold_; }
private:
    float resolution_;           // 栅格分辨率
    int width_cells_;           // 栅格宽度(格数)
    int height_cells_;          // 栅格高度(格数)
    float width_meters_;        // 地图宽度(m)
    float height_meters_;       // 地图高度(m)
    
    float occupancy_threshold_; // 占据阈值
    float decay_rate_;          // 衰减率
    float decay_time_;          // 衰减时间
    
    std::vector<GridCell> grid_; // 栅格数据

    // Bayesian更新参数
    const float P_OCCUPIED_GIVEN_DETECTED = 0.9f;   // 检测到时占据概率
    const float P_FREE_GIVEN_NOT_DETECTED = 0.7f;   // 未检测到时空闲概率

    /**
     * @brief 世界坐标转栅格索引
     */
    bool worldToGrid(float x, float y, int& grid_x, int& grid_y) const;

    /**
     * @brief 栅格索引转世界坐标
     */
    void gridToWorld(int grid_x, int grid_y, float& x, float& y) const;

    /**
     * @brief 获取栅格索引
     */
    int getGridIndex(int grid_x, int grid_y) const;

    /**
     * @brief Bayesian概率更新
     */
    float updateOccupancyProb(float prior_prob, bool detected) const;

    /**
     * @brief 时序衰减
     */
    void applyTemporalDecay(const rclcpp::Time& current_time);

    /**
     * @brief 标记聚类占据的栅格
     */
    void markClusterOccupancy(const ClusterResult& cluster, const rclcpp::Time& current_time);
};

} // namespace mmwave_obstacle_avoidance_ros2

#endif // MMWAVE_OBSTACLE_AVOIDANCE_ROS2_GRID_MAP_FUSION_H
