#ifndef MMWAVE_OBSTACLE_AVOIDANCE_ROS2_DECISION_CONTROLLER_H
#define MMWAVE_OBSTACLE_AVOIDANCE_ROS2_DECISION_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <vector>
#include "grid_map_fusion.h"
#include "can_protocol.h"

namespace mmwave_obstacle_avoidance_ros2 {

/**
 * @brief 告警等级枚举
 */
enum class WarningLevel : uint8_t {
    NONE = 0,      // 无告警
    CAUTION = 1,   // 警告 
    DANGER = 2,    // 危险
    EMERGENCY = 3  // 紧急
};

/**
 * @brief 决策结果结构
 */
struct DecisionResult {
    bool brake_required;           // 是否需要刹车
    bool obstacle_detected;        // 是否检测到障碍物
    WarningLevel warning_level;    // 告警等级
    float min_distance;            // 最近障碍物距离
    Eigen::Vector2f closest_point; // 最近障碍物位置
    int obstacle_count;            // 障碍物数量
};

/**
 * @brief L4 决策控制类
 * 功能：障碍物判断 + CAN指令发送
 */
class DecisionController {
public:
    /**
     * @brief 构造函数
     */
    DecisionController();

    /**
     * @brief 设置避障距离阈值
     * @param emergency_dist 紧急距离(m)
     * @param danger_dist 危险距离(m)
     * @param caution_dist 警告距离(m)
     */
    void setDistanceThresholds(float emergency_dist, float danger_dist, float caution_dist);

    /**
     * @brief 设置持续告警参数
     * @param min_continuous_frames 最小连续帧数
     * @param timeout 告警超时时间(s)
     */
    void setContinuousWarningParams(int min_continuous_frames, float timeout);

    /**
     * @brief 执行决策
     * @param grid_map 栅格地图
     * @param current_time 当前时间
     * @param result 输出决策结果
     * @return 是否需要发送告警
     */
    bool makeDecision(const GridMapFusion& grid_map, 
                      const rclcpp::Time& current_time,
                      DecisionResult& result);

    
    /**
     * @brief 重置决策状态
     */
    void reset();

    /**
     * @brief 获取统计信息
     */
    void getStatistics(int& total_decisions, int& brake_commands, 
                      int& false_alarms, float& avg_min_distance) const;

private:
    // 距离阈值
    float emergency_distance_;  // 紧急刹车距离
    float danger_distance_;     // 危险距离
    float caution_distance_;    // 警告距离

    // 连续告警参数
    int min_continuous_frames_;
    float warning_timeout_;

    // 状态跟踪
    int continuous_warning_count_;
    rclcpp::Time last_warning_time_;
    bool last_brake_state_;

    // 统计信息
    int total_decisions_;
    int brake_commands_;
    int false_alarms_;
    float total_min_distance_;
    
    /**
     * @brief 计算告警等级
     */
    WarningLevel calculateWarningLevel(float min_distance) const;

    /**
     * @brief 检查是否需要刹车
     * @param current_warning 当前告警等级
     * @param current_time 当前时间
     * @return 是否需要刹车
     */
    bool shouldBrake(WarningLevel current_warning, const rclcpp::Time& current_time);

    /**
     * @brief 查找最近障碍物
     */
    void findClosestObstacle(const GridMapFusion& grid_map,
                             float& min_distance,
                             Eigen::Vector2f& closest_point) const;
};

} // namespace mmwave_obstacle_avoidance_ros2

#endif // MMWAVE_OBSTACLE_AVOIDANCE_ROS2_DECISION_CONTROLLER_H
