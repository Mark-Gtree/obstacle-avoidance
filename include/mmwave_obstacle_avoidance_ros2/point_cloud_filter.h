#ifndef MMWAVE_OBSTACLE_AVOIDANCE_ROS2_POINT_CLOUD_FILTER_H
#define MMWAVE_OBSTACLE_AVOIDANCE_ROS2_POINT_CLOUD_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

namespace mmwave_obstacle_avoidance_ros2 {

/**
 * @brief 带功率的点云类型
 */
struct alignas(16) PointXYZP {
    PCL_ADD_POINT4D;
    float power;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace mmwave_obstacle_avoidance_ros2

POINT_CLOUD_REGISTER_POINT_STRUCT(mmwave_obstacle_avoidance_ros2::PointXYZP,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, power, power))

namespace mmwave_obstacle_avoidance_ros2 {

/**
 * @brief L1 点云预处理类
 * 功能：功率过滤 + ROI区域过滤
 */
class PointCloudFilter {
public:
    /**
     * @brief 构造函数
     */
    PointCloudFilter();

    /**
     * @brief 设置功率阈值
     * @param min_power 最小功率阈值
     */
    void setPowerThreshold(float min_power);

    /*
     * @brief 设置高功率参数
     * @param high_power_threshold 高功率阈值
     * @param multiplier 对比度因子
     */
    void setHighPowerParams(float high_power_threshold, int multiplier);

    /**
     * @brief 设置ROI参数
     * @param min_range 最小距离(m)
     * @param max_range 最大距离(m)
     * @param max_fov_deg 最大水平FOV角度(度)
     * @param min_z Z轴最小值(m)
     * @param max_z Z轴最大值(m)
     * @param min_x X轴最小值(m)
     * @param max_x X轴最大值(m)
     * @param min_elevation_deg 最小俯仰角(度)
     * @param max_elevation_deg 最大俯仰角(度)
     */
    void setROI(float min_range, float max_range, float min_x, float max_x, 
                float max_fov_deg, float min_z, float max_z, 
                float min_elevation_deg, float max_elevation_deg);
    /**
     * @brief 执行点云过滤
     * @param input 输入点云
     * @param output 输出过滤后点云
     * @return 过滤后的点数
     */
    int filterPointCloud(const pcl::PointCloud<PointXYZP>::Ptr& input,
                         pcl::PointCloud<PointXYZP>::Ptr& output);

    /**
     * @brief 获取统计信息
     */
    void getStatistics(int& total_points, int& power_filtered, int& roi_filtered) const;

private:
    float min_power_;          // 最小功率阈值
    float high_power_threshold_; // 高功率阈值
    int high_power_multiplier_;  // 复制数量
    
    float min_range_;            // 最小距离
    float max_range_;          // 最大检测距离
    float max_fov_rad_;        // 最大FOV弧度
    
    float min_x_;
    float max_x_;
    float min_z_;
    float max_z_;
    
    float min_elevation_rad_;
    float max_elevation_rad_;
    float min_elevation_deg_;    // 最小俯仰角
    float max_elevation_deg_;    // 最大俯仰角
    
    // 统计信息
    int total_points_;
    int power_filtered_count_;
    int roi_filtered_count_;
    mutable rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    /**
     * @brief 功率过滤
     */
    void filterByPower(const pcl::PointCloud<PointXYZP>::Ptr& input,
                       pcl::PointCloud<PointXYZP>::Ptr& output);

    /**
     * @brief ROI区域过滤
     */
    void filterByROI(const pcl::PointCloud<PointXYZP>::Ptr& input,
                     pcl::PointCloud<PointXYZP>::Ptr& output);

    /**
     * @brief 检查点是否在ROI内
     */
    bool isPointInROI(const PointXYZP& point) const;
};

} // namespace mmwave_obstacle_avoidance_ros2

#endif // MMWAVE_OBSTACLE_AVOIDANCE_ROS2_POINT_CLOUD_FILTER_H
