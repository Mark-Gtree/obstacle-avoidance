#include "mmwave_obstacle_avoidance_ros2/point_cloud_filter.h"
#include <cmath>
#include <cstdlib>

namespace mmwave_obstacle_avoidance_ros2 {

PointCloudFilter::PointCloudFilter()
    : min_power_(15.0f),
      high_power_threshold_(60.0f),
      high_power_multiplier_(5),
      min_range_(3.0f),
      max_range_(20.0f),
      max_fov_rad_(M_PI / 9.0),
      min_x_(-3.0f),
      max_x_(3.0f),
      min_z_(-1.0f),
      max_z_(2.0f),
      min_elevation_rad_(-15.0f * M_PI / 180.0f),
      max_elevation_rad_(15.0f * M_PI / 180.0f),
      total_points_(0),
      power_filtered_count_(0),
      roi_filtered_count_(0) {}

void PointCloudFilter::setPowerThreshold(float min_power) {
    min_power_ = min_power;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Power threshold set to: %.1f", min_power_);
}

void PointCloudFilter::setHighPowerParams(float high_power_threshold, int multiplier) {
    high_power_threshold_ = high_power_threshold;
    high_power_multiplier_ = multiplier;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), 
                "High power enhancement: threshold=%.1f, multiplier=%d", 
                high_power_threshold_, high_power_multiplier_);
}

void PointCloudFilter::setROI(float min_range, float max_range, float min_x, float max_x, 
                              float max_fov_deg, float min_z, float max_z, 
                              float min_elevation_deg, float max_elevation_deg) {
    min_range_ = min_range;
    max_range_ = max_range;
    min_x_ = min_x;
    max_x_ = max_x;
    max_fov_rad_ = max_fov_deg * M_PI / 180.0f;
    min_z_ = min_z;
    max_z_ = max_z;
    min_elevation_rad_ = min_elevation_deg * M_PI / 180.0f;
    max_elevation_rad_ = max_elevation_deg * M_PI / 180.0f;
    
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), 
                "ROI set: Y[%.1f, %.1f]m, X[%.1f, %.1f]m, Z[%.1f, %.1f]m, H-FOV: ±%.1f°, V-FOV[%.1f°, %.1f°]", 
                min_range_, max_range_, min_x_, max_x_, min_z_, max_z_, 
                max_fov_deg, min_elevation_deg, max_elevation_deg);
}

// 主过滤函数，返回过滤后的点云数量
int PointCloudFilter::filterPointCloud(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                        pcl::PointCloud<PointXYZP>::Ptr& output) {
    if (!input || input->points.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("obstacle_avoidance"), "Input point cloud is empty");
        output->clear();
        return 0;
    }

    total_points_ = static_cast<int>(input->points.size());
    power_filtered_count_ = 0;
    roi_filtered_count_ = 0;

    output->clear();
    output->header = input->header;
    output->points.reserve(input->points.size() * (high_power_multiplier_ + 1));

    // 动态极坐标分层偏移：将 multiplier 个复制点均匀分布在 XY 平面的圆环上，
    // 再按层数分配 Z 方向正负交替偏移，支持任意 multiplier 值。
    // 第 i 个复制点：
    //   angle   = 2π * i / multiplier          （XY 平面均匀分布）
    //   ring    = i / base_count + 1           （每 base_count 个点扩大一圈半径）
    //   offset_x = ring * R_STEP * cos(angle)
    //   offset_y = ring * Y_JITTER * sin(angle)（纵向微扰，保持前向为主）
    //   offset_z = (i % 2 == 0 ? 1 : -1) * ring * Z_STEP
    static constexpr float R_STEP   = 0.08f;   // 每圈 X 半径步进 (m)
    static constexpr float Y_JITTER = 0.003f;  // Y 轴最大抖动幅度 (m)
    static constexpr float Z_STEP   = 0.001f;  // 每圈 Z 步进 (m)
    static constexpr float TWO_PI   = 2.0f * static_cast<float>(M_PI);

    for (const auto& point : input->points) {
        // 功率门限先过
        if (point.power < min_power_) {
            power_filtered_count_++;
            continue;
        }
        // ROI 检查
        if (!isPointInROI(point)) {
            roi_filtered_count_++;
            continue;
        }
        output->points.push_back(point);
        if (point.power >= high_power_threshold_) {
            //RCLCPP_INFO_THROTTLE(rclcpp::get_logger("obstacle_avoidance"),
            //                     steady_clock_, 1000,
             //                    "！！高功率！！ 当前点功率: %.1f", point.power);

            const int base_count = std::max(1, high_power_multiplier_ / 3 + 1);
            for (int i = 0; i < high_power_multiplier_; ++i) {
                const float angle   = TWO_PI * i / high_power_multiplier_;
                const int   ring    = i / base_count + 1;
                const float r       = ring * R_STEP;
                PointXYZP dup = point;
                dup.x += r * std::cos(angle);
                dup.y += Y_JITTER * std::sin(angle);
                dup.z += (i % 2 == 0 ? 1.0f : -1.0f) * ring * Z_STEP;
                output->points.push_back(dup);
            }
        }
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    return static_cast<int>(output->points.size());
}



void PointCloudFilter::filterByPower(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                      pcl::PointCloud<PointXYZP>::Ptr& output) {
    output->clear();
    output->header = input->header;
    output->points.reserve(input->points.size());

    for (const auto& point : input->points) {
        if (point.power >= min_power_) {
            output->points.push_back(point);
        } else {
            power_filtered_count_++;
        }
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
}

void PointCloudFilter::filterByROI(const pcl::PointCloud<PointXYZP>::Ptr& input,
                                    pcl::PointCloud<PointXYZP>::Ptr& output) {
    output->clear();
    output->header = input->header;
    output->points.reserve(input->points.size() * (high_power_multiplier_ + 1));

    static const float EXPAND_X[] = {-0.15f, 0.15f, -0.08f, 0.08f, -0.15f, 0.15f};
    static const float NOISE_Y[]  = {0.002f, 0.001f, -0.001f, -0.002f, 0.003f, -0.001f};
    static const float NOISE_Z[]  = {-0.001f, 0.002f, -0.002f, 0.001f, -0.003f, 0.002f};
    
    for (const auto& point : input->points) {
        if (isPointInROI(point)) {
            output->points.push_back(point);
            
            if (point.power >= high_power_threshold_) {
               for (int i = 0; i < high_power_multiplier_; ++i) {
                    PointXYZP dup_pt = point;
                    dup_pt.x += EXPAND_X[i % 6];
                    dup_pt.y += NOISE_Y[i % 6];
                    dup_pt.z += NOISE_Z[i % 6];
                    output->points.push_back(dup_pt);
                }
            }
        } else {
            roi_filtered_count_++;
        }
    }
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
}

bool PointCloudFilter::isPointInROI(const PointXYZP& point) const {
    // 1. Z 轴
    if (point.z < min_z_ || point.z > max_z_) return false;
    // 2. X 轴
    if (point.x < min_x_ || point.x > max_x_) return false;
    // 3. Y 轴最小距离
    if (point.y < min_range_) return false;

    // 4. 水平距离平方判断
    const float hd2 = point.x * point.x + point.y * point.y;
    const float max_range_sq = max_range_ * max_range_;
    if (hd2 > max_range_sq || hd2 < 1e-4f) return false; // 防除零

    // 5. 水平 FOV
    if (std::atan2(std::abs(point.x), point.y) > max_fov_rad_) return false;

    // 6. 垂直俯仰角
    const float horizontal_dist = std::sqrt(hd2);
    const float elevation_angle = std::atan2(point.z, horizontal_dist);
    if (elevation_angle < min_elevation_rad_ || elevation_angle > max_elevation_rad_) return false;

    return true;
}


void PointCloudFilter::getStatistics(int& total_points, 
                                      int& power_filtered, 
                                      int& roi_filtered) const {
    total_points = total_points_;
    power_filtered = power_filtered_count_;
    roi_filtered = roi_filtered_count_;
}

} // namespace mmwave_obstacle_avoidance_ros2
