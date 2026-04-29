#include "mmwave_obstacle_avoidance_ros2/grid_map_fusion.h"
#include <cmath>

namespace mmwave_obstacle_avoidance_ros2 {

GridMapFusion::GridMapFusion()
    : resolution_(0.2f),
      width_cells_(0),
      height_cells_(0),
      width_meters_(30.0f),
      height_meters_(30.0f),
      occupancy_threshold_(0.8f),
      decay_rate_(0.95f),
      decay_time_(1.0f) {}

void GridMapFusion::initialize(float resolution, float width, float height) {
    resolution_ = resolution;
    width_meters_ = width;
    height_meters_ = height;

    width_cells_ = static_cast<int>(std::ceil(width / resolution));
    height_cells_ = static_cast<int>(std::ceil(height / resolution));

    // 初始化栅格
    grid_.resize(width_cells_ * height_cells_);
    for (auto& cell : grid_) {
        cell.occupancy_prob = 0.5f;  // 初始不确定状态
        cell.update_count = 0;
        cell.last_update = rclcpp::Time(0, 0, RCL_ROS_TIME);
        cell.is_instant = false;
    }

    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Grid map initialized: %dx%d cells, resolution=%.2fm",
             width_cells_, height_cells_, resolution_);
}

void GridMapFusion::setOccupancyThreshold(float threshold) {
    occupancy_threshold_ = threshold;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Occupancy threshold set to: %.2f", threshold);
}

void GridMapFusion::setTemporalDecay(float decay_rate, float decay_time) {
    decay_rate_ = decay_rate;
    decay_time_ = decay_time;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Temporal decay: rate=%.3f, time=%.1fs", decay_rate, decay_time);
}

void GridMapFusion::clearInstantMask() {
    for (auto& cell : grid_) cell.is_instant = false;
}

void GridMapFusion::applyTemporalDecay(const rclcpp::Time& current_time) {
    for (auto& cell : grid_) {
        
        if (cell.is_instant) {
            cell.is_instant = false;
        }

        if (cell.update_count > 0) {
            double time_since_update = (current_time - cell.last_update).seconds();
            
            if (time_since_update > decay_time_) {
                float decay_factor = std::pow(decay_rate_, 
                                              time_since_update / decay_time_);
                cell.occupancy_prob = 0.5f + (cell.occupancy_prob - 0.5f) * decay_factor;
                cell.last_update = current_time;
            }
        }
    }
}
void GridMapFusion::updateGridMap(const std::vector<ClusterResult>& clusters,
                                   const rclcpp::Time& current_time) {
    applyTemporalDecay(current_time);
    for (const auto& cluster : clusters) {
        markClusterOccupancy(cluster, current_time);  
    }
}

float GridMapFusion::updateOccupancyProb(float prior_prob, bool detected) const {
    // Bayesian概率更新
    // 使用贝叶斯概率更新，结合先验概率（历史经验prior_prob）和当前检测结果计算后验概率
    // P(occupied|detected) = P(detected|occupied) * P(occupied) / P(detected)
    
    float p_occupied, p_detected;
    
    if (detected) {
        p_occupied = P_OCCUPIED_GIVEN_DETECTED;
        p_detected = P_OCCUPIED_GIVEN_DETECTED * prior_prob + 
                     (1 - P_OCCUPIED_GIVEN_DETECTED) * (1 - prior_prob);
    } else {
        p_occupied = 1 - P_FREE_GIVEN_NOT_DETECTED;
        p_detected = (1 - P_FREE_GIVEN_NOT_DETECTED) * prior_prob + 
                     P_FREE_GIVEN_NOT_DETECTED * (1 - prior_prob);
    }
    
    float posterior = (p_occupied * prior_prob) / p_detected;
    
    // 限制在[0.01, 0.99]范围内避免极端值
    // 避免概率变为0/1，导致死锁
    return std::max(0.01f, std::min(0.99f, posterior));
}

// 根据新看到的聚类更新概率
void GridMapFusion::markClusterOccupancy(const ClusterResult& cluster,
                                          const rclcpp::Time& current_time) {
    int min_x, min_y, max_x, max_y;
    worldToGrid(cluster.min_bound.x(), cluster.min_bound.y(), min_x, min_y);
    worldToGrid(cluster.max_bound.x(), cluster.max_bound.y(), max_x, max_y);

    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    max_x = std::min(width_cells_ - 1, max_x);
    max_y = std::min(height_cells_ - 1, max_y);

    const int grid_size = static_cast<int>(grid_.size());
    for (int gx = min_x; gx <= max_x; ++gx) {
        for (int gy = min_y; gy <= max_y; ++gy) {
            int idx = getGridIndex(gx, gy);
            if (idx >= 0 && idx < grid_size) {
                auto& cell = grid_[idx];
                cell.occupancy_prob = updateOccupancyProb(cell.occupancy_prob, true);
                cell.update_count++;
                cell.last_update = current_time; 
            }
        }
    }
}

void GridMapFusion::markInstantOccupancy(const std::vector<ClusterResult>& instant_clusters, const rclcpp::Time& current_time) {
    for (const auto& cluster : instant_clusters) {
        int min_x, min_y, max_x, max_y;
        worldToGrid(cluster.min_bound.x(), cluster.min_bound.y(), min_x, min_y);
        worldToGrid(cluster.max_bound.x(), cluster.max_bound.y(), max_x, max_y);

        min_x = std::max(0, min_x);
        min_y = std::max(0, min_y);
        max_x = std::min(width_cells_ - 1, max_x);
        max_y = std::min(height_cells_ - 1, max_y);

        const int grid_size = static_cast<int>(grid_.size());
        for (int gx = min_x; gx <= max_x; ++gx) {
            for (int gy = min_y; gy <= max_y; ++gy) {
                int idx = getGridIndex(gx, gy);
                if (idx >= 0 && idx < grid_size) {
                    auto& cell = grid_[idx];
                    cell.is_instant = true; 
                    cell.last_update = current_time; 
                }
            }
        }
    }
}

bool GridMapFusion::isOccupied(float x, float y) const {
     int gx, gy;
    if (!worldToGrid(x, y, gx, gy)) return false;
    int idx = getGridIndex(gx, gy);
    if (idx < 0 || idx >= static_cast<int>(grid_.size())) return false;
    
    return grid_[idx].occupancy_prob >= occupancy_threshold_ || grid_[idx].is_instant;
}

// 获取指定半径内的障碍物位置
void GridMapFusion::getObstaclesInRadius(float radius,
                                          std::vector<Eigen::Vector2f>& obstacles) const {
    obstacles.clear();

    const int radius_cells = static_cast<int>(std::ceil(radius / resolution_));
    const int center_x = width_cells_ / 2;
    const int center_y = 0;
    const float radius_sq = radius * radius;  

    for (int gx = std::max(0, center_x - radius_cells);
         gx <= std::min(width_cells_ - 1, center_x + radius_cells); ++gx) {
        for (int gy = std::max(0, center_y - radius_cells);
             gy <= std::min(height_cells_ - 1, center_y + radius_cells); ++gy) {

            int idx = getGridIndex(gx, gy);
            if (grid_[idx].occupancy_prob >= occupancy_threshold_ || grid_[idx].is_instant) {
                float x, y;
                gridToWorld(gx, gy, x, y);
                if (x * x + y * y <= radius_sq) {
                    obstacles.emplace_back(x, y); 
                }
            }
        }
    }
}

nav_msgs::msg::OccupancyGrid GridMapFusion::getOccupancyGridMsg(const std::string& frame_id) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Clock().now();
    msg.info.resolution = resolution_;
    msg.info.width = width_cells_;
    msg.info.height = height_cells_;
    msg.info.origin.position.x = -width_meters_ / 2.0f;
    msg.info.origin.position.y = 0.0f;
    msg.info.origin.position.z = 0.0f;
    msg.info.origin.orientation.w = 1.0f;

    msg.data.resize(grid_.size());
    for (size_t i = 0; i < grid_.size(); ++i) {
        if (grid_[i].is_instant) {
            msg.data[i] = 100;
        } else {
            msg.data[i] = static_cast<int8_t>(grid_[i].occupancy_prob * 100.0f);
        }
    }
    return msg;
}

void GridMapFusion::clear() {
    for (auto& cell : grid_) {
        cell.occupancy_prob = 0.5f;
        cell.update_count = 0;
        cell.last_update = rclcpp::Time(0, 0, RCL_ROS_TIME);
        cell.is_instant = false;
    }
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Grid map cleared");
}

bool GridMapFusion::worldToGrid(float x, float y, int& grid_x, int& grid_y) const {
    float local_x = x + width_meters_ / 2.0f;
    
    float local_y = y; 

    grid_x = static_cast<int>(std::floor(local_x / resolution_));
    grid_y = static_cast<int>(std::floor(local_y / resolution_));

    return (grid_x >= 0 && grid_x < width_cells_ &&
            grid_y >= 0 && grid_y < height_cells_);
}

void GridMapFusion::gridToWorld(int grid_x, int grid_y, float& x, float& y) const {
    x = (grid_x + 0.5f) * resolution_ - width_meters_ / 2.0f;
    y = (grid_y + 0.5f) * resolution_; 
}

int GridMapFusion::getGridIndex(int grid_x, int grid_y) const {
    return grid_y * width_cells_ + grid_x;
}

void GridMapFusion::getStatistics(int& total_cells, int& occupied_cells, float& avg_occupancy) const {
    total_cells = grid_.size();
    occupied_cells = 0;
    float sum = 0.0f;
    for (const auto& cell : grid_) {
        if (cell.is_instant || cell.occupancy_prob >= occupancy_threshold_) {
            occupied_cells++;
        }
        sum += cell.occupancy_prob;
    }
    avg_occupancy = total_cells > 0 ? sum / total_cells : 0.0f;
}

} // namespace mmwave_obstacle_avoidance_ros2
