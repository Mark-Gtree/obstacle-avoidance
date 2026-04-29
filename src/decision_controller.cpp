#include "mmwave_obstacle_avoidance_ros2/decision_controller.h"
#include <cmath>
#include <limits>

namespace mmwave_obstacle_avoidance_ros2 {

DecisionController::DecisionController()
    : emergency_distance_(2.0f),
      danger_distance_(2.5f),
      caution_distance_(5.0f),
      min_continuous_frames_(3),
      warning_timeout_(0.5f),
      continuous_warning_count_(0),
      last_brake_state_(false),
      total_decisions_(0),
      brake_commands_(0),
      false_alarms_(0),
      total_min_distance_(0.0f) {}

void DecisionController::setDistanceThresholds(float emergency_dist,
                                                float danger_dist,
                                                float caution_dist) {
    emergency_distance_ = emergency_dist;
    danger_distance_ = danger_dist;
    caution_distance_ = caution_dist;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Distance thresholds: emergency=%.1fm, danger=%.1fm, caution=%.1fm",
             emergency_dist, danger_dist, caution_dist);
}

void DecisionController::setContinuousWarningParams(int min_continuous_frames,
                                                     float timeout) {
    min_continuous_frames_ = min_continuous_frames;
    warning_timeout_ = timeout;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Continuous warning: min_frames=%d, timeout=%.1fs",
             min_continuous_frames, timeout);
}

bool DecisionController::makeDecision(const GridMapFusion& grid_map,
                                       const rclcpp::Time& current_time,
                                       DecisionResult& result) {
    total_decisions_++;

    std::vector<Eigen::Vector2f> obstacles;
    grid_map.getObstaclesInRadius(caution_distance_, obstacles);

    float min_distance = 9999.0f;
    Eigen::Vector2f closest_point = Eigen::Vector2f::Zero();
    for (const auto& obs : obstacles) {
        float dist = obs.norm();
        if (dist < min_distance) {
            min_distance = dist;
            closest_point = obs;
        }
    }
    int prev_count = continuous_warning_count_;
    result.min_distance    = min_distance;
    result.closest_point   = closest_point;
    result.obstacle_count  = static_cast<int>(obstacles.size());
    result.obstacle_detected = !obstacles.empty();
    result.warning_level   = calculateWarningLevel(min_distance);
    result.brake_required  = shouldBrake(result.warning_level, current_time);
    if (result.obstacle_detected) {
    total_min_distance_ += min_distance;
    }
    if (result.brake_required) brake_commands_++;

    if (last_brake_state_ && !result.brake_required &&
        prev_count  >= min_continuous_frames_) {
        false_alarms_++;
    }
    last_brake_state_ = result.brake_required;

    if (result.brake_required) {
        RCLCPP_ERROR(rclcpp::get_logger("obstacle_avoidance"),
            "!!! BRAKE COMMAND SENT !!! Distance: %.2fm | Level: %d",
            result.min_distance, static_cast<int>(result.warning_level));
    }
    return result.brake_required;
}


WarningLevel DecisionController::calculateWarningLevel(float min_distance) const {
    if (min_distance < emergency_distance_) {
        return WarningLevel::EMERGENCY;
    } else if (min_distance < danger_distance_) {
        return WarningLevel::DANGER;
    } else if (min_distance < caution_distance_) {
        return WarningLevel::CAUTION;
    } else {
        return WarningLevel::NONE;
    }
}

bool DecisionController::shouldBrake(WarningLevel current_warning,
                                      const rclcpp::Time& current_time) {
    // 检查告警超时
    if (last_warning_time_.nanoseconds() != 0) {
        double time_since_warning = (current_time - last_warning_time_).seconds();
        if (time_since_warning > warning_timeout_) {
            continuous_warning_count_ = 0;
        }
    }

    // 更新连续告警计数
    if (current_warning >= WarningLevel::DANGER) {
        continuous_warning_count_++;
        last_warning_time_ = current_time;
    } else {
        continuous_warning_count_ = 0;
    }

    // 紧急情况立即刹车
    if (current_warning == WarningLevel::EMERGENCY) {
        return true;
    }

    // 危险情况需要连续检测
    if (current_warning == WarningLevel::DANGER &&
        continuous_warning_count_ >= min_continuous_frames_) {
        return true;
    }

    return false;
}

void DecisionController::findClosestObstacle(const GridMapFusion& grid_map,
                                               float& min_distance,
                                               Eigen::Vector2f& closest_point) const {
    std::vector<Eigen::Vector2f> obstacles;
    grid_map.getObstaclesInRadius(caution_distance_, obstacles);

    min_distance = std::numeric_limits<float>::max();
    closest_point = Eigen::Vector2f::Zero();

    for (const auto& obs : obstacles) {
        float dist = obs.norm();
        if (dist < min_distance) {
            min_distance = dist;
            closest_point = obs;
        }
    }

    // 如果没有找到障碍物,设置为最大距离
    if (obstacles.empty()) {
        min_distance = /*caution_distance_*/9999.0f;
    }
}

void DecisionController::reset() {
    continuous_warning_count_ = 0;
    last_warning_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_brake_state_ = false;
    RCLCPP_INFO(rclcpp::get_logger("obstacle_avoidance"), "Decision controller reset");
}

void DecisionController::getStatistics(int& total_decisions,
                                        int& brake_commands,
                                        int& false_alarms,
                                        float& avg_min_distance) const {
    total_decisions = total_decisions_;
    brake_commands = brake_commands_;
    false_alarms = false_alarms_;
    avg_min_distance = (total_decisions_ > 0) ? 
                       (total_min_distance_ / total_decisions_) : 0.0f;
}

} // namespace mmwave_obstacle_avoidance_ros2
