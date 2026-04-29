#ifndef MMWAVE_OBSTACLE_AVOIDANCE_ROS2_CAN_PROTOCOL_H
#define MMWAVE_OBSTACLE_AVOIDANCE_ROS2_CAN_PROTOCOL_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <memory>

// Linux 原生 SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

namespace mmwave_obstacle_avoidance_ros2 {
// 自定义雷达点结构
struct RadarPoint {
    float x;          // 横向距离(m), 右正
    float y;          // 纵向距离(m), 前正
    float z;          // 垂向距离(m), 下正
    float power;      // 功率/信噪比
    uint64_t timestamp;
};

class CANProtocol {
public:
    explicit CANProtocol(const std::shared_ptr<rclcpp::Node>& node);
    ~CANProtocol();

    // 初始化雷达CAN接口
    bool initialize(const std::string& interface);
    
    // 接收雷达点云数据
    bool receiveRadarData(std::vector<RadarPoint>& points, int timeout_ms = 100);
    
    void close();
    bool isConnected() const;

private:
    std::shared_ptr<rclcpp::Node> node_;
    bool initialized_;
    int socket_fd_;
    std::string socket_interface_;

    static const uint32_t RADAR_MSG_ID = 0x60B;  // 雷达数据帧ID

    bool parseCANFrame(const uint8_t* data, RadarPoint& point);
};

sensor_msgs::msg::PointCloud2 radarPointsToROSMsg(
    const std::vector<RadarPoint>& points,
    const std::string& frame_id,
    const rclcpp::Time& timestamp);

} // namespace mmwave_obstacle_avoidance_ros2

#endif // MMWAVE_OBSTACLE_AVOIDANCE_ROS2_CAN_PROTOCOL_H