#include "mmwave_obstacle_avoidance_ros2/can_protocol.h"
#include "mmwave_obstacle_avoidance_ros2/point_cloud_filter.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>

namespace mmwave_obstacle_avoidance_ros2 {

CANProtocol::CANProtocol(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node), initialized_(false), socket_fd_(-1) {}

CANProtocol::~CANProtocol() { close(); }

bool CANProtocol::initialize(const std::string& interface) {

    if (socket_fd_ >= 0) { ::close(socket_fd_); socket_fd_ = -1; }
    
    socket_interface_ = interface;
    
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create SocketCAN: %s", strerror(errno));
        return false;
    }
    
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get interface %s: %s", interface.c_str(), strerror(errno));
        ::close(socket_fd_); socket_fd_ = -1;
        return false;
    }
    
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to bind SocketCAN: %s", strerror(errno));
        ::close(socket_fd_); socket_fd_ = -1;
        return false;
    }
    
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "SocketCAN initialized on %s", interface.c_str());
    return true;
}

bool CANProtocol::receiveRadarData(std::vector<RadarPoint>& points, int timeout_ms) {
    if (!initialized_) return false;
    points.clear();
    points.reserve(200);

    struct can_frame frame;
    fd_set readfds;
    
    struct timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (timeout_ms % 1000) * 1000000LL;
    if (deadline.tv_nsec >= 1000000000LL) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000LL;
    }

    const uint64_t batch_ts = node_->now().nanoseconds();

    while (true) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long remain_ns = (deadline.tv_sec - now.tv_sec) * 1000000000LL
                       + (deadline.tv_nsec - now.tv_nsec);
        if (remain_ns <= 0) break;

        struct timeval tv;
        tv.tv_sec  = remain_ns / 1000000000LL;
        tv.tv_usec = (remain_ns % 1000000000LL) / 1000;

        FD_ZERO(&readfds);
        FD_SET(socket_fd_, &readfds);
        int ret = select(socket_fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ret <= 0) break;

        int nbytes = read(socket_fd_, &frame, sizeof(frame));
        if (nbytes < 0) break;

        if (frame.can_id == RADAR_MSG_ID && frame.can_dlc >= 8) {
            RadarPoint point;
            point.timestamp = batch_ts;
            if (parseCANFrame(frame.data, point)) {
                points.push_back(point);
            }
        }
    }

    return !points.empty();
}

bool CANProtocol::parseCANFrame(const uint8_t* data, RadarPoint& point) {
    const int16_t raw_dist = (int16_t)((data[2] << 8) | data[1]);
    const int16_t raw_az   = (int16_t)((data[4] << 8) | data[3]);
    const int16_t raw_el   = (int16_t)((data[6] << 8) | data[5]);
    const int8_t  raw_pow  = (int8_t)data[7];

    constexpr float DEG_TO_RAD_SCALE = 0.01f * (M_PI / 180.0f);
    const float range  = raw_dist * 0.01f;
    const float az_rad = raw_az * DEG_TO_RAD_SCALE;
    const float el_rad = raw_el * DEG_TO_RAD_SCALE;

    point.power = static_cast<float>(raw_pow);

    const float cos_el = std::cos(el_rad);
    point.y = range * cos_el * std::cos(az_rad);
    point.x = range * cos_el * std::sin(az_rad);
    point.z = -range * std::sin(el_rad);

    return true;
}

void CANProtocol::close() {
    if (initialized_ && socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        RCLCPP_INFO(node_->get_logger(), "SocketCAN closed");
    }
    initialized_ = false;
}

bool CANProtocol::isConnected() const { return initialized_; }

sensor_msgs::msg::PointCloud2 radarPointsToROSMsg(
    const std::vector<RadarPoint>& points,
    const std::string& frame_id,
    const rclcpp::Time& timestamp)
{
    pcl::PointCloud<PointXYZP> cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = timestamp.nanoseconds() / 1000;
    cloud.points.reserve(points.size());  
    for (const auto& pt : points) {
        PointXYZP p; p.x = pt.x; p.y = pt.y; p.z = pt.z; p.power = pt.power;
        cloud.points.push_back(p);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = timestamp;
    return msg;
}


} // namespace mmwave_obstacle_avoidance_ros2