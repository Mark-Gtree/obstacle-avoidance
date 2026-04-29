#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include "mmwave_obstacle_avoidance_ros2/can_protocol.h"
#include "mmwave_obstacle_avoidance_ros2/point_cloud_filter.h"
#include "mmwave_obstacle_avoidance_ros2/spatial_clustering.h"
#include "mmwave_obstacle_avoidance_ros2/grid_map_fusion.h"
#include "mmwave_obstacle_avoidance_ros2/decision_controller.h"
#include "mmwave_obstacle_avoidance_ros2/msg/obstacle_warning.hpp"
#include <mavlink/v2.0/common/mavlink.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

using namespace mmwave_obstacle_avoidance_ros2;

 
static constexpr uint8_t PX4_MAIN_MODE_POSCTL        = 3;  
static constexpr uint8_t PX4_MAIN_MODE_AUTO          = 4;  
static constexpr uint8_t PX4_SUB_MODE_AUTO_LOITER    = 3;  
static constexpr uint8_t PX4_SUB_MODE_AUTO_MISSION   = 4;  

static constexpr uint8_t  ONBOARD_SYSID  = 1;   
static constexpr uint8_t  ONBOARD_COMPID = MAV_COMP_ID_ONBOARD_COMPUTER;// 191   

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node"), self_dev_fd_(-1), px4_fd_(-1) {
        declareParameters();
        loadParameters();

        raw_cloud_pub_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("mmwave/raw_pointcloud", 10);
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mmwave/filtered_pointcloud", 10);
        clustered_cloud_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>("mmwave/clustered_pointcloud", 10);
        grid_map_pub_       = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mmwave/grid_map", 10);
        warning_pub_        = this->create_publisher<mmwave_obstacle_avoidance_ros2::msg::ObstacleWarning>("mmwave/obstacle_warning", 1);
        markers_pub_        = this->create_publisher<visualization_msgs::msg::MarkerArray>("mmwave/cluster_markers", 10);

        tf_buffer_          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_        = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&ObstacleAvoidanceNode::delayedInitialization, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node created, waiting for initialization...");
    }

    ~ObstacleAvoidanceNode() {
        if (can_protocol_)  can_protocol_->close();
        if (self_dev_fd_ != -1) ::close(self_dev_fd_);
        if (px4_fd_ != -1)      ::close(px4_fd_);
        RCLCPP_INFO(this->get_logger(), "Serial ports closed.");
    }

private:
     enum class RadarStatus : uint8_t {
        UNINITIALIZED = 0,// 未初始化
        NORMAL        = 1,// 正常
        DISCONNECTED  = 2,// 连接断开
        HARDWARE_ERROR= 3 // 硬件故障
    };
    RadarStatus radar_status_ = RadarStatus::UNINITIALIZED;

    enum class AvoidState : uint8_t {
        IDLE     = 0,
        AVOIDING = 1,
    };
    AvoidState avoid_state_ = AvoidState::IDLE;

    uint8_t  px4_base_mode_        = 0;
    uint32_t px4_custom_mode_      = 0;
    bool     px4_heartbeat_recv_   = false;
    uint8_t  px4_target_sysid_     = 1;    
    uint8_t  px4_target_compid_    = 1; 
    uint8_t  px4_landed_state_     = 0;  

    bool         mode_switch_ack_pending_ = false;
    rclcpp::Time mode_switch_sent_time_{0, 0, RCL_ROS_TIME};
    static constexpr double ACK_TIMEOUT_S = 3.0;

    rclcpp::Time last_switch_cmd_time_{0, 0, RCL_ROS_TIME};
    bool         obstacle_was_clear_ = false;
    static constexpr double SWITCH_CMD_INTERVAL_S = 1.0; 
    rclcpp::Time obstacle_clear_time_{0, 0, RCL_ROS_TIME};
    static constexpr double AVOID_RESET_TIMEOUT_S = 3.0;

    float hold_trigger_distance_ = 5.0f;
    float obstacle_dist_hz_      = 10.0f;
    float latest_obstacle_dist_ = 999.0f;
    
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr algo_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr obs_dist_timer_;
    rclcpp::TimerBase::SharedPtr px4_recv_timer_;  

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr  grid_map_pub_;
    rclcpp::Publisher<mmwave_obstacle_avoidance_ros2::msg::ObstacleWarning>::SharedPtr warning_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    std::shared_ptr<CANProtocol>          can_protocol_;
    std::unique_ptr<PointCloudFilter>     point_filter_;
    std::unique_ptr<SpatialClustering>    spatial_clustering_;
    std::unique_ptr<GridMapFusion>        grid_map_fusion_;
    std::unique_ptr<DecisionController>   decision_controller_;

    std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;

    std::string radar_can_interface_;
    std::string frame_id_;
    double      processing_rate_;
    bool        enable_visualization_;
    int         self_dev_fd_;
    int         px4_fd_;
    std::string self_dev_port_;
    int         self_dev_baud_;
    std::string px4_port_;
    int         px4_baud_;
    float      base_mode_para_;
    float      main_mode_para_;
    float      sub_mode_para_;
    bool       enable_logging_;
    float current_relative_alt_m_ = 0.0f;
    float min_avoidance_alt_ = 1.5f;
    float direct_map_power_threshold_ = 60.0f;
    float merge_dist_ = 5.0f;
    mavlink_status_t mavlink_status_{};
    bool  enable_attitude_compensation_ = true;
    float attitude_timeout_ = 0.3f;
    float attitude_stream_hz_ = 20.0f;
    float pitch_compensation_sign_ = 1.0f;
    float roll_compensation_sign_ = 1.0f;
    float latest_roll_rad_ = 0.0f;
    float latest_pitch_rad_ = 0.0f;
    bool  attitude_received_ = false;
    bool  attitude_interval_requested_ = false;
    rclcpp::Time last_attitude_time_{0, 0, RCL_ROS_TIME};

 
    void delayedInitialization() {
        init_timer_->cancel();
        initializeModules();

        if (can_protocol_->initialize(radar_can_interface_)) {
            radar_status_ = RadarStatus::NORMAL;
            RCLCPP_INFO(this->get_logger(), "[RADAR] Radar CAN initialized successfully.");
            RCLCPP_INFO(this->get_logger(), "[SYS] Node started. CAN: %s, S5(Self_dev): %s, S9(PX4): %s",
                radar_can_interface_.c_str(), self_dev_port_.c_str(), px4_port_.c_str());
        } else {
            radar_status_ = RadarStatus::UNINITIALIZED;
            RCLCPP_WARN(this->get_logger(), "[SYS] Radar CAN not found. Entering polling mode...");
        }

        // 算法主处理循环
        auto algo_period = std::chrono::milliseconds(static_cast<int>(1000.0 / processing_rate_));
        algo_timer_ = this->create_wall_timer(algo_period, std::bind(&ObstacleAvoidanceNode::processingLoop, this));

        // PX4 MAVLink循环
        if (px4_fd_ != -1) {
            auto obs_period = std::chrono::milliseconds(static_cast<int>(1000.0 / obstacle_dist_hz_));
            obs_dist_timer_ = this->create_wall_timer(obs_period,std::bind(&ObstacleAvoidanceNode::sendObstacleDistanceTick, this));
            RCLCPP_INFO(this->get_logger(),"[PX4-330] OBSTACLE_DISTANCE Send by %.0fHz", obstacle_dist_hz_);

            // 心跳发送循环
            heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ObstacleAvoidanceNode::sendCompanionHeartbeat, this));
            RCLCPP_INFO(this->get_logger(), "[PX4-Heart] PX4 heartbeat send loop started.");

            px4_recv_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&ObstacleAvoidanceNode::receivePX4Messages, this));
            RCLCPP_INFO(this->get_logger(), "[PX4-RX] PX4 MAVLink receive loop started.");
        }
    }

    void sendObstacleDistanceTick() {
        sendPX4ObstacleDistance(latest_obstacle_dist_);
    }
 
    int openSerialPort(const std::string& port, int baud_rate) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            RCLCPP_WARN(this->get_logger(), "Port %s not available or no permission. Skipping...", port.c_str());
            return -1;
        }
        struct termios options;
        tcgetattr(fd, &options);

        speed_t baud = getBaudRateMacro(baud_rate);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_iflag &= ~(IXON | IXOFF | IXANY | 
                     ICRNL | INLCR | IGNCR | 
                     ISTRIP | INPCK | BRKINT); 
        tcsetattr(fd, TCSANOW, &options);
        RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port.c_str(), baud_rate);
        return fd;
    }

    speed_t getBaudRateMacro(int baud_rate) {
        switch (baud_rate) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:     return B115200;
        }
    }

  
    void receivePX4Messages() {
        if (px4_fd_ == -1) return;

        uint8_t buf[512];
        int n = read(px4_fd_, buf, sizeof(buf));
        if (n <= 0) 
        {return;}
        else {RCLCPP_DEBUG(this->get_logger(), "Received %d bytes from PX4", n);}

        mavlink_message_t msg;
        for (int i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &mavlink_status_)) continue;
               // RCLCPP_INFO(this->get_logger(), "[PX4-RX] msgid=%-4d sysid=%d compid=%d", msg.msgid, msg.sysid, msg.compid);
             if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                if (msg.compid == MAV_COMP_ID_AUTOPILOT1) {
                    const uint8_t prev_main_mode = (px4_custom_mode_ >> 16) & 0xFF;
                    px4_base_mode_      = hb.base_mode;
                    px4_custom_mode_    = hb.custom_mode;
                    px4_target_sysid_   = msg.sysid;
                    px4_target_compid_  = msg.compid;
                    px4_heartbeat_recv_ = true;

                    const uint8_t main_mode = (px4_custom_mode_ >> 16) & 0xFF;
                    const uint8_t sub_mode  =  px4_custom_mode_        & 0xFF;
                    const bool    armed     = (px4_base_mode_ & MAV_MODE_FLAG_SAFETY_ARMED) != 0;

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[PX4-Heart] sysid=%d | armed=%-3s | main_mode=%d | sub_mode=%d | base=0x%02X",
                        msg.sysid,
                        armed ? "YES" : "NO",
                        main_mode, sub_mode, px4_base_mode_);

                    if (main_mode != prev_main_mode) {
                        RCLCPP_WARN(this->get_logger(),
                            "[PX4-RX] !! FLY MODE CHANGE !!: main_mode %d -> %d | armed=%s",
                            prev_main_mode, main_mode, armed ? "YES" : "NO");
                    }

                    if (!attitude_interval_requested_) {
                        sendPX4MessageInterval(MAVLINK_MSG_ID_ATTITUDE, attitude_stream_hz_);
                        attitude_interval_requested_ = true;
                    }
                    
                    if (avoid_state_ == AvoidState::AVOIDING &&
                        mode_switch_ack_pending_ &&
                        main_mode == PX4_MAIN_MODE_POSCTL)
                    {
                        mode_switch_ack_pending_ = false;
                        RCLCPP_WARN(this->get_logger(),
                            "[PX4-RX] send mode switch ack（main_mode=%d=POSCTL）", main_mode);
                    }

                    if (avoid_state_ == AvoidState::AVOIDING &&
                    main_mode != PX4_MAIN_MODE_POSCTL)
                    {
                    avoid_state_ = AvoidState::IDLE;
                    mode_switch_ack_pending_ = false;
                    obstacle_was_clear_ = false;
                        RCLCPP_WARN(this->get_logger(),
                            "[PX4-RX] main_mode=%d , avoid_state_-> IDLE",
                            main_mode);
                    }
                }
            }

            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t global_pos;
                mavlink_msg_global_position_int_decode(&msg, &global_pos);
                current_relative_alt_m_ = global_pos.relative_alt / 1000.0f;
            }

            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                latest_roll_rad_ = attitude.roll;
                latest_pitch_rad_ = attitude.pitch;
                last_attitude_time_ = this->now();
                attitude_received_ = true;

                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[PX4-ATT] roll=%.2f deg pitch=%.2f deg",
                    latest_roll_rad_ * 180.0f / static_cast<float>(M_PI),
                    latest_pitch_rad_ * 180.0f / static_cast<float>(M_PI));
            }

            if (msg.msgid == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
                mavlink_extended_sys_state_t ext_state;
                mavlink_msg_extended_sys_state_decode(&msg, &ext_state);
                px4_landed_state_ = ext_state.landed_state;
                // MAV_LANDED_STATE_ON_GROUND = 1 (在地面)
                // MAV_LANDED_STATE_IN_AIR = 2    (在空中)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[PX4-RX] EXT_SYS_STATE: landed_state=%d",
                    px4_landed_state_);
            }

             if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);

                if (ack.command == MAV_CMD_DO_SET_MODE) {
                    mode_switch_ack_pending_ = false;
                    if (ack.result == MAV_RESULT_ACCEPTED) {
                        RCLCPP_WARN(this->get_logger(),
                            "[PX4-ACK]DO_SET_MODE ACK SUCCEED (MAV_CMD_DO_SET_MODE)");
                    } else {
                        RCLCPP_ERROR(this->get_logger(),
                            "[PX4-ACK]DO_SET_MODE ACK FAILED. result=%d, check MAV_RESULT enum for details.",
                            ack.result);
                        avoid_state_ = AvoidState::IDLE;
                    }
                }
            }
        }
    }

    void sendPX4MessageInterval(uint32_t message_id, float rate_hz) {
        if (px4_fd_ == -1 || rate_hz <= 0.0f) return;

        mavlink_message_t msg;
        const float interval_us = 1000000.0f / rate_hz;

        mavlink_msg_command_long_pack(
            ONBOARD_SYSID,
            ONBOARD_COMPID,
            &msg,
            px4_target_sysid_,
            px4_target_compid_,
            MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            static_cast<float>(message_id),
            interval_us,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ssize_t written = write(px4_fd_, buf, len);
        if (written < 0) {
            RCLCPP_WARN(this->get_logger(),
                "[PX4-ATT] Failed to request msgid=%u interval errno=%d",
                message_id, errno);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[PX4-ATT] Requested msgid=%u at %.1fHz",
                message_id, rate_hz);
        }
    }

    void sendCompanionHeartbeat() {
        if (px4_fd_ == -1) return;

        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(
            ONBOARD_SYSID,               
            ONBOARD_COMPID,             
            &msg,
            MAV_TYPE_ONBOARD_CONTROLLER, 
            MAV_AUTOPILOT_INVALID,       
            0,                           
            0,                           
            MAV_STATE_ACTIVE             
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ssize_t written = write(px4_fd_, buf, len);
        if (written < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[PX4-Heart] !! Heartbeat write failed errno =%d", errno);
        }
    }

    void sendPX4ObstacleDistance(float distance_m) {
        if (px4_fd_ == -1) return;

        mavlink_message_t msg;
        uint16_t distances[72];
        std::fill_n(distances, 72, UINT16_MAX);
        uint16_t forward_dist_cm;
        bool has_obstacle = (distance_m > 3.0f && distance_m < 60.0f);
        if (has_obstacle) {
            forward_dist_cm = static_cast<uint16_t>(distance_m * 100.0f);
        }
        else {
            forward_dist_cm = 6000;
        }

            // 正前方 ±10°：扇区 70(-10°)、71(-5°)、0(0°)、1(+5°)、2(+10°)
            //distances[70] = forward_dist_cm;
            //distances[71] = forward_dist_cm;
            distances[0]  = forward_dist_cm;
            //distances[1]  = forward_dist_cm;
            //distances[2]  = forward_dist_cm;

        uint64_t time_usec = this->now().nanoseconds() / 1000;
        mavlink_msg_obstacle_distance_pack(
            ONBOARD_SYSID, ONBOARD_COMPID, &msg,
            time_usec,
            MAV_DISTANCE_SENSOR_RADAR,
            distances,
            5,    
            300,     // min_distance_cm
            5700,   // max_distance_cm
            0,      // angle_offset
            0,      // frame
            MAV_FRAME_BODY_FRD
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        write(px4_fd_, buf, len);

        if (has_obstacle) {
            RCLCPP_INFO(this->get_logger(), /**this->get_clock(),2000,*/
                "[PX4-330] send dist=%.2fm", distance_m);
        } else {
            RCLCPP_INFO(this->get_logger(), /**this->get_clock(),2000,*/
                "[PX4-330] NO obstacle");
        }
    }

    void sendPX4SetMode() {
        if (px4_fd_ == -1) return;
        mavlink_message_t msg;

        float base_mode_para_ = static_cast<float>(px4_base_mode_ | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED); 
         mavlink_msg_command_long_pack(
            ONBOARD_SYSID,          
            ONBOARD_COMPID,         
            &msg,
            px4_target_sysid_,       
            px4_target_compid_,      
            MAV_CMD_DO_SET_MODE,     
            0,                      
            base_mode_para_,                       
            main_mode_para_,   
            sub_mode_para_,                                               
            0.0f, 0.0f, 0.0f, 0.0f                                
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ssize_t written = write(px4_fd_, buf, len);
        if (written < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "[PX4-SetMode] !! Write failed errno =%d", errno);
        } else {
            RCLCPP_INFO(this->get_logger(),
                "[PX4-SetMode] Sent mode switch command: base=0x%02X main=%d sub=%d",
                static_cast<int>(base_mode_para_),
                static_cast<int>(main_mode_para_),
                static_cast<int>(sub_mode_para_));
        }
        mode_switch_ack_pending_ = true;
        mode_switch_sent_time_   = this->now();
    }

    void checkAndSwitchPX4Mode(float min_distance) {
        if (!px4_heartbeat_recv_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "[PX4-Heart] !!! No heartbeat received yet, cannot switch mode. Ensure PX4 is powered and MAVLink is configured correctly!!!");
            return;
        }

        const uint8_t current_main_mode = (px4_custom_mode_ >> 16) & 0xFF;
        const bool    is_armed          = (px4_base_mode_ & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        const bool    is_in_air         = (px4_landed_state_ == MAV_LANDED_STATE_IN_AIR);
        
        if (!is_armed || !is_in_air) {
             return; 
        }

        if (current_relative_alt_m_ < min_avoidance_alt_) {
            if (avoid_state_ == AvoidState::AVOIDING) {
                 avoid_state_ = AvoidState::IDLE;
            }
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[PX4] 当前高度 %.2fm < 设定的避障起效高度 %.2fm，禁止切模式", 
                current_relative_alt_m_, min_avoidance_alt_);
             return; 
        }

        if (avoid_state_ == AvoidState::IDLE) {
            if (min_distance >= hold_trigger_distance_) return;

            if (!is_armed) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[PX4] 障碍物 %.2fm 达到阈值，但飞控未解锁，跳过", min_distance);
                 return;
            }

            double elapsed = (this->now() - last_switch_cmd_time_).seconds();
            if (elapsed < SWITCH_CMD_INTERVAL_S) return;
            last_switch_cmd_time_ = this->now();

            if (current_main_mode == PX4_MAIN_MODE_POSCTL) {
                avoid_state_ = AvoidState::AVOIDING;
                obstacle_was_clear_ = false;
                RCLCPP_WARN(this->get_logger(),
                    "[PX4] 障碍物 %.2fm，飞控已在 POSCTL",
                    min_distance);
                return;
            }

            sendPX4SetMode();
            avoid_state_ = AvoidState::AVOIDING;
            obstacle_was_clear_ = false;
            return;
        }

        if (avoid_state_ == AvoidState::AVOIDING) {

            if (mode_switch_ack_pending_) {
                double ack_elapsed = (this->now() - mode_switch_sent_time_).seconds();
                if (ack_elapsed > ACK_TIMEOUT_S &&
                    current_main_mode != PX4_MAIN_MODE_POSCTL)
                {
                    mode_switch_ack_pending_ = false;
                    avoid_state_ = AvoidState::IDLE;
                    obstacle_was_clear_ = false;
                    RCLCPP_ERROR(this->get_logger(),
                        "[MODE] ACK超时(%.1fs)，模式未切成功(cur=%d)，重置IDLE将重试",
                        ack_elapsed, current_main_mode);
                    return;
                }
            }

            if (min_distance < hold_trigger_distance_) {
                obstacle_was_clear_ = false; 
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[MODE] 避障中！dist=%.2fm | 330持续发送 | POSCTL防撞刹停",
                    min_distance);
                return;
            }

            if (!obstacle_was_clear_) {
                obstacle_clear_time_ = this->now();
                obstacle_was_clear_  = true;
                RCLCPP_WARN(this->get_logger(),
                    "[MODE] 障碍物消失(dist=%.2fm > thr=%.2fm)，"
                    "%.0fs后自动重置，飞手可遥控或地面站恢复任务",
                    min_distance, hold_trigger_distance_, AVOID_RESET_TIMEOUT_S);
            }

            double clear_elapsed = (this->now() - obstacle_clear_time_).seconds();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[MODE] 等待重置 %.1f/%.0fs | dist=%.2fm | mode=%d",
                clear_elapsed, AVOID_RESET_TIMEOUT_S,
                min_distance, current_main_mode);

            if (clear_elapsed >= AVOID_RESET_TIMEOUT_S) {
                avoid_state_ = AvoidState::IDLE;
                mode_switch_ack_pending_ = false;
                obstacle_was_clear_ = false;
                RCLCPP_WARN(this->get_logger(),
                    "[MODE] 障碍物消失超%.0fs，自动重置IDLE，可重新触发或恢复任务",
                    AVOID_RESET_TIMEOUT_S);
            }
        }
    }
 
    static constexpr uint16_t CRC16_TABLE[256] = {
        0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
        0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
        0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
        0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
        0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,
        0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
        0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
        0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
        0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
        0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
        0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
        0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
        0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
        0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
        0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
        0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
        0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
        0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
        0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
        0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
        0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
        0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
        0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
        0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
        0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
        0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
        0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
        0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
        0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
        0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
        0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
        0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
    };

    uint16_t calculateCRC16(const std::vector<uint8_t>& data) {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF];
        }
        return crc;
    }

    std::vector<uint8_t> buildSerialCommand(bool detected, float distance,
                                             uint8_t level, RadarStatus status) {
        std::vector<uint8_t> cmd;
        cmd.reserve(13);
        cmd.push_back(0xFE);
        cmd.push_back(0xEF);
        cmd.push_back(0x02);
        cmd.push_back(0x01);
        cmd.push_back(0x1A);
        cmd.push_back(0x05);
        cmd.push_back(detected ? 0x01 : 0x00);
        uint16_t dist_cm = static_cast<uint16_t>(distance * 100);
        cmd.push_back((dist_cm >> 8) & 0xFF);
        cmd.push_back(dist_cm & 0xFF);
        cmd.push_back(level);
        cmd.push_back(static_cast<uint8_t>(status));
        uint16_t crc = calculateCRC16(cmd);
        cmd.push_back(crc & 0xFF);
        cmd.push_back((crc >> 8) & 0xFF);
        return cmd;
    }

 
    void declareParameters() {
        this->declare_parameter("radar_can_interface", "can0");
        this->declare_parameter("self_dev_port", "/dev/ttyS5");
        this->declare_parameter("self_dev_baud", 460800);
        this->declare_parameter("px4_port", "/dev/ttyS9");
        this->declare_parameter("px4_baud", 115200);
        this->declare_parameter("frame_id", "radar");
        this->declare_parameter("processing_rate", 10.0);
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("enable_logging", true);

        this->declare_parameter("min_power_threshold", 40.0);
        this->declare_parameter("max_range", 15.0);
        this->declare_parameter("direct_map_power_threshold", 60.0);
        this->declare_parameter("merge_dist", 5.0);
        this->declare_parameter("max_fov_deg", 20.0);
        this->declare_parameter("high_power_threshold", 60.0);
        this->declare_parameter("high_power_multiplier", 5);
        this->declare_parameter("min_range", 3.0);
        this->declare_parameter("min_x", -3.0);
        this->declare_parameter("max_x", 3.0);
        this->declare_parameter("min_elevation_deg", -10.0);
        this->declare_parameter("max_elevation_deg", 15.0);
        this->declare_parameter("max_z", 2.0);
        this->declare_parameter("min_z", -1.0);

        this->declare_parameter("sor_mean_k", 20);
        this->declare_parameter("sor_stddev_mul", 1.0);
        this->declare_parameter("cluster_tolerance_base", 0.4);
        this->declare_parameter("cluster_tolerance_k", 0.05);
        this->declare_parameter("cluster_max_tolerance", 1.5);
        this->declare_parameter("min_cluster_size", 1);
        this->declare_parameter("max_cluster_size", 500);

        this->declare_parameter("grid_resolution", 0.2);
        this->declare_parameter("grid_width", 30.0);
        this->declare_parameter("grid_height", 30.0);
        this->declare_parameter("occupancy_threshold", 0.6);
        this->declare_parameter("decay_rate", 0.95);
        this->declare_parameter("decay_time", 1.0);

        this->declare_parameter("emergency_distance", 2.0);
        this->declare_parameter("danger_distance", 2.5);
        this->declare_parameter("caution_distance", 5.0);
        this->declare_parameter("min_continuous_frames", 3);
        this->declare_parameter("warning_timeout", 0.5);
 
        this->declare_parameter("hold_trigger_distance", 5.0);
        this->declare_parameter("obstacle_dist_hz", 10.0);
        this->declare_parameter("base_mode_para", 1.0);
        this->declare_parameter("main_mode_para", 3.0);
        this->declare_parameter("sub_mode_para", 0.0);
        this->declare_parameter("min_avoidance_alt", 1.5);
        this->declare_parameter("enable_attitude_compensation", true);
        this->declare_parameter("attitude_timeout", 0.3);
        this->declare_parameter("attitude_stream_hz", 20.0);
        this->declare_parameter("pitch_compensation_sign", 1.0);
        this->declare_parameter("roll_compensation_sign", 1.0);
    }

    void loadParameters() {
        radar_can_interface_ = this->get_parameter("radar_can_interface").as_string();
        self_dev_port_       = this->get_parameter("self_dev_port").as_string();
        self_dev_baud_       = this->get_parameter("self_dev_baud").as_int();
        px4_port_            = this->get_parameter("px4_port").as_string();
        px4_baud_            = this->get_parameter("px4_baud").as_int();
        frame_id_            = this->get_parameter("frame_id").as_string();
        processing_rate_     = this->get_parameter("processing_rate").as_double();
        enable_visualization_= this->get_parameter("enable_visualization").as_bool();
        base_mode_para_      = static_cast<float>(this->get_parameter("base_mode_para").as_double());
        main_mode_para_      = static_cast<float>(this->get_parameter("main_mode_para").as_double());
        sub_mode_para_       = static_cast<float>(this->get_parameter("sub_mode_para").as_double());
        hold_trigger_distance_ = static_cast<float>(this->get_parameter("hold_trigger_distance").as_double());
        min_avoidance_alt_     = static_cast<float>(this->get_parameter("min_avoidance_alt").as_double());
        obstacle_dist_hz_      = static_cast<float>(this->get_parameter("obstacle_dist_hz").as_double());
        enable_logging_      = this->get_parameter("enable_logging").as_bool();
        direct_map_power_threshold_ = static_cast<float>(this->get_parameter("direct_map_power_threshold").as_double());
        merge_dist_ = static_cast<float>(this->get_parameter("merge_dist").as_double());
        enable_attitude_compensation_ = this->get_parameter("enable_attitude_compensation").as_bool();
        attitude_timeout_ = static_cast<float>(this->get_parameter("attitude_timeout").as_double());
        attitude_stream_hz_ = static_cast<float>(this->get_parameter("attitude_stream_hz").as_double());
        pitch_compensation_sign_ = static_cast<float>(this->get_parameter("pitch_compensation_sign").as_double());
        roll_compensation_sign_ = static_cast<float>(this->get_parameter("roll_compensation_sign").as_double());

        if (!enable_logging_) {
            rcutils_logging_set_logger_level(
                this->get_logger().get_name(), 
                RCUTILS_LOG_SEVERITY_FATAL
            );
            
            rcutils_logging_set_logger_level(
                "obstacle_avoidance", 
                RCUTILS_LOG_SEVERITY_FATAL
            );
            std::cout << "[System] ROS_LOGGER CLOSED" << std::endl;
        }

        RCLCPP_INFO(this->get_logger(),
            "Parameters loaded: hold_trigger=%.1fm  330包=%.0fHz",
            hold_trigger_distance_, obstacle_dist_hz_);
        RCLCPP_INFO(this->get_logger(),
            "[PX4-ATT] compensation=%s timeout=%.2fs stream=%.1fHz pitch_sign=%.1f roll_sign=%.1f",
            enable_attitude_compensation_ ? "ON" : "OFF",
            attitude_timeout_, attitude_stream_hz_,
            pitch_compensation_sign_, roll_compensation_sign_);
    }

 
    void initializeModules() {
        can_protocol_       = std::make_shared<CANProtocol>(shared_from_this());
        self_dev_fd_        = openSerialPort(self_dev_port_, self_dev_baud_);
        px4_fd_             = openSerialPort(px4_port_, px4_baud_);
        point_filter_       = std::make_unique<PointCloudFilter>();
        spatial_clustering_ = std::make_unique<SpatialClustering>();
        grid_map_fusion_    = std::make_unique<GridMapFusion>();
        decision_controller_= std::make_unique<DecisionController>();

        float min_power    = this->get_parameter("min_power_threshold").as_double();
        float high_power   = this->get_parameter("high_power_threshold").as_double();
        int   multiplier   = this->get_parameter("high_power_multiplier").as_int();
        float max_fov      = this->get_parameter("max_fov_deg").as_double();
        float min_range    = this->get_parameter("min_range").as_double();
        float max_range    = this->get_parameter("max_range").as_double();
        float min_z        = this->get_parameter("min_z").as_double();
        float max_z        = this->get_parameter("max_z").as_double();
        float min_x        = this->get_parameter("min_x").as_double();
        float max_x        = this->get_parameter("max_x").as_double();
        float min_elevation= this->get_parameter("min_elevation_deg").as_double();
        float max_elevation= this->get_parameter("max_elevation_deg").as_double();
        point_filter_->setPowerThreshold(min_power);
        point_filter_->setHighPowerParams(high_power, multiplier);
        point_filter_->setROI(min_range, max_range, min_x, max_x,
                              max_fov, min_z, max_z, min_elevation, max_elevation);

        int    sor_k    = this->get_parameter("sor_mean_k").as_int();
        double sor_std  = this->get_parameter("sor_stddev_mul").as_double();
        double tol_base = this->get_parameter("cluster_tolerance_base").as_double();
        double tol_k    = this->get_parameter("cluster_tolerance_k").as_double();
        double max_tol  = this->get_parameter("cluster_max_tolerance").as_double();
        int    min_size = this->get_parameter("min_cluster_size").as_int();
        int    max_size = this->get_parameter("max_cluster_size").as_int();
        spatial_clustering_->setSORParameters(sor_k, sor_std);
        spatial_clustering_->setClusterParameters(tol_base, tol_k, max_tol, min_size, max_size);

        float grid_res   = this->get_parameter("grid_resolution").as_double();
        float grid_w     = this->get_parameter("grid_width").as_double();
        float grid_h     = this->get_parameter("grid_height").as_double();
        float occ_thresh = this->get_parameter("occupancy_threshold").as_double();
        float decay_r    = this->get_parameter("decay_rate").as_double();
        float decay_t    = this->get_parameter("decay_time").as_double();
        grid_map_fusion_->initialize(grid_res, grid_w, grid_h);
        grid_map_fusion_->setOccupancyThreshold(occ_thresh);
        grid_map_fusion_->setTemporalDecay(decay_r, decay_t);

        float emerg_dist  = this->get_parameter("emergency_distance").as_double();
        float danger_dist = this->get_parameter("danger_distance").as_double();
        float caution_dist= this->get_parameter("caution_distance").as_double();
        int   min_frames  = this->get_parameter("min_continuous_frames").as_int();
        float warn_timeout= this->get_parameter("warning_timeout").as_double();
        decision_controller_->setDistanceThresholds(emerg_dist, danger_dist, caution_dist);
        decision_controller_->setContinuousWarningParams(min_frames, warn_timeout);
    }

    void processingLoop() {
        auto start_time = this->now();

         if (radar_status_ != RadarStatus::NORMAL) {
            static rclcpp::Time last_reconnect_time = this->now();
            if ((this->now() - last_reconnect_time).seconds() > 2.0) {
                can_protocol_->close();
                if (can_protocol_->initialize(radar_can_interface_)) {
                    radar_status_ = RadarStatus::NORMAL;
                    RCLCPP_INFO(this->get_logger(), "Radar reconnected!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Radar not connected...");
                }
                last_reconnect_time = this->now();
            }
            publishEmptyWarning(radar_status_);
            return;
        }

        std::vector<RadarPoint> radar_points;
        static int consecutive_failures = 0;
        if (!can_protocol_->receiveRadarData(radar_points, 50)) {
            consecutive_failures++;
            if (consecutive_failures >= 3) {
            can_protocol_->close();
            radar_status_ = RadarStatus::DISCONNECTED;
            consecutive_failures = 0;
            }
            RCLCPP_WARN(this->get_logger(), "Radar link lost!");
            publishEmptyWarning(radar_status_);
            return;
        }
        consecutive_failures = 0;

        grid_map_fusion_->clearInstantMask();
        
        if (enable_visualization_) {
            auto raw_msg = radarPointsToROSMsg(radar_points, frame_id_, start_time);
            raw_cloud_pub_->publish(raw_msg);
        }

         geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "base_link", frame_id_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "等待雷达外参装配参数 (base_link <- %s): %s", frame_id_.c_str(), ex.what());
            return;
        }

        const Eigen::Isometry3f T = tf2::transformToEigen(transform_stamped.transform).cast<float>();
        const Eigen::Matrix3f R = T.rotation();
        const Eigen::Vector3f t = T.translation();

        pcl::PointCloud<PointXYZP>::Ptr base_link_cloud(new pcl::PointCloud<PointXYZP>);
        base_link_cloud->header.frame_id = "base_link";
        base_link_cloud->points.reserve(radar_points.size());

        for (const auto& pt : radar_points) {
            const Eigen::Vector3f p_trans = R * Eigen::Vector3f(pt.x, pt.y, pt.z) + t;
            PointXYZP new_pt;
            new_pt.x = p_trans.x(); new_pt.y = p_trans.y();
            new_pt.z = p_trans.z(); new_pt.power = pt.power;
            base_link_cloud->points.push_back(new_pt);
        }
        base_link_cloud->width  = base_link_cloud->points.size();
        base_link_cloud->height = 1;

        applyAttitudeCompensation(base_link_cloud);

        pcl::PointCloud<PointXYZP>::Ptr filtered_cloud(new pcl::PointCloud<PointXYZP>);
        bool has_valid_points = (point_filter_->filterPointCloud(base_link_cloud, filtered_cloud) > 0);

        if (enable_visualization_ && has_valid_points) {
            sensor_msgs::msg::PointCloud2 filtered_msg;
            pcl::toROSMsg(*filtered_cloud, filtered_msg);
            filtered_msg.header.frame_id = "base_link";
            filtered_msg.header.stamp    = start_time;
            filtered_cloud_pub_->publish(filtered_msg);
        }

        std::vector<ClusterResult> direct_high_power_clusters;
        pcl::PointCloud<PointXYZP>::Ptr regular_cloud(new pcl::PointCloud<PointXYZP>);
        regular_cloud->header.frame_id = "base_link";

        if (has_valid_points) {
            for (const auto& pt : filtered_cloud->points) {              
                regular_cloud->points.push_back(pt);

                if (pt.power >= direct_map_power_threshold_) {
                    bool merged = false;
                    Eigen::Vector3f p_trans(pt.x, pt.y, pt.z);
                    
                    for (auto& cluster : direct_high_power_clusters) {
                        Eigen::Vector3f expanded_min = cluster.min_bound - Eigen::Vector3f(merge_dist_, merge_dist_, merge_dist_);
                        Eigen::Vector3f expanded_max = cluster.max_bound + Eigen::Vector3f(merge_dist_, merge_dist_, merge_dist_);
                        
                        if (p_trans.x() >= expanded_min.x() && p_trans.x() <= expanded_max.x() &&
                            p_trans.y() >= expanded_min.y() && p_trans.y() <= expanded_max.y() &&
                            p_trans.z() >= expanded_min.z() && p_trans.z() <= expanded_max.z()) {
                            
                            cluster.cloud->points.push_back(pt);
                            cluster.min_bound = cluster.min_bound.cwiseMin(p_trans);
                            cluster.max_bound = cluster.max_bound.cwiseMax(p_trans);
                            cluster.centroid = (cluster.min_bound + cluster.max_bound) / 2.0f;
                            merged = true;
                            break;
                        }
                    }
                    
                    if (!merged) {
                        ClusterResult direct_cluster;
                        direct_cluster.centroid = p_trans;
                        direct_cluster.min_bound = p_trans - Eigen::Vector3f(0.1f, 0.1f, 0.1f);
                        direct_cluster.max_bound = p_trans + Eigen::Vector3f(0.1f, 0.1f, 0.1f);
                        direct_cluster.cloud.reset(new pcl::PointCloud<PointXYZP>);
                        direct_cluster.cloud->points.push_back(pt);
                        direct_high_power_clusters.push_back(direct_cluster);
                    }
                } 
            }
        }
        regular_cloud->width = regular_cloud->points.size();
        regular_cloud->height = 1;

        std::vector<ClusterResult> clusters;
        if (!regular_cloud->points.empty()) {
            spatial_clustering_->clusterPointCloud(regular_cloud, clusters);
        }

        if (clusters.empty() && direct_high_power_clusters.empty()) {
            latest_obstacle_dist_ = 9999.0f;
            publishEmptyWarning(RadarStatus::NORMAL);
            return;
        }

        grid_map_fusion_->updateGridMap(clusters, start_time);

        if (!direct_high_power_clusters.empty()) {
            grid_map_fusion_->markInstantOccupancy(direct_high_power_clusters, start_time);
        }

        if (enable_visualization_) {
            std::vector<ClusterResult> vis_clusters = clusters;
            vis_clusters.insert(vis_clusters.end(), direct_high_power_clusters.begin(), direct_high_power_clusters.end());
            publishClusterVisualization(vis_clusters, start_time);
            grid_map_pub_->publish(grid_map_fusion_->getOccupancyGridMsg("base_link"));
        }

        DecisionResult decision;
        decision_controller_->makeDecision(*grid_map_fusion_, start_time, decision);
        latest_obstacle_dist_ = decision.min_distance;

        dispatchSerialData(decision.brake_required, decision.min_distance,
                           static_cast<uint8_t>(decision.warning_level), radar_status_);

        checkAndSwitchPX4Mode(decision.min_distance);  

        publishWarning(decision, start_time);

         double processing_time = (this->now() - start_time).seconds();
        if (processing_time > 1.0 / processing_rate_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Processing slow: %.1f ms", processing_time * 1000.0);
        }
    }

    void applyAttitudeCompensation(const pcl::PointCloud<PointXYZP>::Ptr& cloud) {
        if (!enable_attitude_compensation_ || !cloud || cloud->points.empty()) return;

        if (!attitude_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[PX4-ATT] No ATTITUDE received yet, using uncompensated point cloud.");
            return;
        }

        const double attitude_age = (this->now() - last_attitude_time_).seconds();
        if (attitude_age > attitude_timeout_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[PX4-ATT] ATTITUDE timeout %.2fs > %.2fs, using uncompensated point cloud.",
                attitude_age, attitude_timeout_);
            return;
        }

        const float pitch_rad = -pitch_compensation_sign_ * latest_pitch_rad_;
        const float roll_rad = -roll_compensation_sign_ * latest_roll_rad_;
        const Eigen::Matrix3f pitch_comp =
            Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitX()).toRotationMatrix();
        const Eigen::Matrix3f roll_comp =
            Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitY()).toRotationMatrix();
        const Eigen::Matrix3f attitude_comp = roll_comp * pitch_comp;

        for (auto& pt : cloud->points) {
            const Eigen::Vector3f p_comp = attitude_comp * Eigen::Vector3f(pt.x, pt.y, pt.z);
            pt.x = p_comp.x();
            pt.y = p_comp.y();
            pt.z = p_comp.z();
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[PX4-ATT] Applied compensation roll=%.2f deg pitch=%.2f deg",
            latest_roll_rad_ * 180.0f / static_cast<float>(M_PI),
            latest_pitch_rad_ * 180.0f / static_cast<float>(M_PI));
    }

   
    void dispatchSerialData(bool brake_required, float min_distance,
                             uint8_t warning_level,
                             RadarStatus status = RadarStatus::NORMAL) {
        if (self_dev_fd_ != -1) {
            auto serial_cmd = buildSerialCommand(brake_required, min_distance, warning_level, status);
            //if (min_distance < 60.0f) {
                //RCLCPP_INFO(this->get_logger(),
                //    "=> [自研飞控] 刹车: %s | 距离: %.2f m | 告警等级: %d",
                //   brake_required ? "是(1)" : "否(0)", min_distance, warning_level);
            //}
            write(self_dev_fd_, serial_cmd.data(), serial_cmd.size());
        }
    }

   
    void publishEmptyWarning(RadarStatus status) {
        auto msg = mmwave_obstacle_avoidance_ros2::msg::ObstacleWarning();
        msg.header.stamp     = this->now();
        msg.header.frame_id  = "base_link";
        msg.obstacle_detected= false;
        msg.min_distance     = 100.0f;
        msg.warning_level    = 0;
        msg.obstacle_count   = 0;
        msg.sensor_status    = static_cast<uint8_t>(status);
        warning_pub_->publish(msg);
        dispatchSerialData(false, 100.0f, 0, status);
    }

  
    void publishWarning(const DecisionResult& decision, const rclcpp::Time& timestamp) {
        auto msg = mmwave_obstacle_avoidance_ros2::msg::ObstacleWarning();
        msg.header.stamp      = timestamp;
        msg.header.frame_id   = "base_link";
        msg.obstacle_detected = decision.brake_required;
        msg.min_distance      = decision.min_distance;
        msg.closest_point.x   = decision.closest_point.x();
        msg.closest_point.y   = decision.closest_point.y();
        msg.closest_point.z   = 0.0;
        msg.warning_level     = static_cast<uint8_t>(decision.warning_level);
        msg.obstacle_count    = decision.obstacle_count;
        warning_pub_->publish(msg);
    }
 
    void publishClusterVisualization(const std::vector<ClusterResult>& clusters,
                                      const rclcpp::Time& timestamp) {
        pcl::PointCloud<PointXYZP> combined_cloud;
        for (const auto& cluster : clusters) combined_cloud += *cluster.cloud;

        sensor_msgs::msg::PointCloud2 clustered_msg;
        pcl::toROSMsg(combined_cloud, clustered_msg);
        clustered_msg.header.frame_id = "base_link";
        clustered_msg.header.stamp    = timestamp;
        clustered_cloud_pub_->publish(clustered_msg);

        visualization_msgs::msg::MarkerArray markers;
        for (size_t i = 0; i < clusters.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp    = timestamp;
            marker.ns      = "clusters";
            marker.id      = static_cast<int>(i);
            marker.type    = visualization_msgs::msg::Marker::CUBE;
            marker.action  = visualization_msgs::msg::Marker::ADD;
            marker.lifetime= rclcpp::Duration::from_seconds(0.3);
            marker.pose.position.x   = clusters[i].centroid.x();
            marker.pose.position.y   = clusters[i].centroid.y();
            marker.pose.position.z   = clusters[i].centroid.z();
            marker.pose.orientation.w= 1.0;
            auto sz = clusters[i].max_bound - clusters[i].min_bound;
            marker.scale.x = sz.x() < 0.1f ? 0.1f : sz.x();
            marker.scale.y = sz.y() < 0.1f ? 0.1f : sz.y();
            marker.scale.z = sz.z() < 0.1f ? 0.1f : sz.z();
            marker.color.r = 1.0f; marker.color.g = 0.5f;
            marker.color.b = 0.0f; marker.color.a = 0.5f;
            markers.markers.push_back(marker);
        }
        markers_pub_->publish(markers);
    }
};

 int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ObstacleAvoidanceNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("obstacle_avoidance_node"),
            "Exception: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
