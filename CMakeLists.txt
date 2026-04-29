cmake_minimum_required(VERSION 3.8)
project(mmwave_obstacle_avoidance_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)# 导航地图包 OccupancyGrid概率地图
find_package(visualization_msgs REQUIRED)#可视化包 MarkerArray障碍物三维显示
find_package(pcl_conversions REQUIRED)# PCL与ROS消息类型转换
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)# TF2与Eigen库的转换支持
find_package(tf2_ros REQUIRED)# TF2的ROS接口，用于坐标变换广播和监听
find_package(rosidl_default_generators REQUIRED)# 自定义消息编译器
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Eigen3 REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleWarning.msg"
  "msg/ClusterInfo.msg"
  "msg/GridMapInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs# 自定义消息中使用了std_msgs和geometry_msgs中的类型，因此需要声明依赖
)

set(dependencies
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
  pcl_conversions tf2 tf2_ros tf2_eigen
)

add_library(${PROJECT_NAME}_lib SHARED
  src/can_protocol.cpp
  src/point_cloud_filter.cpp
  src/spatial_clustering.cpp
  src/grid_map_fusion.cpp
  src/decision_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node ${dependencies})
target_link_libraries(obstacle_avoidance_node ${PROJECT_NAME}_lib ${PCL_LIBRARIES})
rosidl_target_interfaces(obstacle_avoidance_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS obstacle_avoidance_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch config scripts DESTINATION share/${PROJECT_NAME}/)

ament_package()