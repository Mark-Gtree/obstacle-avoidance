# mmwave_obstacle_avoidance_ros2

毫米波雷达前向感知避障 ROS 2 节点，用于开源 PX4 无人机在机载计算机上的前向障碍物感知、点云处理、栅格融合、告警输出和飞控刹停避障控制。

本项目面向 RK3588 机载计算机部署，雷达通过 SocketCAN 接入，PX4 通过 MAVLink 串口接入。当前版本已经加入 PX4 横滚和俯仰姿态补偿，用于降低低空飞行时机头下俯导致毫米波雷达照射地面并误判障碍物的概率。

## 功能概览

- 通过 SocketCAN 接收毫米波雷达原始点云。
- 解析雷达距离、方位角、俯仰角和回波功率，生成 `X右、Y前、Z下` 的雷达坐标点。
- 使用静态 TF 完成 `radar -> base_link` 坐标变换，保留雷达安装角补偿。
- 通过 MAVLink 串口读取 PX4 heartbeat、全局高度、落地状态、姿态等状态。
- 根据 PX4 `ATTITUDE` 的 roll/pitch 对点云做动态姿态补偿。
- 按功率、距离、水平 FOV、垂直角度和 ROI 过滤点云。
- 对点云进行聚类、栅格地图融合和时序衰减。
- 输出障碍物告警、点云、聚类可视化和占据栅格。
- 向 PX4 发送 `OBSTACLE_DISTANCE`。
- 当满足避障条件时，通过 MAVLink 命令切换 PX4 到位置控制刹停模式。
- 同时支持向自研飞控串口输出避障状态协议帧。

## 硬件与坐标系

### 硬件连接

默认配置如下：

| 设备 | 默认接口 | 说明 |
| --- | --- | --- |
| 毫米波雷达 | `can0` | SocketCAN 接口 |
| 自研飞控/外部控制器 | `/dev/ttyS5` @ `460800` | 输出避障状态帧 |
| PX4 飞控 | `/dev/ttyS9` @ `115200` | MAVLink 通信 |

这些接口可在 `config/obstacle_avoidance.yaml` 或 launch 参数中修改。

### 雷达原始坐标系

雷达安装在无人机机头，朝向正前方。雷达传感器原始数据坐标约定为：

| 轴 | 正方向 |
| --- | --- |
| X | 向右 |
| Y | 向前 |
| Z | 向下 |

雷达带约 5 度仰角安装。当前 launch 文件中使用静态 TF 发布 `base_link -> radar`，默认参数：

```text
radar_x     = 0.00
radar_y     = -0.758
radar_z     = 0.00
radar_roll  = 0.08
radar_pitch = 0.0
radar_yaw   = 0.0
```

注意：由于本项目点云坐标采用 `X右、Y前、Z下`，安装仰角在当前 TF 中体现在绕 X 轴的旋转上。

## 姿态动态补偿

低空前飞时，如果飞机俯角过大，前向毫米波雷达会照射到地面，地面回波可能落入避障 ROI 并触发错误刹停。本项目通过 PX4 MAVLink `ATTITUDE` 消息读取实时 roll/pitch，并在点云进入滤波器之前进行姿态补偿。

处理顺序：

1. 接收雷达点云。
2. 发布原始 `mmwave/raw_pointcloud`，用于对照雷达原始坐标。
3. 查询静态 TF，完成 `radar -> base_link`。
4. 使用 PX4 roll/pitch 对 `base_link` 点云做动态补偿。
5. 进入 ROI 过滤、聚类、栅格融合和决策。

补偿公式：

```cpp
p_comp = R_roll(-roll) * R_pitch(-pitch) * p_base
```

当前约定：

- pitch 对应绕点云 X 轴补偿。
- roll 对应绕点云 Y 轴补偿。
- yaw 不参与补偿，以保持前向避障栅格方向语义稳定。
- 如果姿态未收到或超时，节点退回未补偿点云并节流打印告警，不中断避障流程。

相关参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_attitude_compensation` | `true` | 是否启用 PX4 姿态补偿 |
| `attitude_timeout` | `0.3` | 姿态超时时间，单位秒 |
| `attitude_stream_hz` | `20.0` | 请求 PX4 发布 `ATTITUDE` 的频率 |
| `pitch_compensation_sign` | `1.0` | pitch 补偿符号，实机方向反了可改为 `-1.0` |
| `roll_compensation_sign` | `1.0` | roll 补偿符号，实机方向反了可改为 `-1.0` |

## 软件依赖

项目为 ROS 2 `ament_cmake` C++ 包，主要依赖：

- ROS 2
- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `pcl_conversions`
- `tf2`
- `tf2_ros`
- `tf2_eigen`
- `PCL`
- `Eigen3`
- Linux SocketCAN
- MAVLink C headers，项目已包含在 `include/mavlink/v2.0`

建议在 Ubuntu/RK3588 ROS 2 环境中编译运行。

## 编译

将本包放入 ROS 2 工作空间的 `src` 目录，例如：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repository-url> mmwave_obstacle_avoidance_ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mmwave_obstacle_avoidance_ros2
source install/setup.bash
```

如果项目目录已经在本机，可直接从工作空间根目录构建：

```bash
colcon build --packages-select mmwave_obstacle_avoidance_ros2
source install/setup.bash
```

## 运行

默认启动：

```bash
ros2 launch mmwave_obstacle_avoidance_ros2 obstacle_avoidance.launch.py
```

指定接口：

```bash
ros2 launch mmwave_obstacle_avoidance_ros2 obstacle_avoidance.launch.py \
  radar_can_interface:=can0 \
  self_dev_port:=/dev/ttyS5 \
  self_dev_baud:=460800 \
  px4_port:=/dev/ttyS9 \
  px4_baud:=115200
```

启用 RViz：

```bash
ros2 launch mmwave_obstacle_avoidance_ros2 obstacle_avoidance.launch.py rviz:=true
```

修改雷达安装 TF：

```bash
ros2 launch mmwave_obstacle_avoidance_ros2 obstacle_avoidance.launch.py \
  radar_x:=0.00 \
  radar_y:=-0.758 \
  radar_z:=0.00 \
  radar_roll:=0.08 \
  radar_pitch:=0.0 \
  radar_yaw:=0.0
```

## CAN 与串口准备

### SocketCAN

示例命令：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

确认 CAN 数据：

```bash
candump can0
```

### 串口权限

如果串口无权限：

```bash
sudo usermod -aG dialout $USER
```

重新登录后生效。也可以临时修改权限：

```bash
sudo chmod 666 /dev/ttyS5 /dev/ttyS9
```

## ROS 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `mmwave/raw_pointcloud` | `sensor_msgs/msg/PointCloud2` | 雷达原始点云，frame 为 `radar` |
| `mmwave/filtered_pointcloud` | `sensor_msgs/msg/PointCloud2` | 经过静态 TF 和姿态补偿后的过滤点云，frame 为 `base_link` |
| `mmwave/clustered_pointcloud` | `sensor_msgs/msg/PointCloud2` | 聚类后的点云 |
| `mmwave/grid_map` | `nav_msgs/msg/OccupancyGrid` | 前向占据栅格 |
| `mmwave/obstacle_warning` | `mmwave_obstacle_avoidance_ros2/msg/ObstacleWarning` | 避障告警结果 |
| `mmwave/cluster_markers` | `visualization_msgs/msg/MarkerArray` | 聚类包围盒可视化 |

## 自定义消息

### `ObstacleWarning`

用于发布避障决策结果，包含：

- `obstacle_detected`
- `min_distance`
- `closest_point`
- `warning_level`
- `obstacle_count`
- `sensor_status`

### `ClusterInfo`

用于描述聚类结果，包含聚类 ID、中心点、体积、点数和边界。

### `GridMapInfo`

用于描述栅格地图元信息和占据概率数组。

## PX4 MAVLink 行为

节点通过 PX4 串口接收：

- `HEARTBEAT`：获取 PX4 模式、解锁状态、目标 sysid/compid。
- `GLOBAL_POSITION_INT`：获取相对高度。
- `EXTENDED_SYS_STATE`：判断是否在空中。
- `ATTITUDE`：获取 roll/pitch 用于点云补偿。
- `COMMAND_ACK`：确认模式切换命令结果。

节点向 PX4 发送：

- companion computer heartbeat。
- `OBSTACLE_DISTANCE`。
- `MAV_CMD_SET_MESSAGE_INTERVAL`，请求 PX4 发布 `ATTITUDE`。
- `MAV_CMD_DO_SET_MODE`，用于触发位置控制刹停。

## 关键参数

配置文件：`config/obstacle_avoidance.yaml`

### 接口参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `radar_can_interface` | `can0` | 雷达 CAN 接口 |
| `self_dev_port` | `/dev/ttyS5` | 自研飞控/外部控制器串口 |
| `self_dev_baud` | `460800` | 自研串口波特率 |
| `px4_port` | `/dev/ttyS9` | PX4 MAVLink 串口 |
| `px4_baud` | `115200` | PX4 MAVLink 波特率 |

### 点云过滤参数

| 参数 | 说明 |
| --- | --- |
| `min_power_threshold` | 最小回波功率阈值 |
| `high_power_threshold` | 高功率点增强阈值 |
| `direct_map_power_threshold` | 高置信度直接入图阈值 |
| `high_power_multiplier` | 高功率点扩增倍数 |
| `min_range` / `max_range` | 前向距离范围 |
| `min_x` / `max_x` | 横向 ROI 范围 |
| `min_z` / `max_z` | 垂向 ROI 范围 |
| `max_fov_deg` | 水平 FOV 限制 |
| `min_elevation_deg` / `max_elevation_deg` | 垂直角限制 |

### 聚类与栅格参数

| 参数 | 说明 |
| --- | --- |
| `sor_mean_k` | 统计滤波邻居数量 |
| `sor_stddev_mul` | 统计滤波标准差倍数 |
| `cluster_tolerance_base` | 近距离聚类基础容差 |
| `cluster_tolerance_k` | 距离相关容差系数 |
| `cluster_max_tolerance` | 聚类容差上限 |
| `min_cluster_size` / `max_cluster_size` | 聚类点数限制 |
| `grid_resolution` | 栅格分辨率 |
| `grid_width` / `grid_height` | 栅格范围 |
| `occupancy_threshold` | 占据判定阈值 |
| `decay_rate` / `decay_time` | 时序衰减参数 |

### 决策与 PX4 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `emergency_distance` | `5.0` | 紧急距离 |
| `danger_distance` | `10.0` | 危险距离 |
| `caution_distance` | `57.0` | 警戒距离 |
| `min_continuous_frames` | `3` | 连续检测帧数 |
| `warning_timeout` | `0.5` | 告警超时 |
| `hold_trigger_distance` | `17.0` | 触发 PX4 刹停模式切换的距离 |
| `min_avoidance_alt` | `2.0` | 最小启用避障高度 |
| `base_mode_para` | `129.0` | PX4 模式命令参数 |
| `main_mode_para` | `3.0` | PX4 主模式参数，默认 POSCTL |
| `sub_mode_para` | `0.0` | PX4 子模式参数 |

## 实机调试建议

1. 先确认 CAN 雷达数据持续输入。
2. 确认 PX4 串口 heartbeat 正常，日志中能看到 `[PX4-Heart]`。
3. 确认姿态流正常，打开 debug 日志时能看到 `[PX4-ATT]`。
4. 在 RViz 中同时观察 `mmwave/raw_pointcloud` 和 `mmwave/filtered_pointcloud`。
5. 低空悬停、平飞、前飞下俯分别测试地面点是否被正确压低或过滤。
6. 如果补偿方向相反，不改代码，优先调整：

```yaml
pitch_compensation_sign: -1.0
roll_compensation_sign: -1.0
```

7. 如果 PX4 姿态消息偶发超时，可适当增大：

```yaml
attitude_timeout: 0.5
attitude_stream_hz: 30.0
```

## 常见问题

### 1. 启动后提示没有收到 ATTITUDE

检查 PX4 MAVLink 串口是否允许 companion computer 请求消息流；确认 `/dev/ttyS9`、波特率和 PX4 MAVLink instance 配置一致。

### 2. 低空下俯仍然误判地面

优先检查：

- 雷达静态安装角是否正确。
- `pitch_compensation_sign` 是否需要反向。
- `min_z`、`max_z`、`min_elevation_deg`、`max_elevation_deg` 是否过宽。
- `min_avoidance_alt` 是否符合实飞高度。

### 3. PX4 不执行刹停模式切换

检查：

- PX4 是否已解锁。
- `EXTENDED_SYS_STATE` 是否显示在空中。
- 当前高度是否大于 `min_avoidance_alt`。
- PX4 是否接受 `MAV_CMD_DO_SET_MODE`。
- `main_mode_para` 是否匹配目标 PX4 模式。

### 4. 运行时没有点云

检查：

- `can0` 是否 up。
- 雷达 CAN ID 是否与代码中的 `0x60B` 一致。
- `min_power_threshold` 是否过高。
- ROI 参数是否过窄。

## 项目结构

```text
.
├── CMakeLists.txt
├── package.xml
├── config/
│   └── obstacle_avoidance.yaml
├── include/
│   ├── mavlink/
│   └── mmwave_obstacle_avoidance_ros2/
├── launch/
│   └── obstacle_avoidance.launch.py
├── msg/
│   ├── ClusterInfo.msg
│   ├── GridMapInfo.msg
│   └── ObstacleWarning.msg
├── scripts/
└── src/
    ├── can_protocol.cpp
    ├── decision_controller.cpp
    ├── grid_map_fusion.cpp
    ├── obstacle_avoidance_node.cpp
    ├── point_cloud_filter.cpp
    └── spatial_clustering.cpp
```

## 版本说明

当前包版本：`2.0.0`

主要能力：

- PX4 MAVLink 串口控制。
- 前向毫米波雷达避障。
- 动态 roll/pitch 姿态补偿。
- `OBSTACLE_DISTANCE` 发布。
- 位置控制模式刹停触发。

## License

MIT
