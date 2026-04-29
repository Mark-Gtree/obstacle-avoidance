from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mmwave_obstacle_avoidance_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'obstacle_avoidance.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'obstacle_avoidance.rviz')
    radar_x_arg = DeclareLaunchArgument('radar_x', default_value='0.00') 
    radar_y_arg = DeclareLaunchArgument('radar_y', default_value='-0.758')
    radar_z_arg = DeclareLaunchArgument('radar_z', default_value='0.00')
    radar_roll_arg = DeclareLaunchArgument('radar_roll', default_value='0.08')
    radar_pitch_arg = DeclareLaunchArgument('radar_pitch', default_value='0.0')
    radar_yaw_arg = DeclareLaunchArgument('radar_yaw', default_value='0.0')
    
    # 声明launch参数
    radar_can_interface_arg = DeclareLaunchArgument(
        'radar_can_interface',
        default_value='can0',
        description='Radar CAN interface name'
    )
    
    self_dev_port_arg = DeclareLaunchArgument(
        'self_dev_port',
          default_value='/dev/ttyS5')

    self_dev_baud_arg = DeclareLaunchArgument(
        'self_dev_baud', 
        default_value='460800')
    
    px4_port_arg = DeclareLaunchArgument(
        'px4_port',
        default_value='/dev/ttyS9')
    
    px4_baud_arg = DeclareLaunchArgument(
        'px4_baud', 
        default_value='115200')

    enable_viz_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization topics'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2'
    )
    
    # 避障节点
    obstacle_avoidance_node = Node(
        package='mmwave_obstacle_avoidance_ros2',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance',
        output='screen',
        parameters=[
            config_file,
            {
                'radar_can_interface': LaunchConfiguration('radar_can_interface'),
                'self_dev_port': LaunchConfiguration('self_dev_port'),
                'self_dev_baud': LaunchConfiguration('self_dev_baud'),
                'px4_port': LaunchConfiguration('px4_port'),
                'px4_baud': LaunchConfiguration('px4_baud'),
                'enable_visualization': LaunchConfiguration('enable_visualization'),
            }
        ]
    )
    
    # TF静态变换（雷达到机体）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_to_base_link',
        arguments=[
            LaunchConfiguration('radar_x'), LaunchConfiguration('radar_y'), LaunchConfiguration('radar_z'),
            LaunchConfiguration('radar_yaw'), LaunchConfiguration('radar_pitch'), LaunchConfiguration('radar_roll'), 
            'base_link', 'radar'
        ]
    )
    
    def launch_rviz(context, *args, **kwargs):
        rviz_enabled = LaunchConfiguration('rviz').perform(context)
        if rviz_enabled.lower() == 'true':
            return [Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config]
            )]
        return []
    
    return LaunchDescription([
        radar_can_interface_arg,
        radar_x_arg, radar_y_arg, radar_z_arg,   
        radar_roll_arg, radar_pitch_arg, radar_yaw_arg,
        self_dev_port_arg,
        self_dev_baud_arg,
        px4_port_arg,
        px4_baud_arg,
        enable_viz_arg,
        rviz_arg,
        obstacle_avoidance_node,
        static_tf_node,
        OpaqueFunction(function=launch_rviz)
    ])