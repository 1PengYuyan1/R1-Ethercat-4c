from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ifname = LaunchConfiguration('ifname')
    start_vehicle_control = LaunchConfiguration('start_vehicle_control')
    vehicle_prefix = LaunchConfiguration('vehicle_prefix')
    ros_nodes_prefix = LaunchConfiguration('ros_nodes_prefix')

    return LaunchDescription([
        DeclareLaunchArgument('ifname', default_value='enp86s0'),
        DeclareLaunchArgument('start_vehicle_control', default_value='true'),
        DeclareLaunchArgument('vehicle_prefix', default_value=''),
        DeclareLaunchArgument('ros_nodes_prefix', default_value=''),

        # 1) 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            prefix=ros_nodes_prefix,
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
            output='screen',
        ),

        # 2) 遥控解算节点（直接发布 /cmd_vel + /robot_buttons）
        #    max_speed/max_omega/deadzone 默认值在 remote_node.cpp 的 declare_parameter 里;
        #    临时调试用 `ros2 launch ... max_speed:=X` 命令行覆盖。
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            prefix=ros_nodes_prefix,
            output='screen',
        ),

        # 3) 整车主控程序（可选）
        Node(
            package='linkx_soem_demo',
            executable='linkx_soem_demo',
            name='vehicle_control',
            output='screen',
            condition=IfCondition(start_vehicle_control),
            prefix=vehicle_prefix,
            arguments=[ifname],
        ),
    ])
