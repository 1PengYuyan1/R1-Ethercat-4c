import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 手柄驱动
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}]
        ),

        # 2. 解算节点（直接发布 /cmd_vel + /robot_buttons）
        #    max_speed/max_omega/deadzone 默认值在 remote_node.cpp 的 declare_parameter 里;
        #    临时调试用 `ros2 launch ... max_speed:=X` 命令行覆盖。
        Node(
            package='linkx_soem_demo',
            executable='remote_node_cpp',
            name='remote_node',
            output='screen',
        ),
    ])
