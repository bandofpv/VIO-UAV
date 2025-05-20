import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '/workspaces/isaac_ros-dev/VSLAM-UAV/sim/isaac_sim.cfg.rviz'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node
    ])
