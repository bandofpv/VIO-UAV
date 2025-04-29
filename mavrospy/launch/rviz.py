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

    # Launch Pose to Path node
    pose_to_path_node = Node(
        package='mavrospy',
        executable='pose_to_path_py',
        name='pose_to_path',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'topic': '/mavros/local_position/pose'
        }]
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            os.getenv('AMENT_PREFIX_PATH').split(':')[0],
            'share',
            'mavrospy',
            'rviz',
            'sim.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        pose_to_path_node,
        rviz_node
    ])
