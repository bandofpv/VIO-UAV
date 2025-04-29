import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument

def generate_launch_description():
    # Declare the fcu_url argument
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='URL for MAVROS FCU connection'
    )

    # Declare the pattern argument
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='square',
        description='Movement pattern to execute (e.g., square)'
    )

    # Define the path to the mavrospy executable
    mavrospy_executable = [LaunchConfiguration('pattern'), TextSubstitution(text="_py")]

    # Path to the px4.launch file in MAVROS and PX4-Autopilot
    px4_launch_path = os.path.expanduser('~/ros2_ws/install/mavros/share/mavros/launch/px4.launch')

    # Launch px4.launch with the fcu_url argument
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(px4_launch_path),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
    )

    # Launch mavrospy node
    mavrospy_node = Node(
        package='mavrospy',
        executable=mavrospy_executable,
        name='control_node',
        output='screen',
    )

    # Build the launch description
    return LaunchDescription([
        fcu_url_arg,
        pattern_arg,
        mavros_node,
        mavrospy_node
    ])
