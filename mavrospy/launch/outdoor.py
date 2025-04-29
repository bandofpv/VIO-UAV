from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the fcu_url argument
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/serial0:921600',
        description='FCU URL for MAVROS PX4 connection'
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
    ),

    # Launch mavrospy with specified pattern
    mavrospy_node = Node(
        package='mavrospy',
        executable=mavrospy_executable,
        name='control_node',
        output='screen',
    )

    # Build the launch description
    return LaunchDescription([
        pattern_arg,
        fcu_url_arg,
        mavros_node,
        mavrospy_node
    ])
