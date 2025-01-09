import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Isaac Sim."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('visual_slam/image_0', 'camera/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_0', 'camera/infra1/camera_info'),
                    ('visual_slam/image_1', 'camera/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_1', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
        parameters=[{
            'use_sim_time': True,
            'enable_image_denoising': True,
            'rectified_images': True,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': True,
        }])

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
        ],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container])
