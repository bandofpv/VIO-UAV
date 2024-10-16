import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_image_denoising': False,
                    'rectified_images': True,
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.00015409893096,
                    'gyro_random_walk': 0.00000739592221,
                    'accel_noise_density': 0.00185259081322,
                    'accel_random_walk': 0.00018498030363,
                    'calibration_frequency': 200.0,
                    'image_jitter_threshold_ms': 22.00,
                    'base_frame': 'camera_link',
                    'imu_frame': 'camera_gyro_optical_frame',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'camera_optical_frames': [
                        'camera_infra1_optical_frame',
                        'camera_infra2_optical_frame',
                    ],
                    }],
        remappings=[('visual_slam/image_0', 'camera/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_0', 'camera/infra1/camera_info'),
                    ('visual_slam/image_1', 'camera/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_1', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])
