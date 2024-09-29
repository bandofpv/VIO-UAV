ros2 bag record -o rosbag-test --include-hidden-topics \
    /camera/infra1/camera_info /camera/infra1/image_rect_raw /camera/infra1/metadata \
    /camera/infra2/camera_info /camera/infra2/image_rect_raw /camera/infra2/metadata \
    /tf_static /tf \
    /camera/imu \
    /qualisys/My_Quad/pose
