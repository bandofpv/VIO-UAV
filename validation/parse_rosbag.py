import argparse
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


# Parse Name Argument
parser = argparse.ArgumentParser()
parser.add_argument("--name", help="name of rosbag")
args = parser.parse_args()

# Plain dictionary to hold message definitions
add_types = {}

# Read NamedPose message definition to python strings
msg_text = Path('motion_capture_tracking_interfaces/msg/NamedPose.msg').read_text()

# Add definition from msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'motion_capture_tracking_interfaces/msg/NamedPose'))

# Repeat for NamedPoseArray
msg_text = Path('motion_capture_tracking_interfaces/msg/NamedPoseArray.msg').read_text()
add_types.update(get_types_from_msg(msg_text, 'motion_capture_tracking_interfaces/msg/NamedPoseArray'))

# Load and register ros2 humble and custom ros messages
typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

# Initialize lists to hold ground truth and visual SLAM data
gt = []; vslam =[]
gt_secs = []; gt_nsecs = []
gt_position_x = []; gt_position_y = []; gt_position_z = []
gt_orientation_x = []; gt_orientation_y = []; gt_orientation_z = []; gt_orientation_w = []

# Create reader instance and open for reading.
with Reader('/home/analysis/VSLAM-UAV/validation/' + str(args.name)) as reader:
    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        # Ground truth
        if connection.topic == '/poses':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            gt.append({
                'gt_secs': msg.header.stamp.sec,
                'gt_nsecs': msg.header.stamp.nanosec,
                'gt_position_x': msg.poses[0].pose.position.x,
                'gt_position_y': msg.poses[0].pose.position.y,
                'gt_position_z': msg.poses[0].pose.position.z,
                'gt_orientation_x': msg.poses[0].pose.orientation.x,
                'gt_orientation_y': msg.poses[0].pose.orientation.y,
                'gt_orientation_z': msg.poses[0].pose.orientation.z,
                'gt_orientation_w': msg.poses[0].pose.orientation.w
                })
        # Visual SLAM
        if connection.topic == '/visual_slam/tracking/vo_pose_covariance':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            vslam.append({
                'vslam_secs': msg.header.stamp.sec,
                'vslam_nsecs': msg.header.stamp.nanosec,
                'vslam_position_x': msg.pose.pose.position.x,
                'vslam_position_y': msg.pose.pose.position.y,
                'vslam_position_z': msg.pose.pose.position.z,
                'vslam_orientation_x': msg.pose.pose.orientation.x,
                'vslam_orientation_y': msg.pose.pose.orientation.y,
                'vslam_orientation_z': msg.pose.pose.orientation.z,
                'vslam_orientation_w': msg.pose.pose.orientation.w
                })

# Convert lists to pandas DataFrames
gt_df = pd.DataFrame(gt)
vslam_df = pd.DataFrame(vslam)

# Save DataFrames to CSV files
gt_df.to_csv('/home/analysis/VSLAM-UAV/validation/' + str(args.name) + '/ground_truth.csv', index=False)
vslam_df.to_csv('/home/analysis/VSLAM-UAV/validation/' + str(args.name) + '/vslam_estimates.csv', index=False)

print(f"Ground truth and visual SLAM data saved to the {args.name} directory.")
