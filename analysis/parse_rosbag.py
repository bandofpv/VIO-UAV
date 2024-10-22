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

gt_secs = []; gt_nsecs = []
gt_position_x = []; gt_position_y = []; gt_position_z = []
gt_orientation_x = []; gt_orientation_y = []; gt_orientation_z = []; gt_orientation_w = []

gt = []
vio =[]

# Create reader instance and open for reading.
with Reader('/home/analysis/VIO-UAV/analysis/data/' + str(args.name)) as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
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

        if connection.topic == '/visual_slam/tracking/vo_pose_covariance':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

            vio.append({
                'vio_secs': msg.header.stamp.sec,
                'vio_nsecs': msg.header.stamp.nanosec,
                'vio_position_x': msg.pose.pose.position.x, 
                'vio_position_y': msg.pose.pose.position.y,
                'vio_position_z': msg.pose.pose.position.z,
                'vio_orientation_x': msg.pose.pose.orientation.x,
                'vio_orientation_y': msg.pose.pose.orientation.y,
                'vio_orientation_z': msg.pose.pose.orientation.z,
                'vio_orientation_w': msg.pose.pose.orientation.w
                })

gt_df = pd.DataFrame(gt)
#print(gt_df.head())

vio_df = pd.DataFrame(vio)
#print(vio_df.head(10))

gt_df.to_csv('/home/analysis/VIO-UAV/analysis/data/' + str(args.name) + '/gt.csv', index=False)
vio_df.to_csv('/home/analysis/VIO-UAV/analysis/data/' + str(args.name) + '/vio.csv', index=False)
