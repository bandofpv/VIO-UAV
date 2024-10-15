import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


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

# Create reader instance and open for reading.
with Reader('/home/analysis/rosbag-circle') as reader:
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

df = pd.DataFrame(gt)
print(df.head())
