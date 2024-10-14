from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from pathlib import Path

from rosbags.typesys import get_types_from_idl, get_types_from_msg

# Plain dictionary to hold message definitions.
add_types = {}

# Read definitions to python strings.
msg_text = Path('motion_capture_tracking_interfaces/msg/NamedPose.msg').read_text()

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'motion_capture_tracking_interfaces/msg/NamedPose'))

# Read definitions to python strings.
msg_text = Path('motion_capture_tracking_interfaces/msg/NamedPoseArray.msg').read_text()
add_types.update(get_types_from_msg(msg_text, 'motion_capture_tracking_interfaces/msg/NamedPoseArray'))

typestore = get_typestore(Stores.ROS2_HUMBLE)
typestore.register(add_types)

# Create a typestore and get the string class.
#typestore = get_typestore(Stores.LATEST)

# Create reader instance and open for reading.
with Reader('/home/analysis/rosbag2_2024_10_13-16_52_41') as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/poses':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print(str(msg.poses[0].pose.position.x) + "\n")

    # The .messages() method accepts connection filters.
#    connections = [x for x in reader.connections if x.topic == '/poses']
#    for connection, timestamp, rawdata in reader.messages(connections=connections):
#        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
#        print(msg.header.frame_id)
