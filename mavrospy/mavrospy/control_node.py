#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from pymavlink import mavutil
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandHome
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MavrospyController(Node):
    """
    Controller class to help interface with MAVROS in ROS2
    """
    def __init__(self, frequency):
        super().__init__('mavrospy_control_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declare and retrieve parameters
        self.declare_parameter('vision', False)
        self.vision = self.get_parameter('vision').value

        # Create subscribers
        self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.extended_state_callback, qos_profile)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.origin_callback, qos_profile)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)

        # Create publishers
        self.cmd_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        self.origin_pub = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', qos_profile)

        # Create service clients
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # ROS messages
        self.pose = Pose()
        self.current_state = State()
        self.current_extended_state = ExtendedState()
        self.timestamp = self.get_clock().now()

        # Initialize constants
        self.pi_2 = math.pi / 2.0
        self.freq = frequency
        self.rate = self.create_rate(frequency)
        self.target_lat = 38.9853504
        self.target_lon = -76.4857648
        self.target_alt = 36.810

        # Status variables
        self.origin_set = False
        self.current_gps = NavSatFix()
        self.current_gps.status.status = NavSatStatus.STATUS_NO_FIX

        # Vision initialization
        if self.vision:
            self.set_global_origin()
            self.wait_for_gps_fix()
            self.set_home_position()

        self.get_logger().info("MavrospyController Initiated")

    def state_callback(self, msg):
        """
        Callback for PX4 state messages

        params: msg: State message
        """
        self.current_state = msg

    def extended_state_callback(self, msg):
        """
        Callback for PX4 extended state messages

        params: msg: ExtendedState message
        """
        self.current_extended_state = msg

    def pose_callback(self, msg):
        """
        Callback for PX4 local position messages

        params: msg: PoseStamped message
        """
        self.timestamp = msg.header.stamp
        self.pose = msg.pose

    def origin_callback(self, msg):
        """
        Callback for PX4 global origin messages

        params: msg: GeoPointStamped message
        """
        # Check if global origin is set to the target location
        if (msg.position.latitude == self.target_lat and
                msg.position.longitude == self.target_lon) or not self.vision:
            self.origin_set = True
        else:
            self.get_logger().warn(f"Global origin mismatch: Expected "
                                   f"({self.target_lat}, {self.target_lon}) but got "
                                   f"({msg.position.latitude}, {msg.position.longitude})")

    def gps_callback(self, msg):
        """
        Callback for PX4 GPS messages

        params: msg: NavSatFix message
        """
        self.current_gps = msg

    def set_global_origin(self):
        """
        Sets the global origin to the target latitude, longitude, and altitude
        """
        self.get_logger().info("Setting global origin...")
        while rclpy.ok():
            # Setup GeoPointStamped message
            origin_msg = GeoPointStamped()
            origin_msg.header.stamp = self.get_clock().now().to_msg()
            origin_msg.header.frame_id = "map"

            # Assign lat, long, and alt to the message
            origin_msg.position.latitude = self.target_lat
            origin_msg.position.longitude = self.target_lon
            origin_msg.position.altitude = self.target_alt

            # Publish the message
            self.origin_pub.publish(origin_msg)

            # Check if the global origin has been set
            if self.origin_set:
                self.get_logger().info("Global origin set.")
                break
            self.rate.sleep()

    def wait_for_gps_fix(self):
        """
        Waits for a GPS fix before continuing
        """
        self.get_logger().info("Waiting for GPS fix...")
        while rclpy.ok():
            if self.current_gps.status.status == NavSatStatus.STATUS_FIX:
                self.get_logger().info("GPS fix acquired.")
                break
            self.rate.sleep()

    def set_home_position(self):
        """
        Sets the home position to the current GPS position
        """
        self.get_logger().info("Setting home position...")
        req = CommandHome.Request()
        req.current_gps = True
        if self.set_home_client.call(req).success:
            self.get_logger().info("Home position set.")
        else:
            self.get_logger().error("Failed to set home position.")

    def arm(self, status):
        """
        Arm or disarm the throttle

        params: status: True to arm, False to disarm
        """
        self.get_logger().info(f"{'Arming' if status else 'Disarming'} throttle...")
        req = CommandBool.Request()
        req.value = status
        if self.arm_client.call(req).success:
            self.get_logger().info(f"{'Armed' if status else 'Disarmed'} throttle")
        else:
            self.get_logger().error(f"Failed to {'arm' if status else 'disarm'} throttle")

    def check_offboard(self):
        """
        Check if the vehicle is in offboard mode
        """
        if self.current_state.mode != "OFFBOARD":
            self.get_logger().error("Not in OFFBOARd mode.")
            return False

    def is_close(self, target, current, tolerance, quat=False):
            """
            Check if UAV is close to target set point

            params: target: target set point
                    current: current set point
                    tolerance: acceptable error
                    quat: True if comparing quaternions, False otherwise

            Recommended tolerances: point x & y --> 0.2
                                    point z --> 0.5
                                    quaternion x, y, z, & w --> 0.1
            """
            if quat:
                return abs(target - current) < tolerance or abs(target + current) < tolerance

            return abs(target - current) < tolerance


    def goto(self, pose):
        """
        Move the vehicle

        params: pose: Pose message to publish
        """
        # Iniitialize ROS PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        # Publish the message
        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw, timeout=30, isClose=True, checkMode=True, slow=False):
        """
        Move the vehicle to a position and orientation

        params: x: x-coordinate
                y: y-coordinate
                z: z-coordinate
                roll: roll angle
                pitch: pitch angle
                yaw: yaw angle
                timeout: time to reach set point
                isClose: True if checking if close to set point
                checkMode: True if checking if in OFFBOARD mode
                slow: True if moving slowly
        """
        # Check if in OFFBOARD mode
        if checkMode:
            self.check_offboard()

        # Initialize ROS Pose message
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z

        # Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        quaternion = quaternion_from_euler(roll, pitch, yaw + self.pi_2)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        # Move the vehicle to the desired position and orientation for the specified timeout
        for i in range(int(timeout * self.freq)):
            self.goto(pose)

            # Check if close to set point
            if (isClose
                and self.is_close(x, self.pose.position.x, 0.2)
                and self.is_close(y, self.pose.position.y, 0.2)
                and self.is_close(z, self.pose.position.z, 0.5)
                and self.is_close(quaternion[0], self.pose.orientation.x, 0.1, True)
                and self.is_close(quaternion[1], self.pose.orientation.y, 0.1, True)
                and self.is_close(quaternion[2], self.pose.orientation.z, 0.1, True)
                and self.is_close(quaternion[3], self.pose.orientation.w, 0.1, True)):
                    if (slow):
                        break
                    self.get_logger().info("Reached target position")
                    break

            self.rate.sleep()

    def slow_goto_xyz_rpy(self, x, y, z, roll, pitch, yaw, height=False, steps=90):
        """
        Move the drone slowly to a target position and orientation

        params: x: target x-coordinate
                y: target y-coordinate
                z: target z-coordinate
                roll: target roll angle
                pitch: target pitch angle
                yaw: target yaw angle
                height: True if moving in height
                steps: number of increments to reach the target
        """
        # Convert current orientation to euler angles
        current_quaternion = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (current_roll, current_pitch, current_yaw) = euler_from_quaternion(current_quaternion)

        # Generate incremental steps for smooth movement
        step_x = np.linspace(self.pose.position.x, x, steps)
        step_y = np.linspace(self.pose.position.y, y, steps)
        step_z = np.linspace(self.pose.position.z, z, steps)
        step_yaw = np.linspace(current_yaw - self.pi_2, yaw, steps)

        # Don't rotate if yaw is within 0.2 rad
        tolerance = 0.2
        if abs(yaw - current_yaw - self.pi_2) < tolerance or abs(yaw + current_yaw - self.pi_2) < tolerance:
            step_yaw = np.linspace(yaw, yaw, steps)

        # Move in small steps
        for i in range(steps):
            if height:
                self.goto_xyz_rpy(x, y, step_z[i], roll, pitch, step_yaw[i], 1/20, isClose=False, slow=True)
            self.goto_xyz_rpy(step_x[i], step_y[i], z, roll, pitch, step_yaw[i], 1/20, isClose=False, slow=True)
        self.get_logger().info("Reached target position")

    def takeoff(self, height):
        """
        Arm the throttle and takeoff to given height (feet)

        params: height: target height in feet
        """
        self.arm(True)
        self.get_logger().info("Taking Off...")
        self.slow_goto_xyz_rpy(0.0, 0.0, height, 0, 0, 0, height=True)

    def land(self):
        """
        Set mode to AUTO.LAND for immediate descent and disarm when on ground.
        """
        self.get_logger().info("Changing mode: AUTO.LAND...")
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        if self.mode_client.call(req).mode_sent:
            self.get_logger().info("Mode set: AUTO.LAND")
        else:
            self.get_logger().error("Failed to set mode: AUTO.LAND")

        # Loop until landed
        while rclpy.ok():
            if self.current_extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.get_logger().info("Landed")
                break
            self.rate.sleep()

        self.arm(False)  # disarm throttle
