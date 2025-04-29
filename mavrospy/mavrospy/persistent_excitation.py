#! /usr/bin/env python3

import time
import rclpy
import random
import threading
from mavrospy.control_node import MavrospyController

def main():
    """
    Move UAV in random directions to generate persistent excitation
    """
    rclpy.init()

    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    controller = MavrospyController(rate) # create mavrospy controller instance

    # Spin controller node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    # Set center position
    center_position = [1.5, 1.5, 1.5]  # X, Y, Z in meters

    # Movement bounds
    position_bounds = 1  # ±1m in X and Y
    altitude_bounds = 0.5  # ±0.5m from start altitude
    orientation_bounds = 1  # ±1 radians

    # Flight time
    flight_time = 5*60  # seconds

    # Wait until the drone is in OFFBOARD mode
    while rclpy.ok():
        if controller.current_state.mode == "OFFBOARD":
            controller.get_logger().info("OFFBOARD mode enabled")
            break

        # Stream setpoints before entering OFFBOARD mode
        controller.goto_xyz_rpy(0.0, 0.0, 0.0, 0, 0, 0, timeout=1, isClose=False, checkMode=False)

    # Takeoff
    controller.get_logger().info(f"Takeoff: {center_position[2]}m")
    controller.takeoff(center_position[2])

    # Go to center
    controller.get_logger().info("Going to center...")
    controller.slow_goto_xyz_rpy(center_position[0], center_position[1], center_position[2], 0, 0, 0)

    # Start timer
    start_time = time.time()

    controller.get_logger().info("Starting persistent excitation...")

    # Fly in random directions for flight_time seconds
    while rclpy.ok() and time.time() - start_time < flight_time:
        # Generate random target position and orientation
        target_x = center_position[0] + random.uniform(-position_bounds, position_bounds)
        target_y = center_position[1] + random.uniform(-position_bounds, position_bounds)
        target_z = center_position[2] + random.uniform(-altitude_bounds, altitude_bounds)
        target_yaw = random.uniform(-orientation_bounds, orientation_bounds)

        # Go to target position and orientation
        controller.goto_xyz_rpy(target_x, target_y, target_z, 0, 0, target_yaw)

    # Land
    controller.get_logger().info("Landing...")
    controller.land()

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
