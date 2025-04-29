#!/usr/bin/env python3

import rclpy
import threading
from mavrospy.control_node import MavrospyController

def fly_square(controller, width, altitude):
    """
    Fly in a square pattern facing only in the forward direction.
    """
    controller.get_logger().info("Waypoint 1")
    controller.slow_goto_xyz_rpy(width, 0.0, altitude, 0, 0, 0)
    controller.get_logger().info("Waypoint 2")
    controller.slow_goto_xyz_rpy(width, width, altitude, 0, 0, 0)
    controller.get_logger().info("Waypoint 3")
    controller.slow_goto_xyz_rpy(0.0, width, altitude, 0, 0, 0)
    controller.get_logger().info("Waypoint 4")
    controller.slow_goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 0)
    controller.get_logger().info("Square pattern complete")

def main():
    """
    Move UAV in a square pattern at given height and width for given
    repetitions and altitude levels.
    """
    rclpy.init()

    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    controller = MavrospyController(rate) # create mavrospy controller instance

    # Spin controller node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    min_height = 1.0  # min height to fly at
    max_height = 3.0  # max height to fly at
    width = 3.0  # width of the square pattern
    levels = 3  # number of different altitudes to complete square pattern
    repetitions = 2   # number of times to repeat the square pattern at each altitude

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / (levels-1) for l in range(levels)]

    # Wait until the drone is in OFFBOARD mode
    while rclpy.ok():
        if controller.current_state.mode == "OFFBOARD":
            controller.get_logger().info("OFFBOARD mode enabled")
            break

        # Stream setpoints before entering OFFBOARD mode
        controller.goto_xyz_rpy(0.0, 0.0, 0.0, 0, 0, 0, timeout=1, isClose=False, checkMode=False)

    # Takeoff at the lowest altitude
    controller.get_logger().info(f"Takeoff: {altitudes[0]} meters")
    controller.takeoff(altitudes[0])

    controller.get_logger().info("Starting square pattern...")

    # Fly square patterns at each altitude
    for altitude in altitudes:
        controller.get_logger().info(f"Pattern Altitude: {altitude} meters")
        controller.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 0)  # reset to origin
        for r in range(repetitions):  # repeat square pattern
            fly_square(controller, width, altitude)

    # Land
    controller.get_logger().info("Landing...")
    controller.land()

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
