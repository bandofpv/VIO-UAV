#!/usr/bin/env python3

import math
import rclpy
import threading
from mavrospy.control_node import MavrospyController

def fly_figure_eight(controller, width, length, altitude, resolution=360):
    """
    Fly in a figure-eight pattern facing only in forward direction
    """
    # Generate figure-eight points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # increment angle around unit circle
        x = width/2 * math.cos(theta)  # calculate x position
        y = length/2 * math.sin(2 * theta)  # calculate y position

        # Account for center offset
        x += width/2
        y += length/2

        controller.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)
    controller.get_logger().info("Figure-eight pattern complete")

def main():
    """
    Move UAV in a figure-eight pattern at given height and width for given
    repititions and altitude levels
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
    width = 4.0  # width of the figure-eight pattern
    length = 2.5  # length of the figure-eight pattern
    levels = 3  # number of different altitudes to complete figure-eight pattern
    repetitions = 3  # number of times to repeat the figure-eight pattern at each altitude

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

    # Go to center of figure-eight pattern
    controller.get_logger().info("Going to center of figure-eight pattern..")
    controller.slow_goto_xyz_rpy(width/2, length/2, altitudes[0], 0, 0, 0)

    # Go to first point on figure-eight pattern
    controller.get_logger().info("Going to radius of figure-eight pattern..")
    controller.slow_goto_xyz_rpy(width, length/2, altitudes[0], 0, 0, 0)

    controller.get_logger().info("Starting figure-eight pattern...")

    # Fly figure-eight pattern at all altitudes
    for altitude in altitudes:
        controller.get_logger().info(f"Pattern Altitude: {altitude} meters")
        for r in range(repetitions):  # repeat figure-eight pattern
            fly_figure_eight(controller, width, length, altitude)

    # Go back to center of figure-eight pattern
    controller.get_logger().info("Returning to center of figure-eight pattern...")
    controller.slow_goto_xyz_rpy(width/2, length/2, altitudes[-1], 0, 0, 0)

    # Land
    controller.get_logger().info("Landing...")
    controller.land()

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
