#! /usr/bin/env python3

import math
import rclpy
import threading
from mavrospy.control_node import MavrospyController

def fly_circle(controller, radius, altitude, resolution=180):
    """
    Fly in a circle pattern facing only in forward direction
    """
    # Generate circle points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # angle from center to perimeter
        x = radius * math.cos(theta)  # calculate x position
        y = radius * math.sin(theta)  # calculate y position

        # Account for center offset
        x += radius
        y += radius

        controller.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)
    controller.get_logger().info("Circle pattern complete")

def main():
    """
    Move UAV in a circle pattern at given height and width for given
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
    width = 3.0  # width of the circle pattern
    radius = width / 2  # radius of circle pattern
    levels = 3  # number of different altitudes to complete circle pattern
    repetitions = 3  # number of times to repeat the circle pattern at each altitude

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / (levels-1) for l in range(levels)]

    # Wait until the drone is in OFFBOARD mode
    while rclpy.ok():
        if controller.current_state.mode == "OFFBOARD":
            controller.get_logger().info("OFFBOARD mode enabled")
            break

        # Stream setpoints before entering OFFBOARD mode
        controller.goto_xyz_rpy(0.0, 0.0, 0.0, 0, 0, 0, timeout=1, isClose=False, checkMode=False)

    # Takeoff at lowest altitude in altitudes list
    controller.get_logger().info(f"Takeoff: {altitudes[0]} meters")
    controller.takeoff(altitudes[0])

    # Go to center of circle
    controller.get_logger().info("Flying to center of circle...")
    controller.slow_goto_xyz_rpy(radius, radius, altitudes[0], 0, 0, 0)

    # Go to first point on circle
    controller.get_logger().info("Flying to radius of circle...")
    controller.slow_goto_xyz_rpy(radius*2, radius, altitudes[0], 0, 0, 0)

    controller.get_logger().info("Starting circle pattern...")

    # Fly circle pattern at all altitudes
    for altitude in altitudes:
        controller.get_logger().info(f"Pattern Altitude: {altitude} meters")
        for r in range(repetitions):  # repeat circle pattern
            fly_circle(controller, radius, altitude)

    # Go back to center of circle
    controller.get_logger().info("Returning to center of circle...")
    controller.slow_goto_xyz_rpy(radius, radius, altitudes[-1], 0, 0, 0)

    # Land
    controller.get_logger().info("Landing...")
    controller.land()

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
