#! /usr/bin/env python3

import math
import rclpy
import threading
from mavrospy.control_node import MavrospyController

def fly_spiral(controller, min_radius, max_radius, min_height, max_height, final_radius, resolution=180):
    """
    Fly in a spiral pattern facing only in forward direction
    """
    # Calculate step size for radius and height
    radius_step = (max_radius - min_radius) / resolution
    height_step = (max_height - min_height) / resolution

    # Generate spiral points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # angle from center to perimeter
        radius = min_radius + i * radius_step  # calculate radius
        altitude = min_height + i * height_step  # calculate altitude

        x = radius * math.cos(theta)  # calculate x position
        y = radius * math.sin(theta)  # calculate y position

        # Account for center offset
        x += final_radius
        y += final_radius

        controller.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)
    controller.get_logger().info("Spiral pattern complete")

def main():
    """
    Move UAV in a spiral pattern at given height and width
    """
    rclpy.init()

    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    controller = MavrospyController(rate) # create mavrospy controller instance

    # Spin controller node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()

    min_height = 1.0  # minimum height to fly at
    max_height = 3.0  # maximum height to fly at
    min_width = 0.0  # minimum width of spiral pattern
    max_width = 3.0  # maximum width of spiral pattern
    min_radius = min_width / 2  # minimum radius of spiral pattern
    max_radius = max_width / 2  # maximum radius of spiral pattern
    resolution = 180  # number of steps for smoothness
    levels = 10  # number different altitudes to complete spiral pattern

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / (levels-1) for l in range(levels)]
    radii = [min_radius + (max_radius - min_radius) * l / (levels-1) for l in range(levels)]

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

    # Go to center of spiral pattern
    controller.get_logger().info("Going to center of spiral...")
    controller.slow_goto_xyz_rpy(max_radius, max_radius, altitudes[0], 0, 0, 0)

    controller.get_logger().info("Starting spiral pattern...")

    # Fly spiral pattern at all altitudes and radii
    for i in range(len(altitudes)-1):
        controller.get_logger().info(f"Pattern Altitude: {altitudes[i]} meters")
        fly_spiral(controller, radii[i], radii[i+1], altitudes[i], altitudes[i+1], max_radius, resolution)

    # Go back to the center of the spiral pattern
    controller.get_logger().info("Returning to center of spiral...")
    controller.slow_goto_xyz_rpy(max_radius, max_radius, altitudes[-1], 0, 0, 0)

    # Land
    controller.get_logger().info("Landing...")
    controller.land()

    # Shutdown
    controller.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == "__main__":
    main()
