# VSLAm-UAV
Visual SLAM Integration for UAVs

## IMU Calibration

In order to best take advantage of VSLAM, we must incorporate IMU data into the VSLAM pipeline. We can use the built in IMU of the Intel D435i camera. This requires the calibration of the IMU and the estimation of its parameters. The calibration of the IMU is done by estimating the bias, scale factor, and misalignment of the accelerometer and gyroscope.

To calibrate the IMU, we can use the IMU Calibration tool that comes with [Librealsense](https://github.com/IntelRealSense/librealsense). First, we need to build the `realsense` docker image:

```bash
$ cd VIO-UAV/docker/realsense
$ bash run_docker.sh
```

This will build the image and run it in a container. Once the container is running, we can run the IMU calibration tool:

```bash
$ cd ~/librealsense/tools/rs-imu-calibration
$ python3 rs-imu-calibration.py
```

Simply follow the command line instructions to calibrate the IMU. Below are images of the calibration process to help guide you through the process.

Position 1: Upright facing out

Position 2: USB cable facing out

Position 3: Upside down facing out

Position 4: USB cable pointed down

Position 5: Viewing direction facing down

Position 6: Viewing direction facing up

Make sure you write the calibration results to the camera's eeprom.

Note: if you get permission denied errors, you need to update the jetson uddev rules. Close the container (Ctrl+D) and run the following commands:

```bash
$ cd ~
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense/scripts
$ bash setup_udev_rules.sh
```

Follow any instructions that are printed out. Then, restart the container and try running the IMU calibration tool again.

## IMU Parameter Estimation

With the internal IMU calibrated, we need to estimate the parameters of the IMU. The parameters of the IMU are the accelerometer and gyroscope noise and bias instability. We can estimate these parameters by collecting data from the IMU and using [Allan Variance](https://en.wikipedia.org/wiki/Allan_variance) to estimate the parameters.

First, we need to record data from the IMU for an extended period of time (at least 3 hours). In the `realsense` container, open up the realsense ros nodes:

```bash
$ cd ~/VIU-UAV/bash
$ bash real-ros2.sh
```

This will open up the realsense ros nodes. Next, open up a new terminal and attach to the `realsense` container:

```bash
$ cd ~/VIO-UAV/docker/realsense
$ bash run_docker.sh
```

In the attached container, run the following command to record IMU data:

```bash
$ cd ~
$ ros2 bag record -o imu_rosbag /camera/imu
```

After at least 3 hours, stop the recording with **CTRL+C**.

Go to the terminal running the real-ros2.sh script and stop the realsense ros nodes with **CTRL+C**.

With our IMU data, we can now proceed to estimate the IMU parameters. This can be done easily within the ROS2 framework via [allan_ros2](https://github.com/CruxDevStuff/allan_ros2/tree/main).

First, we need to copy the IMU data to our desktop computer. Open a new terminal on your desktop computer and run the following command:

```bash
$ scp <username>@<jetson_ip>:/home/jetson/imu_rosbag/imu_rosbag.db3 ~/VIO-UAV/docker/analysis/imu_rosbag.db3
```

Where `<username>` is your username on the jetson and `<jetson_ip>` is the IP address.

## Check to see if the COPY works

Next, we need to build the `analysis` docker image on our desktop computer:

```bash
$ cd VIO-UAV/docker/analysis
$ bash run_docker.sh
```

This will build the image and run it in a container. Once the container is running, edit the allan_ros2 config file:

```bash
$ nano ~/allan_ws/src/allan_ros2/config/config.yaml
```

## Check if this works (name wise)

Modify the file to look like the following:

```yaml
allan_node:
  ros__parameters:
     topic: /camera/imu
     bag_path: /home/analysis/imu_rosbag/imu_rosbag.db3
     msg_type: ros
     publish_rate: 200
     sample_rate: 200
```

Next, we need to re-build the allan_ros2 package and run allan_node to compute the raw deviation values:

```bash
$ cd ~/allan_ws
$ colcon build --packages-select allan_ros2
$ source ~/allan_ws/install/setup.bash
$ ros2 launch allan_ros2 allan_node.py
```

After you get the `DONE` message, exit via **CTRL+C** and run the `analysis.py` script to compute the IMU parameters:

```bash
$ python3 src/allan_ros2/scripts/analysis.py --data deviation.csv
```

This will output the IMU parameters and save them to the `imu.yaml` file. I would recommend saving this file or at least the IMU parameters for future reference.

## SIMULATION INSTRUCTIONS???
