# Import Libraries and Mount Google Drive
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation

# Load Ground Truth and VSLAM .csv Files
gt_df = pd.read_csv('/home/analysis/VSLAM-UAV/validation/validation_data/ground_truth.csv')
vslam_df = pd.read_csv('/home/analysis/VSLAM-UAV/validation/validation_data/vslam_estimates.csv')

"""Zero out initial ground truth position"""

# Average first 2 seconds to grab initial condition
average_x = np.mean(gt_df['gt_position_x'].iloc[:200])
average_y = np.mean(gt_df['gt_position_y'].iloc[:200])
average_z = np.mean(gt_df['gt_position_z'].iloc[:200])

# Zero out position
gt_df['gt_position_x'] = gt_df['gt_position_x'] - average_x
gt_df['gt_position_y'] = gt_df['gt_position_y'] - average_y
gt_df['gt_position_z'] = gt_df['gt_position_z'] - average_z

"""Convert Quaternions to Euler Angles"""

# Locate x, y, z, and w orientations
gt_quat_df = gt_df.loc[:,['gt_orientation_x', 'gt_orientation_y', 'gt_orientation_z', 'gt_orientation_w']]

# Create a rotation object from the ground truth quaternion
gt_rotation = Rotation.from_quat(gt_quat_df)

# Convert the ground truth quaternion to Euler angles (in degrees)
gt_euler = gt_rotation.as_euler('xyz', degrees=True)

# Convert to a DataFrame
gt_euler_df = pd.DataFrame(gt_euler, columns=['gt_roll', 'gt_pitch', 'gt_yaw'])

# Concatenate to main DataFrame
gt_df = pd.concat([gt_df, gt_euler_df], axis=1)

# Repeat for vslam quaternion
vslam_quat_df = vslam_df.loc[:,['vslam_orientation_x', 'vslam_orientation_y', 'vslam_orientation_z', 'vslam_orientation_w']]
vslam_rotation = Rotation.from_quat(vslam_quat_df)
vslam_euler = vslam_rotation.as_euler('xyz', degrees=True)
vslam_euler_df = pd.DataFrame(vslam_euler, columns=['vslam_roll', 'vslam_pitch', 'vslam_yaw'])
vslam_df = pd.concat([vslam_df, vslam_euler_df], axis=1)

"""Filter out drop outs in yaw"""

# Apply a rolling median filter to smooth noisy data
gt_df['gt_yaw_smoothed'] = gt_df['gt_yaw'].rolling(window=150, center=True).median()

# Forward fill and backward fill any NaN values from the rolling window
gt_df['gt_yaw_smoothed'] = gt_df['gt_yaw_smoothed'].bfill().ffill()

"""Add `secs` and `nanosecs` into a singular time series"""

# Create time series
gt_df['gt_time'] = gt_df['gt_secs'] + gt_df['gt_nsecs'] / 1e9
vslam_df['vslam_time'] = vslam_df['vslam_secs'] + vslam_df['vslam_nsecs'] / 1e9

# zero out time series
gt_df['gt_time'] = gt_df['gt_time'] - gt_df['gt_time'].iloc[0]
vslam_df['vslam_time'] = vslam_df['vslam_time'] - vslam_df['vslam_time'].iloc[0]

"""Convert DataFrames to Numpy Arrays"""

gt_position_x = gt_df['gt_position_x'].values
gt_position_y = gt_df['gt_position_y'].values
gt_position_z = gt_df['gt_position_z'].values

vslam_position_x = vslam_df['vslam_position_x'].values
vslam_position_y = vslam_df['vslam_position_y'].values
vslam_position_z = vslam_df['vslam_position_z'].values

gt_yaw_smoothed = gt_df['gt_yaw_smoothed'].values
vslam_yaw = vslam_df['vslam_yaw'].values

gt_time = gt_df['gt_time'].values
vslam_time = vslam_df['vslam_time'].values

"""Plots"""

# Plot the Rotated vslam and Ground Truth
fig = plt.figure(1, figsize=(12, 8))
ax = fig.add_subplot(projection='3d')
fig.suptitle("Ground Truth vs. VSLAM Position", fontsize=20)
ax.plot(gt_position_x, gt_position_y, gt_position_z)
ax.plot(vslam_position_x, vslam_position_y, vslam_position_z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend(['Ground Truth', 'VSLAM'], fontsize=12)

# Plot X Position vs. Time
plt.figure(2, figsize=(10, 6))
plt.plot(gt_time, gt_position_x)
plt.plot(vslam_time, vslam_position_x)
plt.xlabel('Time (sec)')
plt.ylabel('X Position (m)')
plt.title('X Position vs. Time')
plt.legend(['Ground Truth', 'VSLAM'], loc='upper right')

# Plot Y Position vs. Time
plt.figure(3, figsize=(10, 6))
plt.plot(gt_time, gt_position_y)
plt.plot(vslam_time, vslam_position_y)
plt.xlabel('Time (sec)')
plt.ylabel('Y Position (m)')
plt.title('Y Position vs. Time')
plt.legend(['Ground Truth', 'VSLAM'], loc='upper right')

# Plot Z Position vs. Time
plt.figure(4, figsize=(10, 6))
plt.plot(gt_time, gt_position_z)
plt.plot(vslam_time, vslam_position_z)
plt.xlabel('Time (sec)')
plt.ylabel('Z Position (m)')
plt.title('Z Position vs. Time')
plt.legend(['Ground Truth', 'VSLAM'], loc='upper left')

# Plot X Position vs. Y Position
plt.figure(5, figsize=(10, 6))
plt.plot(gt_position_x, gt_position_y)
plt.plot(vslam_position_x, vslam_position_y)
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('X Position vs. Y Position')
plt.legend(['Ground Truth', 'VSLAM'], loc='upper right')

# Plot X Position vs. Z Position
plt.figure(6, figsize=(10, 6))
plt.plot(gt_position_x, gt_position_z)
plt.plot(vslam_position_x, vslam_position_z)
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.title('X Position vs. Z Position')
plt.legend(['Ground Truth', 'VSLAM'], loc='lower right')

# Plot Yaw Rotation vs. Time
plt.figure(7, figsize=(10, 6))
plt.plot(gt_time, gt_yaw_smoothed)
plt.plot(vslam_time, vslam_yaw)
plt.xlabel('Time (sec)')
plt.ylabel('Yaw (deg)')
plt.title('Yaw vs Time')
plt.legend(['Ground Truth', 'VSLAM'], loc='upper right')

"""Calculate Absolute Trajectory Error RMSE"""

# Create interpolation functions for each ground truth coordinate axis
interp_x = interp1d(gt_time, gt_position_x, kind='linear', fill_value="extrapolate")
interp_y = interp1d(gt_time, gt_position_y, kind='linear', fill_value="extrapolate")
interp_z = interp1d(gt_time, gt_position_z, kind='linear', fill_value="extrapolate")

# Interpolate ground truth to vslam's time steps
gt_interp_x = interp_x(vslam_time)
gt_interp_y = interp_y(vslam_time)
gt_interp_z = interp_z(vslam_time)

# Stack interpolated ground truth points into a single array
gt_positions = np.stack([gt_interp_x, gt_interp_y, gt_interp_z], axis=1)

# Convert vslam position Series' to Numpy arrays
vslam_positions = np.stack([vslam_position_x, vslam_position_y, vslam_position_z], axis=1)

# Calculate Absolute Trajectory Error (ATE) for each time step
ate = np.linalg.norm(gt_positions - vslam_positions, axis=1)

# Calculate RMSE of ATE
rmse_ate = np.sqrt(np.mean(ate**2))

print(f"RMSE of ATE: {rmse_ate}")

"""Calculate Yaw Error RMSE"""

# Create interpolation function
interp_yaw = interp1d(gt_time, gt_yaw_smoothed, kind='linear', fill_value="extrapolate")

# Interpolate ground truth to vslam's time steps
gt_yaw = interp_yaw(vslam_time)

# Calculate Absolute Trajectory Error (ATE) for each time step
yaw_error = gt_yaw - vslam_yaw

# Calculate RMSE of ATE
rmse_yaw_error = np.sqrt(np.mean(yaw_error**2))

print(f"RMSE of Yaw Error: {rmse_yaw_error}")

# Show all plots
plt.show()
