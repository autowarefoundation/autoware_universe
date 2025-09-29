# autoware_speed_scale_corrector

## Overview

This package performs vehicle speed scale correction by estimating and correcting speed scale factors. It compares velocities calculated from odometry with velocities reported by the vehicle's velocity sensors to correct discrepancies caused by sensor individual differences and calibration errors.

## Algorithm

The speed scale estimation follows these detailed steps:

### 1. Data Collection and Buffering

- Collects pose information (PoseWithCovarianceStamped), IMU data, and vehicle velocity reports (VelocityReport)
- Maintains internal buffers for each data stream with timestamps
- Extracts time series data: position (x, y) from poses, angular velocity (z) from IMU, and longitudinal velocity from velocity reports

### 2. Time Synchronization

- Finds the common time interval across all sensor data streams using interval intersection
- Ensures sufficient time window (≥ `time_window` parameter) is available for reliable estimation
- Early returns with previous scale factor if data is insufficient or time windows don't overlap

### 3. Data Preprocessing

- Applies Gaussian smoothing (σ=0.7) to all sensor data to reduce noise:
  - Position data (x, y coordinates)
  - IMU angular velocity
  - Vehicle velocity reports
- Uses a sliding window Gaussian kernel with 3σ cutoff for efficient computation

### 4. Trajectory Interpolation

- Creates cubic spline interpolators for smoothed position data (x, y)
- Creates linear interpolators for angular velocity and velocity data
- Generates uniform time samples within the valid time window at specified intervals

### 5. State Vector Creation

For each time sample, computes:

- **Odometry velocity**: Magnitude of position derivatives

  $$
  v_{odom} = \sqrt{\left(\frac{dx}{dt}\right)^2 + \left(\frac{dy}{dt}\right)^2}
  $$

- **Distance between points**: Euclidean distance from previous position
- **Integrated distance from velocity**: Trapezoidal integration of velocity reports
- **Angular velocity**: Interpolated IMU z-axis rotation

### 6. Constraint Validation

Validates that all states satisfy operational constraints to ensure reliable estimation:

- **Angular velocity constraint**:

  $$
  |\omega| \leq \omega_{max}
  $$

  (avoids estimation during sharp turns)

- **Speed constraints**:

  $$
  v_{min} \leq v \leq v_{max}
  $$

  (ensures sufficient motion for accurate differentiation and avoids extreme speeds)

- **Speed change constraint**:

  $$
  |\Delta v| \leq \Delta v_{max}
  $$

  (filters out periods of rapid acceleration/deceleration where tire slip may occur)

### 7. Scale Factor Estimation

- Computes instantaneous scale factor:

  $$
  s = \frac{d_{odom}}{d_{velocity}}
  $$

- Updates running average:

  $$
  \bar{s}_{new} = \frac{\bar{s}_{old} \times n + s_{current}}{n + 1}
  $$

- Clears sensor buffers after successful estimation

## Inputs / Outputs

### Input Topics

| Name                           | Type                                            | Description                 |
| ------------------------------ | ----------------------------------------------- | --------------------------- |
| `~/input/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | Pose information (odometry) |
| `~/input/velocity_report`      | `autoware_vehicle_msgs::msg::VelocityReport`    | Vehicle velocity report     |
| `~/input/imu`                  | `sensor_msgs::msg::Imu`                         | IMU sensor data             |

### Output Topics

| Name                    | Type                                                | Description                  |
| ----------------------- | --------------------------------------------------- | ---------------------------- |
| `~/output/scale_factor` | `autoware_internal_debug_msgs::msg::Float32Stamped` | Estimated speed scale factor |
| `~/output/debug_info`   | `autoware_internal_debug_msgs::msg::StringStamped`  | Debug information            |

## Parameters

{{ json_to_markdown("sensing/autoware_speed_scale_corrector/schema/speed_scale_corrector.schema.json") }}
