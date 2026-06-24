# autoware_speed_scale_corrector

## Overview

This package estimates the vehicle speed scale factor by comparing map-matched pose velocity with the longitudinal velocity reported by the vehicle. The reference velocity is computed from localization pose (e.g. NDT scan matcher), so the estimate reflects consistency between localization motion and wheel speed.

The estimated scale factor can be used to monitor and tune the speed scale in `autoware_vehicle_velocity_converter`. The node runs periodically at `update_interval` and updates the estimate only when all operational constraints are satisfied.

## Block Diagram

### Node architecture

ROS node interfaces and publish timing.

![node architecture](docs/block_diagram_node.drawio.svg)

### Estimation algorithm

Internal processing flow inside `SpeedScaleEstimator`.

![estimation algorithm](docs/block_diagram_algorithm.drawio.svg)

## Algorithm

The speed scale estimation follows these steps:

### 1. Data Collection

- Collects pose (`PoseStamped`), IMU, and vehicle velocity reports (`VelocityReport`) received since the previous timer callback
- Keeps the previous pose internally to compute odometry velocity between consecutive updates

### 2. Odometry Velocity Calculation

Computes odometry velocity from the previous and current poses:

$$
v_{odom} = \frac{\| \Delta \mathbf{p} \|}{\Delta t}
$$

If the time difference between the two poses is too large, the previous pose is reset and estimation is skipped.

### 3. Timestamp Synchronization

For the current pose timestamp, the node selects:

- the nearest IMU sample for angular velocity constraint checking
- the nearest velocity report for Kalman filter observation

Estimation is skipped when either timestamp difference exceeds `update_interval`.

### 4. Constraint Validation

Estimation is performed only when all of the following constraints are satisfied:

- **Angular velocity constraint (IMU)**:

  $$
  |\omega_{imu}| \leq \omega_{max}
  $$

  Default $\omega_{max} \approx 0.6\ \mathrm{deg/s}$ ($0.0105\ \mathrm{rad/s}$) rejects obvious curves and turns. This threshold is set near the practical IMU bias/noise floor rather than fine straight-line discrimination.

- **Speed constraints (odometry)**:

  $$
  v_{min} \leq v_{odom} \leq v_{max}
  $$

  Default $v_{min} = 6.0\ \mathrm{m/s}$ improves pose-differentiation SNR. Together with $\omega_{max}$, estimation runs mainly on moderate-speed, near-straight segments.

- **Velocity report validity**:

  $$
  |v_{report}| > 0
  $$

### 5. Scale Factor Estimation (Kalman Filter)

The scale factor is modeled as a scalar state $x$ with the observation:

$$
z = v_{odom} = x \cdot v_{report}
$$

Kalman filter update:

- Prediction:

  $$
  x_{pred} = x,\quad P_{pred} = P + Q
  $$

- Update:

  $$
  H = v_{report},\quad y = z - H x_{pred}
  $$

  $$
  S = H^2 P_{pred} + R,\quad K = \frac{P_{pred} H}{S}
  $$

  $$
  x = x_{pred} + K y,\quad P = (1 - K H) P_{pred}
  $$

## Inputs / Outputs

### Input Topics

| Name                      | Type                                         | Description                 |
| ------------------------- | -------------------------------------------- | --------------------------- |
| `~/input/pose`            | `geometry_msgs::msg::PoseStamped`            | Localization pose (e.g. NDT scan matcher) |
| `~/input/velocity_report` | `autoware_vehicle_msgs::msg::VelocityReport` | Vehicle velocity report     |
| `~/input/imu`             | `sensor_msgs::msg::Imu`                      | IMU sensor data             |

### Output Topics

| Name                    | Type                                                | Description                                     |
| ----------------------- | --------------------------------------------------- | ----------------------------------------------- |
| `~/output/scale_factor` | `autoware_internal_debug_msgs::msg::Float32Stamped` | Estimated speed scale factor (updated on success) |
| `~/output/debug_info`   | `autoware_internal_debug_msgs::msg::StringStamped`  | Debug information                               |

## Parameters

{{ json_to_markdown("localization/autoware_speed_scale_corrector/schema/speed_scale_corrector.schema.json") }}

### Tuning

Default constraint values are a practical starting point, not vehicle-specific optima. If estimation accuracy is insufficient, tune `max_angular_velocity`, `min_speed`, and Kalman noise parameters per vehicle and operation. IMU bias cancellation or other preprocessing may allow a stricter angular velocity threshold; without it, thresholds near the IMU noise floor are expected.
