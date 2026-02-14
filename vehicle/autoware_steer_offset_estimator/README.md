# Steer Offset Estimator

## Purpose

The role of this node is to automatically estimate the `steer_offset` used in the lateral controller and vehicle interface, and enable auto or manual calibration in real time.

## Inner-workings / Algorithms

This module estimates the steering offset using a **Kalman Filter** algorithm based on vehicle kinematic model constraints.

### Kinematic Model

![kinematics](./image/kinematics.png)

The vehicle kinematic model relates steering angle to angular velocity:

$$
\omega = \frac{v}{L} \times \tan(\delta) \approx \frac{v}{L} \times \delta
$$

Where:

- $\omega$: Angular velocity (yaw rate) [rad/s]
- $v$: Vehicle velocity [m/s]
- $L$: Wheelbase [m]
- $\delta$: Steering angle [rad]

### Problem Formulation

Due to mechanical tolerances and sensor calibration errors, there exists a steering offset $\delta_{offset}$. The true relationship becomes:

$$
\omega_{observed} = \frac{v}{L} \times (\delta_{measured} + \delta_{offset}) + noise
$$

The algorithm estimates $\delta_{offset}$ by minimizing the error between observed and predicted angular velocity.

### Kalman Filter Algorithm

The Kalman Filter algorithm updates the offset estimate and covariance recursively with time and measurement updates:

- **Regressor and measurement formulation:**

  $$
  \phi = \frac{v}{L}
  $$

  $$
  y = \omega_{observed} - \phi \times \delta_{measured}
  $$

- **Time update (process model):**

  $$
  P_{prior} = P_{k-1} + Q
  $$

- **Measurement update denominator:**

  $$
  denom = R + \phi^2 \times P_{prior}
  $$

- **Kalman gain calculation:**

  $$
  K = \frac{P_{prior} \times \phi}{denom}
  $$

- **Innovation (residual) and state update:**

  $$
  residual = y - \phi \times \delta_{offset,prev}
  $$

  $$
  \delta_{offset,new} = \delta_{offset,prev} + K \times residual
  $$

- **Covariance update:**

  $$
  P_k = P_{prior} - \frac{P_{prior} \times \phi^2 \times P_{prior}}{denom}
  $$

Where:

- $P$: Estimation covariance matrix (scalar in this 1D case)
- $Q$: Process noise covariance (allows parameter drift)
- $R$: Measurement noise covariance
- $K$: Kalman gain
- $k$: Current time step

### Algorithm Constraints

The algorithm only updates when:

- Both pose and steering data are available
- Vehicle velocity > `min_velocity` (ensures reliable kinematic model)
- $|\delta_{\text{measured}}|$ < `max_steer` (avoids nonlinear tire behavior)
- $|\dot{\delta}_{\text{measured}}|$ < `max_steer_rate` (avoids "zero-crossing" errors during transient maneuvers)
- angular velocity < `max_ang_velocity` (avoids tire slip and dynamic noise)

This Kalman Filter approach provides continuous, real-time calibration of steering offset during normal driving operations, with process noise allowing adaptation to changing conditions and measurement noise handling sensor uncertainties.

## Calibration

The node manages how the estimated offset is applied to the vehicle system through three distinct modes and a set of safety validation gates.

### Calibration Modes

- **OFF**: The estimator runs in the background for monitoring and debug purposes. No parameters are updated and no calibration topics are published.
- **MANUAL**: Calibration is triggered only via a dedicated ROS 2 service call (`~/trigger_steer_offset_calibration`).
- **AUTO**: The system automatically triggers a calibration update when the internal stability and confidence conditions are met.

### Calibration Execution Logic

A calibration is executed when a service call is received, or in case of **AUTO** mode update conditions are satisfied.
When calibration is triggered the node will update the persistent YAML file (if parameter is enabled) and publish the offset update value on the topic `~/steering_offset_update`.

Calibration is executed only if specific validation checks are satisfied.

#### 1. Manual Mode Checks

When a service call is received, the following checks are performed:

- **Confidence**: The estimation covariance must be below `covariance_threshold`.
- **Safety**: The total offset must not exceed `max_offset_limit`.

#### 2. Auto Mode Checks

In addition to the Manual gates, Auto mode applies more restrictive temporal and significance filters:

- **Steady State**: Valid estimation results must be continuous for at least `min_steady_duration`.
- **Update Interval**: Time since the last update must exceed `min_update_interval`.
- **Significance**: The difference between the current estimate and the registered offset must exceed `error_threshold`.

## Interfaces

### Inputs / Outputs

#### Input

| Name            | Type                                         | Description  |
| --------------- | -------------------------------------------- | ------------ |
| `~/input/pose`  | `geometry_msgs::msg::PoseStamped`            | vehicle pose |
| `~/input/steer` | `autoware_vehicle_msgs::msg::SteeringReport` | steering     |

#### Output

| Name                                  | Type                                                | Description                                   |
| ------------------------------------- | --------------------------------------------------- | --------------------------------------------- |
| `~/output/steering_offset`            | `autoware_internal_debug_msgs::msg::Float32Stamped` | steering offset                               |
| `~/output/steering_offset_covariance` | `autoware_internal_debug_msgs::msg::Float32Stamped` | covariance of steering offset                 |
| `~/output/steering_offset_error`      | `autoware_internal_debug_msgs::msg::Float32Stamped` | difference between estimate and current value |
| `~/output/steering_offset_update`     | `autoware_internal_debug_msgs::msg::Float32Stamped` | updated steering offset value                 |
| `~/output/debug_info`                 | `autoware_internal_debug_msgs::msg::StringStamped`  | debug info                                    |

### Services

| Name                                 | Type                     | Description                         |
| ------------------------------------ | ------------------------ | ----------------------------------- |
| `~/trigger_steer_offset_calibration` | `std_srvs::srv::Trigger` | trigger steering offset calibration |

## Parameters

{{ json_to_markdown("vehicle/autoware_steer_offset_estimator/schema/steer_offset_estimator.schema.json") }}
