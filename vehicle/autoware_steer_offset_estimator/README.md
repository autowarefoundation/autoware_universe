# steer_offset_estimator

## Purpose

The role of this node is to automatically calibrate `steer_offset` used in the `vehicle_interface` node.
k

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

- Vehicle velocity > `min_velocity` (ensures reliable kinematic model)
- $|\delta_{measured}|$ < `max_steer` (avoids nonlinear tire behavior)
- Both pose and steering data are available

This Kalman Filter approach provides continuous, real-time calibration of steering offset during normal driving operations, with process noise allowing adaptation to changing conditions and measurement noise handling sensor uncertainties.

## Inputs / Outputs

### Input

| Name            | Type                                         | Description  |
| --------------- | -------------------------------------------- | ------------ |
| `~/input/pose`  | `geometry_msgs::msg::PoseStamped`            | vehicle pose |
| `~/input/steer` | `autoware_vehicle_msgs::msg::SteeringReport` | steering     |

### Output

| Name                                  | Type                                                | Description                   |
| ------------------------------------- | --------------------------------------------------- | ----------------------------- |
| `~/output/steering_offset`            | `autoware_internal_debug_msgs::msg::Float32Stamped` | steering offset               |
| `~/output/steering_offset_covariance` | `autoware_internal_debug_msgs::msg::Float32Stamped` | covariance of steering offset |
| `~/output/debug_info`                 | `autoware_internal_debug_msgs::msg::StringStamped`  | debug info                    |

## Parameters

{{ json_to_markdown("vehicle/autoware_steer_offset_estimator/schema/steer_offset_estimator.schema.json") }}
