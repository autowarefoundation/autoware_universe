# steer_offset_estimator

## Purpose

The role of this node is to automatically calibrate `steer_offset` used in the `vehicle_interface` node.
k

## Inner-workings / Algorithms

This module estimates the steering offset using a **Recursive Least Squares (RLS)** algorithm based on vehicle kinematic model constraints.

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

### Recursive Least Squares Algorithm

The RLS algorithm updates the offset estimate and covariance recursively:

1. **Prediction coefficient calculation:**

   $$
   \phi = \frac{v}{L}
   $$

2. **Covariance update with forgetting factor:**

   $$
   P_k = \frac{P_{k-1} - \frac{P_{k-1} \times \phi^2 \times P_{k-1}}{\lambda + \phi \times P_{k-1} \times \phi}}{\lambda}
   $$

3. **Kalman gain calculation:**

   $$
   K = \frac{P_k \times \phi}{\lambda + \phi \times P_k \times \phi}
   $$

4. **Error calculation:**

   $$
   e_{observed} = \omega_{observed} - \phi \times \delta_{measured}
   $$

   $$
   e_{estimated} = \phi \times \delta_{offset,prev}
   $$

5. **Offset estimate update:**

   $$
   \delta_{offset,new} = \delta_{offset,prev} + K \times (e_{observed} - e_{estimated})
   $$

Where:

- $P$: Estimation covariance matrix (scalar in this 1D case)
- $\lambda$: Forgetting factor (typically 0.999) for adaptation to changing conditions
- $K$: Kalman gain
- $k$: Current time step

### Algorithm Constraints

The algorithm only updates when:

- Vehicle velocity > `min_velocity` (ensures reliable kinematic model)
- $|\delta_{measured}|$ < `max_steer` (avoids nonlinear tire behavior)
- Both pose and steering data are available

This approach provides continuous, real-time calibration of steering offset during normal driving operations.

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
