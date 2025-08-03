# Trajectory Checker

The `intersection_collision_checker` is a plugin module of `autoware_planning_validator` node. It is responsible for validating the planning trajectory before it is published.

## Supported features

The following features are supported for trajectory validation and can have thresholds set by parameters:

- **Invalid field** : e.g. Inf, Nan
- **Trajectory points interval** : invalid if any of the distance of trajectory points is too large
- **Curvature** : invalid if the trajectory has too sharp turns that is not feasible for the given vehicle kinematics
- **Relative angle** : invalid if the yaw angle changes too fast in the sequence of trajectory points
- **Lateral acceleration** : invalid if the expected lateral acceleration/deceleration is too large. Lateral acceleration is calculated using the formula:

  $$
  a_{lat} = v_{lon}^2 * \kappa
  $$

  Where $v_{lon}$ is longitudinal velocity and $\kappa$ is curvature. Since the acceleration embedded in path points is perpendicular to the derived lateral acceleration, projections are not considered. The velocity and acceleration assigned to each point are directed toward the next path point.

- **Lateral jerk** : invalid if the rate of change of lateral acceleration is too large. Lateral jerk is calculated using the formula:

  $$
  j_{lat} = v_{lon}^3 * \frac{d\kappa}{ds} + 3 * v_{lon}^2 * a_{lon} * \kappa
  $$

  Where $v_{lon}$ is longitudinal velocity, $\kappa$ is curvature, $a_{lon}$ is longitudinal acceleration, and $\frac{d\kappa}{ds}$ is the rate of curvature change with respect to distance. In this implementation, the curvature change ($\frac{d\kappa}{ds}$) is not considered, simplifying the calculation to only the second term. The lateral jerk represents how quickly the lateral acceleration changes, which affects ride comfort and vehicle stability.

- **Longitudinal acceleration/deceleration** : invalid if the acceleration/deceleration in the trajectory point is too large
- **Steering angle** : invalid if the expected steering value is too large estimated from trajectory curvature
- **Steering angle rate** : invalid if the expected steering rate value is too large
- **Velocity deviation** : invalid if the planning velocity is too far from the ego velocity
- **Distance deviation** : invalid if the ego is too far from the trajectory
- **Longitudinal distance deviation** : invalid if the trajectory is too far from ego in longitudinal direction
- **Forward trajectory length** : invalid if the trajectory length is not enough to stop within a given deceleration
- **Yaw deviation** : invalid if the difference between the ego yaw and closest trajectory yaw is too large.
  - in general planning is not responsible for keeping a low yaw deviation, so this metric is checked only when the closest trajectory yaw changes by more than `th_trajectory_yaw_shift` between successive trajectories.
- **Trajectory Shift** : invalid if the lat./long. distance between two consecutive trajectories near the Ego exceed the thresholds.

## Parameters

The following parameters are used to configure the different validation checks performed by `trajectory_checker`:

### Interval Check

| Name                           | Unit | Type   | Description                                                                                                              | Default value |
| :----------------------------- | :--- | :----- | :----------------------------------------------------------------------------------------------------------------------- | :------------ |
| `interval.enable`              | [-]  | bool   | flag to enable/disable interval validation check                                                                         | true          |
| `interval.threshold`           | [m]  | double | max valid distance between two consecutive trajectory points                                                             | 100.0         |
| `interval.handling_type`       | [-]  | int    | specify handling type for invalid interval (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `interval.override_error_diag` | [-]  | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)             | unspecified   |

### Curvature Check

| Name                            | Unit  | Type   | Description                                                                                                               | Default value |
| :------------------------------ | :---- | :----- | :------------------------------------------------------------------------------------------------------------------------ | :------------ |
| `curvature.enable`              | [-]   | bool   | flag to enable/disable curvature validation check                                                                         | true          |
| `curvature.threshold`           | [1/m] | double | max valid value for the trajectory curvature                                                                              | 2.0           |
| `curvature.handling_type`       | [-]   | int    | specify handling type for invalid curvature (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `curvature.override_error_diag` | [-]   | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)              | unspecified   |

### Relative Angle Check

| Name                                 | Unit  | Type   | Description                                                                                                                    | Default value |
| :----------------------------------- | :---- | :----- | :----------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `relative_angle.enable`              | [-]   | bool   | flag to enable/disable relative angle validation check                                                                         | true          |
| `relative_angle.threshold`           | [rad] | double | max valid angle difference between two consecutive trajectory points                                                           | 2.0           |
| `relative_angle.handling_type`       | [-]   | int    | specify handling type for invalid relative_angle (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `relative_angle.override_error_diag` | [-]   | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                   | unspecified   |

### Lateral Acceleration Check

| Name                               | Unit   | Type   | Description                                                                                                                  | Default value |
| :--------------------------------- | :----- | :----- | :--------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `acceleration.enable`              | [-]    | bool   | flag to enable/disable lateral acceleration validation check                                                                 | true          |
| `acceleration.threshold`           | [m/ss] | double | max valid value for the lateral acceleration along the trajectory                                                            | 9.8           |
| `acceleration.handling_type`       | [-]    | int    | specify handling type for invalid acceleration (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `acceleration.override_error_diag` | [-]    | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                 | unspecified   |

### Max Longitudinal Acceleration Check

| Name                               | Unit   | Type   | Description                                                                                                                  | Default value |
| :--------------------------------- | :----- | :----- | :--------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `acceleration.enable`              | [-]    | bool   | flag to enable/disable max longitudinal acceleration validation check                                                        | true          |
| `acceleration.threshold`           | [m/ss] | double | max valid value for the longitudinal acceleration along the trajectory                                                       | 9.8           |
| `acceleration.handling_type`       | [-]    | int    | specify handling type for invalid acceleration (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `acceleration.override_error_diag` | [-]    | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                 | unspecified   |

### Min Longitudinal Acceleration Check

| Name                               | Unit   | Type   | Description                                                                                                                  | Default value |
| :--------------------------------- | :----- | :----- | :--------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `acceleration.enable`              | [-]    | bool   | flag to enable/disable min longitudinal acceleration validation check                                                        | true          |
| `acceleration.threshold`           | [m/ss] | double | min valid value for the longitudinal acceleration along the trajectory                                                       | -9.8          |
| `acceleration.handling_type`       | [-]    | int    | specify handling type for invalid acceleration (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `acceleration.override_error_diag` | [-]    | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                 | unspecified   |

### Lateral Jerk Check

| Name                               | Unit    | Type   | Description                                                                                                                  | Default value |
| :--------------------------------- | :------ | :----- | :--------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `lateral_jerk.enable`              | [-]     | bool   | flag to enable/disable lateral jerk validation check                                                                         | true          |
| `lateral_jerk.threshold`           | [m/sss] | double | max valid value for the lateral jerk along the trajectory                                                                    | 7.0           |
| `lateral_jerk.handling_type`       | [-]     | int    | specify handling type for invalid lateral jerk (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `lateral_jerk.override_error_diag` | [-]     | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                 | unspecified   |

### Steering Check

| Name                           | Unit  | Type   | Description                                                                                                              | Default value |
| :----------------------------- | :---- | :----- | :----------------------------------------------------------------------------------------------------------------------- | :------------ |
| `steering.enable`              | [-]   | bool   | flag to enable/disable steering validation check                                                                         | true          |
| `steering.threshold`           | [rad] | double | max valid steering value along the trajectory                                                                            | 1.414         |
| `steering.handling_type`       | [-]   | int    | specify handling type for invalid steering (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `steering.override_error_diag` | [-]   | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)             | unspecified   |

### Steering Rate Check

| Name                                | Unit    | Type   | Description                                                                                                                   | Default value |
| :---------------------------------- | :------ | :----- | :---------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `steering_rate.enable`              | [-]     | bool   | flag to enable/disable steering rate validation check                                                                         | true          |
| `steering_rate.threshold`           | [rad/s] | double | max valid steering rate along the trajectory                                                                                  | 10.0          |
| `steering_rate.handling_type`       | [-]     | int    | specify handling type for invalid steering rate (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `steering_rate.override_error_diag` | [-]     | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                  | unspecified   |

### Distance Deviation Check

| Name                                     | Unit | Type   | Description                                                                                                                    | Default value |
| :--------------------------------------- | :--- | :----- | :----------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `distance_deviation.enable`              | [-]  | bool   | flag to enable/disable distance deviation validation check                                                                     | true          |
| `distance_deviation.threshold`           | [m]  | double | max valid lateral distance between ego and the nearest trajectory segment                                                      | 100.0         |
| `distance_deviation.handling_type`       | [-]  | int    | specify handling type for invalid dist deviation (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `distance_deviation.override_error_diag` | [-]  | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                   | unspecified   |

### Longitudinal Distance Deviation Check

| Name                                         | Unit | Type   | Description                                                                                                                         | Default value |
| :------------------------------------------- | :--- | :----- | :---------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `lon_distance_deviation.enable`              | [-]  | bool   | flag to enable/disable longitudinal distance deviation validation check                                                             | true          |
| `lon_distance_deviation.threshold`           | [m]  | double | max valid longitudinal distance between ego and nearest trajectory point                                                            | 2.0           |
| `lon_distance_deviation.handling_type`       | [-]  | int    | specify handling type for invalid lon. dist deviation (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `lon_distance_deviation.override_error_diag` | [-]  | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                        | unspecified   |

### Velocity Deviation Check

| Name                                     | Unit  | Type   | Description                                                                                                                        | Default value |
| :--------------------------------------- | :---- | :----- | :--------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `velocity_deviation.enable`              | [-]   | bool   | flag to enable/disable velocity deviation validation check                                                                         | true          |
| `velocity_deviation.threshold`           | [m/s] | double | max valid velocity deviation between ego and nearest trajectory point                                                              | 100.0         |
| `velocity_deviation.handling_type`       | [-]   | int    | specify handling type for invalid velocity deviation (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `velocity_deviation.override_error_diag` | [-]   | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                       | unspecified   |

### Yaw Deviation Check

| Name                                    | Unit  | Type   | Description                                                                                                                   | Default value |
| :-------------------------------------- | :---- | :----- | :---------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `yaw_deviation.enable`                  | [-]   | bool   | flag to enable/disable yaw deviation validation check                                                                         | true          |
| `yaw_deviation.threshold`               | [rad] | double | max valid yaw deviation between ego and nearest trajectory point                                                              | 1.5708        |
| `yaw_deviation.th_trajectory_yaw_shift` | [rad] | double | minimum change in nearest yaw value between the previous and current trajectory to trigger check                              | 0.1           |
| `yaw_deviation.handling_type`           | [-]   | int    | specify handling type for invalid yaw deviation (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `yaw_deviation.override_error_diag`     | [-]   | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                  | unspecified   |

### Forward Trajectory Length Check

| Name                                            | Unit   | Type   | Description                                                                                                                    | Default value |
| :---------------------------------------------- | :----- | :----- | :----------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `forward_trajectory_length.enable`              | [-]    | bool   | flag to enable/disable forward length validation check                                                                         | true          |
| `forward_trajectory_length.acceleration`        | [m/ss] | double | acceleration value used to calculate required trajectory length.                                                               | -5.0          |
| `forward_trajectory_length.margin`              | [m]    | double | margin of the required length not to raise an error when ego slightly exceeds the end point.                                   | 2.0           |
| `forward_trajectory_length.handling_type`       | [-]    | int    | specify handling type for invalid forward length (optional parameter, if not specified will use default of planning validator) | unspecified   |
| `forward_trajectory_length.override_error_diag` | [-]    | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                   | unspecified   |

### Trajectory Shift Check

| Name                                   | Unit | Type   | Description                                                                                                                      | Default value |
| :------------------------------------- | :--- | :----- | :------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `trajectory_shift.enable`              | [-]  | bool   | flag to enable/disable trajectory shift validation check                                                                         | true          |
| `trajectory_shift.lat_shift_th`        | [m]  | double | max valid lateral distance between two consecutive trajectories (measured at nearest points to ego)                              | 0.5           |
| `trajectory_shift.forward_shift_th`    | [m]  | double | max valid Longitudinal distance between two consecutive trajectories (measured at nearest point to ego)                          | 1.0           |
| `trajectory_shift.backward_shift_th`   | [m]  | double | min valid longitudinal distance between two consecutive trajectories (measured at nearest point to ego)                          | 0.1           |
| `trajectory_shift.handling_type`       | [-]  | int    | specify handling type for invalid trajectory shift (optional parameter, if not specified will use default of planning validator) | 2             |
| `trajectory_shift.override_error_diag` | [-]  | bool   | if true, will override error diag from other checks (optional parameter, if not specified will assume FALSE)                     | true          |
