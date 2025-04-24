# Planning Validator

The `autoware_planning_validator` is a module that checks the validity of a trajectory before it is published. The status of the validation can be viewed in the `/diagnostics` and `/validation_status` topics. When an invalid trajectory is detected, the `autoware_planning_validator` will process the trajectory following the selected option: "0. publish the trajectory as it is", "1. stop publishing the trajectory", "2. publish the last validated trajectory".

![autoware_planning_validator](./image/planning_validator.drawio.svg)

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
- **Yaw difference** : invalid if the difference between the ego yaw and closest trajectory yaw is too large
- **Trajectory Shift** : invalid if the lat./long. distance between two consecutive trajectories near the Ego exceed the thresholds.

The following features are to be implemented.

- **(TODO) TTC calculation** : invalid if the expected time-to-collision is too short on the trajectory

## Inputs/Outputs

### Inputs

The `autoware_planning_validator` takes in the following inputs:

| Name                   | Type                                     | Description                                    |
| ---------------------- | ---------------------------------------- | ---------------------------------------------- |
| `~/input/kinematics`   | nav_msgs/Odometry                        | ego pose and twist                             |
| `~/input/acceleration` | geometry_msgs/AccelWithCovarianceStamped | current acceleration of the ego vehicle        |
| `~/input/trajectory`   | autoware_planning_msgs/Trajectory        | target trajectory to be validated in this node |

### Outputs

It outputs the following:

| Name                         | Type                                       | Description                                                               |
| ---------------------------- | ------------------------------------------ | ------------------------------------------------------------------------- |
| `~/output/trajectory`        | autoware_planning_msgs/Trajectory          | validated trajectory                                                      |
| `~/output/validation_status` | planning_validator/PlanningValidatorStatus | validator status to inform the reason why the trajectory is valid/invalid |
| `/diagnostics`               | diagnostic_msgs/DiagnosticStatus           | diagnostics to report errors                                              |

## Parameters

The following parameters can be set for the `autoware_planning_validator`:

### System parameters

| Name                            | Type   | Description                                                                                                                                                                                      | Default value |
| :------------------------------ | :----- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `handling_type.noncritical`     | int    | set the handling type for noncritical invalid trajectory. <br>0: publish invalid trajectory as it is, <br>1: stop publishing the trajectory, <br>2: publish last valid trajectory.               | 0             |
| `handling_type.critical`        | int    | set the handling type for critical invalid trajectory. <br>0: publish the trajectory even if it is invalid, <br>1: stop publishing the trajectory, <br>2: publish the last validated trajectory. | 0             |
| `publish_diag`                  | bool   | if true, diagnostics msg is published.                                                                                                                                                           | true          |
| `diag_error_count_threshold`    | int    | Number of consecutive invalid trajectories to set the Diag to ERROR. (Fe.g, threshold = 1 means, even if the trajectory is invalid, the Diag will not be ERROR if the next trajectory is valid.) | 0             |
| `display_on_terminal`           | bool   | show error msg on terminal                                                                                                                                                                       | true          |
| `enable_soft_stop_on_prev_traj` | bool   | if true, and handling type is `2: publish last valid trajectory`, the soft stop is applied instead of using mrm emergency stop.                                                                  | true          |
| `soft_stop_deceleration`        | double | deceleration value to be used for soft stop action.                                                                                                                                              | -1.0          |
| `soft_stop_jerk_lim`            | double | jerk limit value to be used for soft stop action.                                                                                                                                                | 0.3           |

### Algorithm parameters

#### Latency Check

| Name                                  | Type   | Description                                                   | Default value |
| :------------------------------------ | :----- | :------------------------------------------------------------ | :------------ |
| `validity_checks.latency.enable`      | bool   | flag to enable/disable latency validation check               | true          |
| `validity_checks.latency.threshold`   | double | max valid value for the age of the trajectory msg [s]         | 1.0           |
| `validity_checks.latency.is_critical` | bool   | if true, will use handling type specified for critical checks | false         |

#### Interval Check

| Name                                   | Type   | Description                                                      | Default value |
| :------------------------------------- | :----- | :--------------------------------------------------------------- | :------------ |
| `validity_checks.interval.enable`      | bool   | flag to enable/disable interval validation check                 | true          |
| `validity_checks.interval.threshold`   | double | max valid distance between two consecutive trajectory points [m] | 100.0         |
| `validity_checks.interval.is_critical` | bool   | if true, will use handling type specified for critical checks    | false         |

#### Curvature Check

| Name                                    | Type   | Description                                                   | Default value |
| :-------------------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| `validity_checks.curvature.enable`      | bool   | flag to enable/disable curvature validation check             | true          |
| `validity_checks.curvature.threshold`   | double | max valid value for the trajectory curvature [1/m]            | 2.0           |
| `validity_checks.curvature.is_critical` | bool   | if true, will use handling type specified for critical checks | false         |

#### Relative Angle Check

| Name                                         | Type   | Description                                                                | Default value |
| :------------------------------------------- | :----- | :------------------------------------------------------------------------- | :------------ |
| `validity_checks.relative_angle.enable`      | bool   | flag to enable/disable relative angle validation check                     | true          |
| `validity_checks.relative_angle.threshold`   | double | max valid angle difference between two consecutive trajectory points [rad] | 2.0           |
| `validity_checks.relative_angle.is_critical` | bool   | if true, will use handling type specified for critical checks              | false         |

#### Acceleration Check

| Name                                               | Type   | Description                                                                   | Default value |
| :------------------------------------------------- | :----- | :---------------------------------------------------------------------------- | :------------ |
| `validity_checks.acceleration.enable`              | bool   | flag to enable/disable acceleration validation check                          | true          |
| `validity_checks.acceleration.lateral_th`          | double | max valid value for the lateral acceleration along the trajectory [m/ss]      | 9.8           |
| `validity_checks.acceleration.longitudinal_max_th` | double | max valid value for the longitudinal acceleration along the trajectory [m/ss] | 9.8           |
| `validity_checks.acceleration.longitudinal_min_th` | double | min valid value for the longitudinal acceleration along the trajectory [m/ss] | -9.8          |
| `validity_checks.acceleration.is_critical`         | bool   | if true, will use handling type specified for critical checks                 | false         |

### Lateral Jerk Check

| Name                                       | Type   | Description                                                       | Default value |
| :----------------------------------------- | :----- | :---------------------------------------------------------------- | :------------ |
| `validity_checks.lateral_jerk.enable`      | bool   | flag to enable/disable lateral jerk validation check              | true          |
| `validity_checks.lateral_jerk.threshold`   | double | max valid value for the lateral jerk along the trajectory [m/sss] | 7.0           |
| `validity_checks.lateral_jerk.is_critical` | bool   | if true, will use handling type specified for critical checks     | false         |

#### Steering Check

| Name                                   | Type   | Description                                                   | Default value |
| :------------------------------------- | :----- | :------------------------------------------------------------ | :------------ |
| `validity_checks.steering.enable`      | bool   | flag to enable/disable steering validation check              | true          |
| `validity_checks.steering.threshold`   | double | max valid steering value along the trajectory [rad]           | 1.414         |
| `validity_checks.steering.is_critical` | bool   | if true, will use handling type specified for critical checks | false         |

#### Steering Rate Check

| Name                                        | Type   | Description                                                   | Default value |
| :------------------------------------------ | :----- | :------------------------------------------------------------ | :------------ |
| `validity_checks.steering_rate.enable`      | bool   | flag to enable/disable steering rate validation check         | true          |
| `validity_checks.steering_rate.threshold`   | double | max valid steering rate along the trajectory [rad/s]          | 10.0          |
| `validity_checks.steering_rate.is_critical` | bool   | if true, will use handling type specified for critical checks | false         |

#### Deviation Check

| Name                                        | Type   | Description                                                                  | Default value |
| :------------------------------------------ | :----- | :--------------------------------------------------------------------------- | :------------ |
| `validity_checks.deviation.enable`          | bool   | flag to enable/disable deviation validation check                            | true          |
| `validity_checks.deviation.velocity_th`     | double | max valid velocity deviation between ego and nearest trajectory point [m/s]  | 100.0         |
| `validity_checks.deviation.distance_th`     | double | max valid euclidean distance between ego and nearest trajectory point [m]    | 100.0         |
| `validity_checks.deviation.lon_distance_th` | double | max valid longitudinal distance between ego and nearest trajectory point [m] | 2.0           |
| `validity_checks.deviation.yaw_th`          | double | max valid yaw deviation between ego and nearest trajectory point [rad]       | 1.5708        |
| `validity_checks.deviation.is_critical`     | bool   | if true, will use handling type specified for critical checks                | false         |

#### Forward Trajectory Length Check

| Name                                                     | Type   | Description                                                                                      | Default value |
| :------------------------------------------------------- | :----- | :----------------------------------------------------------------------------------------------- | :------------ |
| `validity_checks.forward_trajectory_length.enable`       | bool   | flag to enable/disable latency validation check                                                  | true          |
| `validity_checks.forward_trajectory_length.acceleration` | double | acceleration value used to calculate required trajectory length. [m/ss]                          | -5.0          |
| `validity_checks.forward_trajectory_length.margin`       | double | margin of the required length not to raise an error when ego slightly exceeds the end point. [m] | 2.0           |
| `validity_checks.forward_trajectory_length.is_critical`  | bool   | if true, will use handling type specified for critical checks                                    | false         |

#### Trajectory Shift Check

| Name                                                 | Type   | Description                                                                                                 | Default value |
| :--------------------------------------------------- | :----- | :---------------------------------------------------------------------------------------------------------- | :------------ |
| `validity_checks.trajectory_shift.enable`            | bool   | flag to enable/disable trajectory shift validation check                                                    | true          |
| `validity_checks.trajectory_shift.lat_shift_th`      | double | max valid lateral distance between two consecutive trajectories (measured at nearest points to ego) [m]     | 0.5           |
| `validity_checks.trajectory_shift.forward_shift_th`  | double | max valid Longitudinal distance between two consecutive trajectories (measured at nearest point to ego) [m] | 1.0           |
| `validity_checks.trajectory_shift.backward_shift_th` | double | min valid longitudinal distance between two consecutive trajectories (measured at nearest point to ego) [m] | 0.1           |
| `validity_checks.trajectory_shift.is_critical`       | bool   | if true, will use handling type specified for critical checks                                               | true          |
