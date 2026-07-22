# Traffic Light Stop

## Purpose/Role

This module modifies DP trajectories if they are found to run through a red or amber traffic light. If it finds that the DP trajectories causes the ego to cross or overshoot a red/amber traffic light, it will insert a stop point to ensure traffic rules compliance.

This module uses the TrafficLightComplianceChecker to check for violations. For details of the compliance checker logic refer to
[Compliance Checker Documentation](../../../common/autoware_traffic_light_compliance_checker/README.md).

The TrafficLightComplianceChecker will check for violations and return the results. The traffic light stop module will then scan the provided results for any red/amber light crossing. If the trajectory is crossing a red/amber light then a stop point is inserted.

## Algorithm Overview

The following diagram shows the overall logic flow of the TrafficLightStop module:

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE
start
group Is Modification Required {
if (Is Feature Disabled OR Invalid Input OR \nCompliance Checker Not Initialized?) then (yes)
:Return False;<<#LightYellow>>
stop
else (no)
endif
:Constuct Compliance Checker Input Struct;<<#LightBlue>>
:Run Compliance Checker;<<#LightBlue>>
if (Is Results Unavailable OR \nNo Violation is Found?) then (yes)
:Return False;<<#LightYellow>>
stop
else (no)
endif
:Set Nearest Violation & Debug Data;<<#LightBlue>>
}
group Modify Trajectory {
:Calculate Trajectory Length;<<#LightBlue>>
:Calculate Stop Margin;<<#LightBlue>>
:Calculate Target Stop Point Arc length\n(Clamped by Min. Stopping Distance & Traj Length);<<#LightBlue>>
if (Is Stop Point Exists already?) then (yes)
:Return False;<<#LightYellow>>
stop
else (no)
endif
if (Is Arrived at Target Stop Point?) then (yes)
:Set Stop Trajectory;
else (no)
:Insert Stop Point;
endif
:Set Planning Factor;<<#LightBlue>>
}
:Return True;<<#LightGreen>>
stop
@enduml
```

## Interface

### Context & Data

The module utilizes the following data from `TrajectoryModifierContext` & `InputData`:

- **Vehicle Info**: Used to account for the vehicle's dimensions (longitudinal offset) when checking for stop line intersections & computing stopping margin.
- **Lanelet Map**: Used to find regulatory elements (traffic lights) and their associated stop lines.
- **Traffic Light Signals**: Provides the current state (color) of traffic light groups.
- **Route**: Used to map traffic light signals to lanelets and filter those that are relevant to the vehicle's path.
- **Odometry**: Provides the current ego velocity to determine if ego is stopped and to calculate stopping distances.
- **Acceleration**: Provides the current ego acceleration to calculate stopping distances.

### Parameters

The module parameters are grouped under `traffic_light_stop` section of `trajectory_modifier.param.yaml`

| Parameter name                  | Type   | Default | Description                                                                                                       |
| ------------------------------- | ------ | ------- | ----------------------------------------------------------------------------------------------------------------- |
| `stop_margin`                   | double | 0.75    | [m] Distance to keep from stop line (measured from ego front)                                                     |
| `stop_for_red_light`            | bool   | true    | Flag to enable/disable stopping for red light violation                                                           |
| `stop_for_amber_light`          | bool   | false   | Flag to enable/disable stopping for amber light violation                                                         |
| `treat_amber_light_as_red`      | bool   | false   | When true, amber lights are treated identically to red lights (rejection on intersection regardless of distance). |
| `treat_unknown_light_as_red`    | bool   | false   | When true, unknown lights are treated identically to red lights (rejection on intersection).                      |
| `overshoot_tolerance`           | double | 0.0     | [m] Maximum distance between the stop line and the trajectory stop point to consider the trajectory feasible.     |
| `allow_if_cannot_stop_distance` | double | 0.0     | [m] Allow crossing when the ego front is within this distance and ego cannot stop before the stop line.           |
| `th_stable_duration_red`        | double | 0.2     | [s] Minimum duration a RED light must be seen before it is considered active (only when ego is moving).           |
| `th_stable_duration_amber`      | double | 0.2     | [s] Minimum duration an AMBER light must be seen before it is considered active (only when ego is moving).        |
| `th_stable_duration_unknown`    | double | 0.0     | [s] Minimum duration an UNKNOWN light must be seen before it is considered active (only when ego is moving).      |
| `th_amber_rejection_hysteresis` | double | 0.5     | [s] Duration to persist an amber rejection state to prevent "flipping" due to minor velocity/distance changes.    |
| `crossing_time_limit`           | double | 2.75    | [s] Maximum time allowed for the ego vehicle to cross the stop line after an amber light appears.                 |
| `delay_response_time`           | double | 0.5     | [s] Delay response time used to estimate the minimum ego stopping distance.                                       |
| `ego_stopped_vel_th`            | double | 0.01    | [m/s] Velocity below which ego is considered stopped when checking traffic lights.                                |
