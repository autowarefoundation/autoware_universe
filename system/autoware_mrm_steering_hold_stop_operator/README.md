# autoware_mrm_steering_hold_stop_operator

## Overview

The `autoware_mrm_steering_hold_stop_operator` publishes the control command of an in-lane stop MRM that does not depend on any planner or trajectory follower.
When the MRM is triggered, it holds the steering angle measured at the MRM start and stops the vehicle with a constant-jerk deceleration computed open-loop from the measured velocity and acceleration.

It subscribes to the same `InLaneStopTrigger` as the in-lane MRM planner, so it can be used as an alternative publisher of the in-lane stop command source of the control command gate.

- **Trigger OFF**: mirrors the measured vehicle state (steering, velocity, and acceleration clamped to `<= 0`) on its own timer, so that the gate input is alive and continuous before the gate switches to it.
- **Trigger ON**: latches the measured steering angle, velocity, and acceleration, then decelerates open-loop with the `target_jerk` down to the `target_acceleration` of the requested profile. The output continues after the velocity reaches zero.
- **Required inputs stale**: publishes nothing so that the input timeout of the control command gate escalates to its fallback, and reports the reason on `~/debug/status` and the error log.

Note that holding the steering angle does not guarantee that the vehicle stays in its lane.

## State machine

| State           | Condition                                         | Output                                     |
| --------------- | ------------------------------------------------- | ------------------------------------------ |
| `MIRROR`        | trigger OFF and required inputs are fresh         | measured state                             |
| `WAITING_INPUT` | required inputs are stale before the deceleration | none                                       |
| `DECELERATING`  | trigger ON and initialized from the fresh inputs  | held steering + constant-jerk deceleration |

- The required inputs are `steering_status` and `kinematic_state`. `acceleration` is also required at or above `low_speed_threshold`.
- Once `DECELERATING`, the deceleration continues even if the inputs become stale.
- Below `low_speed_threshold`, the acceleration measurement is not used and the deceleration starts directly at `target_acceleration`.
- A profile change while the trigger is ON updates only the targets. The held steering angle is kept.
- Trigger OFF returns to `MIRROR`. The next trigger latches the measurements again.

## Deceleration profiles

The deceleration constraints are owned by this node as parameters per `InLaneStopTrigger` profile.

| `InLaneStopTrigger::profile` | Parameters used                       |
| ---------------------------- | ------------------------------------- |
| `PROFILE_MODERATE`           | `profiles.moderate`                   |
| `PROFILE_EMERGENCY`          | `profiles.emergency`                  |
| others (`PROFILE_UNKNOWN`)   | `profiles.moderate` with an error log |

An unknown profile still decelerates, because not stopping is more dangerous than stopping with the moderate constraints.

## Interfaces

### Input

| Name                      | Type                                           | Description                                                                        |
| ------------------------- | ---------------------------------------------- | ---------------------------------------------------------------------------------- |
| `~/input/trigger`         | `tier4_system_msgs/msg/InLaneStopTrigger`      | in-lane stop trigger and deceleration profile (reliable, transient_local, depth 1) |
| `~/input/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport`     | measured steering angle                                                            |
| `~/input/kinematic_state` | `nav_msgs/msg/Odometry`                        | measured velocity                                                                  |
| `~/input/acceleration`    | `geometry_msgs/msg/AccelWithCovarianceStamped` | measured acceleration                                                              |

The trigger is `transient_local`, so a node restarted during the MRM receives the latest trigger and continues to decelerate.
In that case the steering angle is latched again from the measurement after the restart.

### Output

| Name               | Type                                                        | Description                        |
| ------------------ | ----------------------------------------------------------- | ---------------------------------- |
| `~/output/control` | `autoware_control_msgs/msg/Control`                         | control command for the gate input |
| `~/debug/status`   | `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped` | internal state (see below)         |

`~/debug/status` layout:

| Index | Content                                                    |
| ----- | ---------------------------------------------------------- |
| 0     | state (0: `MIRROR`, 1: `WAITING_INPUT`, 2: `DECELERATING`) |
| 1     | trigger (1: ON, 0: OFF)                                    |
| 2     | age of `steering_status` [s] (-1: never received)          |
| 3     | age of `kinematic_state` [s] (-1: never received)          |
| 4     | age of `acceleration` [s] (-1: never received)             |
| 5     | held steering angle [rad]                                  |
| 6     | commanded velocity [m/s]                                   |
| 7     | commanded acceleration [m/s^2]                             |
| 8     | target acceleration of the resolved profile [m/s^2]        |
| 9     | target jerk of the resolved profile [m/s^3]                |
| 10    | received `InLaneStopTrigger::profile`                      |

## Parameters

{{ json_to_markdown("system/autoware_mrm_steering_hold_stop_operator/schema/mrm_steering_hold_stop_operator.schema.json") }}
