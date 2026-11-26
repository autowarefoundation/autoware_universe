# external_cmd_converter

`external_cmd_converter` is a node that converts desired mechanical input to acceleration and velocity by using accel/brake map.

## Algorithm

### How to calculate reference acceleration and velocity

A reference acceleration and velocity are derived from the throttle and brake values of the pedals command.

#### Reference Acceleration

A reference acceleration is calculated from accel_brake_map based on values of a desired_pedal and the absolute value of the current velocity;

$$
    pedal_d = throttle_d - brake_d,
$$

$$
    acc_{ref} = Acc(pedal_d, |v_{x,current}|).
$$

| Parameter       | Description                                                           |
| --------------- | --------------------------------------------------------------------- |
| $throttle_d$    | throttle value of the pedals command (`in/pedals_cmd`, `throttle`)    |
| $brake_d$       | brake value of the pedals command (`in/pedals_cmd`, `brake`)          |
| $v_{x,current}$ | current longitudinal velocity (`in/odometry`, `twist.twist.linear.x`) |
| Acc             | accel_brake_map                                                       |

#### Reference Velocity

A reference velocity is calculated based on a current velocity and a reference acceleration:

$$
v_{ref} =
    v_{x,current} + k_{v_{ref}} \cdot \text{sign}_{gear} \cdot acc_{ref}.
$$

| Parameter            | Description                                                        |
| -------------------- | ------------------------------------------------------------------ |
| $acc_{ref}$          | reference acceleration                                             |
| $k_{v_{ref}}$        | reference velocity gain                                            |
| $\text{sign}_{gear}$ | gear command (`in/gear_cmd`) (Drive/Low: 1, Reverse: -1, Other: 0) |

## Input topics

| Name                   | Type                                               | Description                                                                                                                                                |
| ---------------------- | -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `in/pedals_cmd`        | autoware_adapi_v1_msgs/msg/PedalsCommand           | throttle and brake. Each message triggers the conversion once `in/odometry`, `in/gear_cmd` and `in/steering_cmd` have been received.                       |
| `in/steering_cmd`      | autoware_adapi_v1_msgs/msg/SteeringCommand         | steering tire angle and velocity, copied to the control command.                                                                                           |
| `in/gear_cmd`          | autoware_vehicle_msgs/msg/GearCommand              | current gear command, which gives the sign of the reference velocity.                                                                                      |
| `in/heartbeat`         | autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat | heartbeat of the operator. Only messages with `ready: true` count for `emergency_stop_timeout`.                                                            |
| `in/current_gate_mode` | tier4_control_msgs/msg/GateMode                    | gate mode. Until it is received, neither timeout is checked. In `AUTO` mode, the stored receive times are cleared, as if no message had been received yet. |
| `in/odometry`          | nav_msgs/msg/Odometry                              | current velocity (`twist.twist.linear.x`).                                                                                                                 |

## Output topics

| Name              | Type                              | Description                                                                                         |
| ----------------- | --------------------------------- | --------------------------------------------------------------------------------------------------- |
| `out/control_cmd` | autoware_control_msgs/msg/Control | control command with the reference velocity and acceleration, and the steering of `in/steering_cmd` |

## Parameters

| Parameter                 | Type   | Description                                           |
| ------------------------- | ------ | ----------------------------------------------------- |
| `csv_path_accel_map`      | string | CSV file path for accel map                           |
| `csv_path_brake_map`      | string | CSV file path for brake map                           |
| `ref_vel_gain`            | double | reference velocity gain                               |
| `timer_rate`              | double | timer's update rate [Hz]                              |
| `wait_for_first_topic`    | bool   | if time out check is done after receiving first topic |
| `control_command_timeout` | double | time out check for `in/pedals_cmd` [s]                |
| `emergency_stop_timeout`  | double | time out check for `in/heartbeat` [s]                 |

## Limitation

tbd.
