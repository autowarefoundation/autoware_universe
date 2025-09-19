# vehicle_cmd_gate

## Purpose

`vehicle_cmd_gate` is the package to get information from emergency handler, planning module, and external controller, and send a message to the vehicle.

## Role

Receive multiple control commands and select one to forward to the vehicle.

## Inputs / Outputs

### Input

| Name                                        | Type                                                | Description                                                          |
| ------------------------------------------- | --------------------------------------------------- | -------------------------------------------------------------------- |
| `~/input/steering`                          | `autoware_vehicle_msgs::msg::SteeringReport`        | steering status                                                      |
| `~/input/auto/control_cmd`                  | `autoware_control_msgs::msg::Control`               | command for lateral and longitudinal velocity from planning module   |
| `~/input/auto/turn_indicators_cmd`          | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command from planning module                         |
| `~/input/auto/hazard_lights_cmd`            | `autoware_vehicle_msgs::msg::HazardLightsCommand`   | hazard lights command from planning module                           |
| `~/input/auto/gear_cmd`                     | `autoware_vehicle_msgs::msg::GearCommand`           | gear command from planning module                                    |
| `~/input/external/control_cmd`              | `autoware_control_msgs::msg::Control`               | command for lateral and longitudinal velocity from external          |
| `~/input/external/turn_indicators_cmd`      | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command from external                                |
| `~/input/external/hazard_lights_cmd`        | `autoware_vehicle_msgs::msg::HazardLightsCommand`   | hazard lights command from external                                  |
| `~/input/external/gear_cmd`                 | `autoware_vehicle_msgs::msg::GearCommand`           | gear command from external                                           |
| `~/input/external_emergency_stop_heartbeat` | `tier4_external_api_msgs::msg::Heartbeat`           | heartbeat                                                            |
| `~/input/gate_mode`                         | `tier4_control_msgs::msg::GateMode`                 | gate mode (AUTO or EXTERNAL)                                         |
| `~/input/emergency/control_cmd`             | `autoware_control_msgs::msg::Control`               | command for lateral and longitudinal velocity from emergency handler |
| `~/input/emergency/turn_indicators_cmd`     | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command from emergency handler                       |
| `~/input/emergency/hazard_lights_cmd`       | `autoware_vehicle_msgs::msg::HazardLightsCommand`   | hazard lights command from emergency handler                         |
| `~/input/emergency/gear_cmd`                | `autoware_vehicle_msgs::msg::GearCommand`           | gear command from emergency handler                                  |
| `~/input/engage`                            | `autoware_vehicle_msgs::msg::Engage`                | engage signal                                                        |
| `~/input/operation_mode`                    | `autoware_adapi_v1_msgs::msg::OperationModeState`   | operation mode of Autoware                                           |

### Output

| Name                                   | Type                                                | Description                                              |
| -------------------------------------- | --------------------------------------------------- | -------------------------------------------------------- |
| `~/output/vehicle_cmd_emergency`       | `tier4_vehicle_msgs::msg::VehicleEmergencyStamped`  | emergency state which was originally in vehicle command  |
| `~/output/command/control_cmd`         | `autoware_control_msgs::msg::Control`               | command for lateral and longitudinal velocity to vehicle |
| `~/output/command/turn_indicators_cmd` | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand` | turn indicators command to vehicle                       |
| `~/output/command/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand`   | hazard lights command to vehicle                         |
| `~/output/command/gear_cmd`            | `autoware_vehicle_msgs::msg::GearCommand`           | gear command to vehicle                                  |
| `~/output/gate_mode`                   | `tier4_control_msgs::msg::GateMode`                 | gate mode (AUTO or EXTERNAL)                             |
| `~/output/engage`                      | `autoware_vehicle_msgs::msg::Engage`                | engage signal                                            |
| `~/output/external_emergency`          | `tier4_external_api_msgs::msg::Emergency`           | external emergency signal                                |
| `~/output/operation_mode`              | `tier4_system_msgs::msg::OperationMode`             | current operation mode of the vehicle_cmd_gate           |

## Parameters

| Parameter                                             | Type     | Description                                                                                                                                                                                 |
| ----------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `update_period`                                       | double   | update period                                                                                                                                                                               |
| `use_emergency_handling`                              | bool     | true when emergency handler is used                                                                                                                                                         |
| `check_external_emergency_heartbeat`                  | bool     | true when checking heartbeat for emergency stop                                                                                                                                             |
| `system_emergency_heartbeat_timeout`                  | double   | timeout for system emergency                                                                                                                                                                |
| `external_emergency_stop_heartbeat_timeout`           | double   | timeout for external emergency                                                                                                                                                              |
| `filter_activated_count_threshold`                    | int      | threshold for filter activation                                                                                                                                                             |
| `filter_activated_velocity_threshold`                 | double   | velocity threshold for filter activation                                                                                                                                                    |
| `stop_hold_acceleration`                              | double   | longitudinal acceleration cmd when vehicle should stop                                                                                                                                      |
| `emergency_acceleration`                              | double   | longitudinal acceleration cmd when vehicle stop with emergency                                                                                                                              |
| `moderate_stop_service_acceleration`                  | double   | longitudinal acceleration cmd when vehicle stop with moderate stop service                                                                                                                  |
| `nominal.vel_lim`                                     | double   | limit of longitudinal velocity (activated in AUTONOMOUS operation mode)                                                                                                                     |
| `nominal.reference_speed_points`                      | <double> | velocity point used as a reference when calculate control command limit (activated in AUTONOMOUS operation mode). The size of this array must be equivalent to the size of the limit array. |
| `nominal.lon_acc_lim_for_lon_vel`                     | <double> | array of limits for longitudinal acceleration (activated in AUTONOMOUS operation mode)                                                                                                      |
| `nominal.lon_jerk_lim_for_lon_acc`                    | <double> | array of limits for longitudinal jerk (activated in AUTONOMOUS operation mode)                                                                                                              |
| `nominal.lat_acc_lim_for_steer_cmd`                   | <double> | array of limits for lateral acceleration (activated in AUTONOMOUS operation mode)                                                                                                           |
| `nominal.lat_jerk_lim_for_steer_cmd`                  | <double> | array of limits for lateral jerk (activated in AUTONOMOUS operation mode)                                                                                                                   |
| `nominal.steer_cmd_lim`                               | <double> | array of limits for steering angle (activated in AUTONOMOUS operation mode)                                                                                                                 |
| `nominal.steer_rate_lim_for_steer_cmd`                | <double> | array of limits for command steering rate (activated in AUTONOMOUS operation mode)                                                                                                          |
| `nominal.lat_jerk_lim_for_steer_rate`                 | double   | limit for lateral jerk constraint on steering rate (activated in AUTONOMOUS operation mode)                                                                                                 |
| `nominal.steer_cmd_diff_lim_from_current_steer`       | <double> | array of limits for difference between current and command steering angle (activated in AUTONOMOUS operation mode)                                                                          |
| `on_transition.vel_lim`                               | double   | limit of longitudinal velocity (activated in TRANSITION operation mode)                                                                                                                     |
| `on_transition.reference_speed_points`                | <double> | velocity point used as a reference when calculate control command limit (activated in TRANSITION operation mode). The size of this array must be equivalent to the size of the limit array. |
| `on_transition.lon_acc_lim_for_lon_vel`               | <double> | array of limits for longitudinal acceleration (activated in TRANSITION operation mode)                                                                                                      |
| `on_transition.lon_jerk_lim_for_lon_acc`              | <double> | array of limits for longitudinal jerk (activated in TRANSITION operation mode)                                                                                                              |
| `on_transition.lat_acc_lim_for_steer_cmd`             | <double> | array of limits for lateral acceleration (activated in TRANSITION operation mode)                                                                                                           |
| `on_transition.lat_jerk_lim_for_steer_cmd`            | <double> | array of limits for lateral jerk (activated in TRANSITION operation mode)                                                                                                                   |
| `on_transition.steer_cmd_lim`                         | <double> | array of limits for steering angle (activated in TRANSITION operation mode)                                                                                                                 |
| `on_transition.steer_rate_lim_for_steer_cmd`          | <double> | array of limits for command steering rate (activated in TRANSITION operation mode)                                                                                                          |
| `on_transition.lat_jerk_lim_for_steer_rate`           | double   | limit for lateral jerk constraint on steering rate (activated in TRANSITION operation mode)                                                                                                 |
| `on_transition.steer_cmd_diff_lim_from_current_steer` | <double> | array of limits for difference between current and command steering angle (activated in TRANSITION operation mode)                                                                          |

### Parameter Naming Convention

The parameters follow specific naming patterns to clearly distinguish between different types of constraints and their relationships:

#### Pattern 1: `[constraint]_lim_for_[target]`

- **Format**: `[physical_constraint]_lim_for_[controlled_variable]`
- **Description**: Defines limits based on physical constraints (acceleration, jerk, etc.) applied to control variables

#### Pattern 2: `[target]_[constraint]_lim_from_[reference]`

- **Format**: `[controlled_variable]_[constraint_type]_lim_from_[reference_variable]`
- **Description**: Defines limits on the difference or deviation of a control variable from a reference value

#### Pattern 3: `[target]_lim`

- **Format**: `[controlled_variable]_lim`
- **Description**: Defines absolute limits for control variables

## Functionality

### Main Functionality

- Receive multiple control commands (from Autoware planning, emergency handler, remote control, etc.) and select one to forward to the vehicle.
- Apply a final guard on the selected command to enforce absolute safety limits (e.g., maximum steering rate). This is not a comfort filter.
- Enforce transition guards during mode changes into autonomous driving (e.g., remote→autonomous, manual→autonomous) to limit sudden changes. Integration with the Operation Transition Manager is recommended, though code boundaries should be maintained due to its complexity.

### Sub-Functionality

- Check heartbeat signals to verify connectivity for each input (e.g., emergency external heartbeat).
- Publish status indicating whether the final guard is active. Active guard in autonomous mode implies an unexpected constraint in command generation and requires attention.
- Leverage guard status during mode transitions to notify operators/drivers that a strong constraint is active (focus on "transition in progress" rather than simple filter activation).

### Filter function

This module incorporates a limitation filter to the control command right before its published. Primarily for safety, this filter restricts the output range of all control commands published through Autoware.

The limitation values are calculated based on the 1D interpolation of the limitation array parameters. Here is an example for the longitudinal jerk limit.

![filter-example](./image/filter.png)

Notation: this filter is not designed to enhance ride comfort. Its main purpose is to detect and remove abnormal values in the control outputs during the final stages of Autoware. If this filter is frequently active, it implies the control module may need tuning. If you're aiming to smoothen the signal via a low-pass filter or similar techniques, that should be handled in the control module. When the filter is activated, the topic `~/is_filter_activated` is published.

Notation 2: If you use vehicles in which the driving force is controlled by the accelerator/brake pedal, the jerk limit, denoting the pedal rate limit, must be sufficiently relaxed at low speeds.
Otherwise, quick pedal changes at start/stop will not be possible, resulting in slow starts and creep down on hills.
This functionality for starting/stopping was embedded in the source code but was removed because it was complex and could be achieved by parameters.

## Assumptions / Known limits

### External Emergency Heartbeat

The parameter `check_external_emergency_heartbeat` (true by default) enables an emergency stop request from external modules.
This feature requires a `~/input/external_emergency_stop_heartbeat` topic for health monitoring of the external module, and the vehicle_cmd_gate module will not start without the topic.
The `check_external_emergency_heartbeat` parameter must be false when the "external emergency stop" function is not used.

### Commands on Mode changes

Output commands' topics: `turn_indicators_cmd`, `hazard_light` and `gear_cmd` are selected based on `gate_mode`.
However, to ensure the continuity of commands, these commands will not change until the topics of new input commands arrive, even if a mode change occurs.

## Caution

- This node depends on the Operation Mode Transition Manager for Engage state transitions at the design level.
- Tests are essential and must be retained.
