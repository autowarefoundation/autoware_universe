# autoware_command_mode_switcher_plugins

## Overview

`autoware_command_mode_switcher_plugins` is a package that manages plugins used by `autoware_command_mode_switcher`.

`autoware_command_mode_switcher` is a node that manages switching between multiple control sources during MRM (Minimal Risk Maneuver) execution. Each plugin is implemented by inheriting the `CommandPlugin` interface defined in `autoware_command_mode_switcher/command_plugin.hpp`.

Each plugin implements the following methods:

| Method                              | Description                                                                                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `mode()`                            | Returns the command mode ID that this plugin corresponds to                                                                                      |
| `source()`                          | Returns the control source ID that this plugin uses                                                                                              |
| `autoware_control()`                | Returns whether Autoware controls the vehicle                                                                                                    |
| `initialize()`                      | Initializes publishers, subscribers, and service clients                                                                                         |
| `update_source_state(bool request)` | Manages source gate open/close based on the switcher selection state (`request`) and executes side effects (deceleration command, relay control) |
| `update_mrm_state()`                | Returns the MRM execution state (`Normal` / `Operating` / `Succeeded`)                                                                           |

---

## Plugins

### `StopSwitcher`

**Class**: `autoware::command_mode_switcher::StopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::stop`

### `AutonomousSwitcher`

**Class**: `autoware::command_mode_switcher::AutonomousSwitcher`  
**Mode**: `autoware::command_mode_types::modes::autonomous`

### `LocalSwitcher`

**Class**: `autoware::command_mode_switcher::LocalSwitcher`  
**Mode**: `autoware::command_mode_types::modes::local`

### `RemoteSwitcher`

**Class**: `autoware::command_mode_switcher::RemoteSwitcher`  
**Mode**: `autoware::command_mode_types::modes::remote`

### `EmergencyStopSwitcher`

**Class**: `autoware::command_mode_switcher::EmergencyStopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::emergency_stop`

### `ComfortableStopSwitcher`

**Class**: `autoware::command_mode_switcher::ComfortableStopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::comfortable_stop`

### `PullOverSwitcher`

**Class**: `autoware::command_mode_switcher::PullOverSwitcher`  
**Mode**: `autoware::command_mode_types::modes::pull_over`

---

## x2 Plugins

The following plugins are designed for a redundant Main ECU / Sub ECU configuration used in the x2 product.

### `MainEcuInLaneEmergencyStopSwitcher`

**Class**: `autoware::command_mode_switcher::MainEcuInLaneEmergencyStopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::main_ecu_in_lane_emergency_stop`  
**Source**: `autoware::command_mode_types::sources::in_lane_stop`

#### Overview

A switcher plugin that triggers in_lane_emergency_stop on the Main ECU in a redundant Main ECU / Sub ECU configuration. Its purpose is to bring the vehicle to an emergency stop within the current driving lane when the normal autonomous driving function fails.

#### Functionality

This switcher is composed of the following three functions.

**1. State Management (`update_source_state` / `update_mrm_state`)**

Conforming to the `autoware_command_mode_switcher` framework, this function manages source gate open/close and MRM execution state based on the switcher selection state (`request`) and `mrm_state_`.

The state transitions in `update_source_state(bool request)` are as follows:

| request | mrm*state*                | Action                                            | SourceState     |
| ------- | ------------------------- | ------------------------------------------------- | --------------- |
| `true`  | `Operating`               | No action (already operating)                     | `{true, false}` |
| `true`  | `Succeeded`               | No action (already stopped)                       | `{true, false}` |
| `false` | `Normal`                  | No action (already released)                      | `{false, true}` |
| `true`  | `Normal`                  | Trigger ON, relay OFF → transition to `Operating` | `{true, false}` |
| `false` | `Operating` / `Succeeded` | Trigger OFF, relay ON → transition to `Normal`    | `{false, true}` |

`update_mrm_state()` monitors the odometry from `/localization/kinematic_state` while in the `Operating` state, and transitions to `Succeeded` when the vehicle speed falls below 0.001 m/s.

**2. Deceleration Command (`publish_constant_jerk_deceleration_trigger`)**

Publishes a trigger message to the `constant_jerk_deceleration_controller` node specifying the target deceleration and jerk values. An ON trigger is published when the switcher is selected, and an OFF trigger is published to cancel when the switcher is deselected.

- Publisher: `/control/constant_jerk_deceleration_trigger`
- The deceleration profile is specified via the `target_acceleration` and `target_jerk` parameters.
- For emergency stop, values corresponding to hard braking are expected to be configured.

**3. Topic Relay Stop Control (`request_topic_relay_control`)**

During normal autonomous driving, in_lane_stop continuously captures `pose_with_covariance` and `trajectory`, and uses the most recent valid data at the time of failure to control the vehicle. However, if unreliable sensor data or path planning continues to flow in after a failure, there is a risk of incorporating untrustworthy data. To prevent this, the `topic_relay_controller` filters these topics while the switcher is active.

- Service client: `/system/topic_relay_controller_trajectory/operate`
- Service client: `/system/topic_relay_controller_pose_with_covariance/operate`
- Relay is turned OFF (filtering starts) when the switcher is selected, and turned ON (filtering ends) when deselected.
- Each relay can be individually enabled or disabled via the `enable_trajectory_relay` and `enable_pose_with_covariance_relay` parameters.

#### Parameters

| Parameter                           | Type     | Description                                                                   |
| ----------------------------------- | -------- | ----------------------------------------------------------------------------- |
| `target_acceleration`               | `double` | Target deceleration commanded to constant_jerk_deceleration_controller [m/s²] |
| `target_jerk`                       | `double` | Target jerk commanded to constant_jerk_deceleration_controller [m/s³]         |
| `enable_trajectory_relay`           | `bool`   | Enable/disable trajectory relay control                                       |
| `enable_pose_with_covariance_relay` | `bool`   | Enable/disable pose_with_covariance relay control                             |
| `service_timeout_ms`                | `int64`  | Timeout for service calls [ms]                                                |

---

### `MainEcuInLaneModerateStopSwitcher`

**Class**: `autoware::command_mode_switcher::MainEcuInLaneModerateStopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::main_ecu_in_lane_moderate_stop`  
**Source**: `autoware::command_mode_types::sources::in_lane_stop`

#### Overview

A switcher plugin that triggers in_lane_moderate_stop on the Main ECU in a redundant Main ECU / Sub ECU configuration.

The implementation is identical to `MainEcuInLaneEmergencyStopSwitcher`, providing the same three functions: state management, deceleration trigger publishing, and topic relay control. See [`MainEcuInLaneEmergencyStopSwitcher`](#mainecuinlaneemergencystopswitcher) for details.

The only difference from `MainEcuInLaneEmergencyStopSwitcher` is that the `target_acceleration` and `target_jerk` parameters are expected to be configured with a gentler deceleration profile. While the emergency stop plugin handles hard braking to stop the vehicle immediately upon anomaly detection, the moderate stop plugin handles a smoother deceleration that reduces discomfort to occupants.

#### Parameters

Same as `MainEcuInLaneEmergencyStopSwitcher`. See the [parameter table above](#parameters).

---

### `SubEcuInLaneModerateStopSwitcher`

**Class**: `autoware::command_mode_switcher::SubEcuInLaneModerateStopSwitcher`  
**Mode**: `autoware::command_mode_types::modes::sub_ecu_in_lane_moderate_stop`  
**Source**: `autoware::command_mode_types::sources::in_lane_stop`

#### Overview

A switcher plugin that triggers in_lane_moderate_stop on the Sub ECU in a redundant Main ECU / Sub ECU configuration.

The implementation and functionality are identical to `MainEcuInLaneModerateStopSwitcher`, providing state management, deceleration trigger publishing, and topic relay control. See [`MainEcuInLaneEmergencyStopSwitcher`](#mainecuinlaneemergencystopswitcher) for details.

The reason this plugin is defined separately from `MainEcuInLaneModerateStopSwitcher` is that its mode ID is defined as `sub_ecu_in_lane_moderate_stop` for the Sub ECU. This allows `autoware_command_mode_switcher` to manage the Main ECU and Sub ECU switchers independently.

#### Parameters

Same as `MainEcuInLaneEmergencyStopSwitcher`. See the [parameter table above](#parameters).

---

### `SubEcuStandbySwitcher`

**Class**: `autoware::command_mode_switcher::SubEcuStandbySwitcher`  
**Mode**: `autoware::command_mode_types::modes::sub_ecu_standby`  
**Source**: `autoware::command_mode_types::sources::in_lane_stop`

#### Overview

The switcher plugin selected on the Sub ECU while the Main ECU is in control during normal operation in a redundant Main ECU / Sub ECU configuration.

`initialize()` is an empty implementation with no publishers, subscribers, or service clients. Since `update_source_state` and `update_mrm_state` are not overridden, the base class default implementations are used. In other words, the Sub ECU makes no intervention to the vehicle command while this switcher is selected.

#### Design Intent

In a redundant configuration, the Sub ECU doing nothing has a clear purpose. If the Sub ECU were to independently issue deceleration commands or relay control while the Main ECU is operating normally, it would risk interfering with the Main ECU's control commands. By keeping the Sub ECU in a deliberately inactive state with `SubEcuStandbySwitcher` selected, the same control commands as the Main ECU are passed through to the vehicle without modification.

When the Main ECU fails and the Sub ECU takes over control, a different switcher such as `sub_ecu_in_lane_moderate_stop` is selected, which initiates active MRM control on the Sub ECU side.
