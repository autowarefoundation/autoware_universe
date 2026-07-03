# autoware_mrm_in_lane_stop_operator

## Overview

The `autoware_mrm_in_lane_stop_operator` is a ROS 2 node that implements a Minimum Risk Maneuver (MRM) operator specifically designed for in-lane stop functionality. It acts as a bridge between the **driving mode manager** and the **in-lane stop execution modules** (e.g., `in_lane_mrm_planner`).

### System Role

In the MRM decision flow:

1. **Driving Mode Manager** detects an emergency or fallback condition and issues a driving mode request for in-lane stop MRM
2. **MrmInLaneStopOperator** receives the activation request and:
   - Validates the requested mode against configured modes
   - Communicates with the relay controller to enable the constant jerk deceleration trigger topic
   - Publishes `ConstantJerkDecelerationTrigger` messages to signal the in-lane stop planner to execute the maneuver
   - Tracks MRM state transitions (UNKNOWN → NORMAL/OPERATING → SUCCEEDED)
   - Detects when the vehicle has come to a complete stop
3. **In-lane Stop Execution Modules** (e.g., `in_lane_mrm_planner`) receive the trigger and execute the controlled deceleration trajectory

### Key Characteristics

- Operates at a higher level than motion control; focuses on **mode lifecycle management** and **relay coordination**
- Relies on polling-based vehicle stop detection for robust state confirmation
- Ensures relay service availability before committing to mode activation (fail-safe behavior)
- Publishes both MRM state and per-mode `DrivingModeActive` flags for system-wide visibility

## Features

- **Flexible Driving Mode Management**: Support for multiple configurable driving modes with per-mode deceleration parameters
- **Relay Service Control**: Seamlessly switches between normal operation and MRM mode via topic relay control
- **MRM State Machine**: Tracks MRM operation state (UNKNOWN → NORMAL/OPERATING → SUCCEEDED)
- **Vehicle Stop Detection**: Polls kinematic state to detect when the vehicle has come to a complete stop
- **Driving Mode Active Flags**: Per-mode publication of `DrivingModeActive` flags for external monitoring
- **Configurable Launch Parameters**: All topic and service endpoints are remappable via launch arguments

- **Flexible Driving Mode Management**: Support for multiple configurable driving modes with per-mode deceleration parameters
- **Relay Service Control**: Seamlessly switches between normal operation and MRM mode via topic relay control
- **MRM State Machine**: Tracks MRM operation state (UNKNOWN → NORMAL/OPERATING → SUCCEEDED)
- **Vehicle Stop Detection**: Polls kinematic state to detect when the vehicle has come to a complete stop
- **Driving Mode Active Flags**: Per-mode publication of `DrivingModeActive` flags for external monitoring
- **Configurable Launch Parameters**: All topic and service endpoints are remappable via launch arguments

## Architecture

### Input/Output

#### Subscriptions
- `/localization/kinematic_state` (nav_msgs/Odometry)
  - Vehicle odometry used for stop detection (polling, not callback-based)
- `~/input/driving_mode_request` (tier4_system_msgs/DrivingModeRequest)
  - Requests to activate/deactivate driving modes
- `~/input/driving_mode_info` (tier4_system_msgs/DrivingModeInfo)
  - Current driving mode status information

#### Publishers
- `~/output/mrm_state` (tier4_system_msgs/DrivingModeMrmState)
  - MRM operation state (UNKNOWN, NORMAL, OPERATING, SUCCEEDED)
- `~/output/jerk_deceleration_trigger` (tier4_control_msgs/ConstantJerkDecelerationTrigger)
  - Trigger signal for constant jerk deceleration when MRM is active
- `~/output/driving_mode_active` (tier4_system_msgs/DrivingModeFlag)
  - Flags indicating active status for each configured driving mode

#### Service Clients
- Relay control service (default: `/system/topic_relay_controller_trajectory/operate`)
  - Used to enable/disable the constant jerk deceleration trigger

### MRM State Machine

```
┌─────────┐
│ UNKNOWN │  (initial state)
└────┬────┘
     │
     ├─ [mode requested inactive] → NORMAL
     │
     ├─ [mode requested active] → OPERATING
     │                  ↓
     │          [vehicle stopped] → SUCCEEDED
     │
     └─ [any error] → remains in current state
```

**State Descriptions:**
- `UNKNOWN`: Initial state before any mode request
- `NORMAL`: Mode is inactive (requested_mode_id is not set)
- `OPERATING`: Mode is active and vehicle is still moving
- `SUCCEEDED`: Mode is active and vehicle has stopped (abs(velocity) < 0.001 m/s)

### Relay Service Integration

The node communicates with a relay controller service to enable/disable the constant jerk deceleration topic routing. On mode activation:
1. Calls the relay service with `enable=true`
2. Only updates internal `active_mode_id_` if the relay service call succeeds
3. On mode deactivation, calls relay with `enable=false`

This ensures that if the relay service is unavailable, the node does not incorrectly track mode state.

## Configuration

### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `jerk_deceleration_trigger_topic` | string | `/control/constant_jerk_deceleration_trigger` | Topic for jerk deceleration trigger |
| `relay_service_name` | string | `/system/topic_relay_controller_trajectory/operate` | Service name for relay control |

### Parameters (YAML)

**Global Parameters:**
- `mode_names` (list of strings): Names of driving modes to manage
- `service_timeout_ms` (int): Timeout for relay service calls [ms]

**Per-Mode Parameters:**
Each mode in `mode_names` has a configuration block:
```yaml
<mode_name>:
  target_acceleration: <double>  # Target acceleration [m/s²]
  target_jerk: <double>          # Target jerk [m/s³]
```

**Example:**
```yaml
/**:
  ros__parameters:
    mode_names:
      - in_lane_moderate_stop
    service_timeout_ms: 100
    in_lane_moderate_stop:
      target_acceleration: -2.5
      target_jerk: -1.5
```

## Usage

### Basic Launch

```bash
ros2 launch autoware_mrm_in_lane_stop_operator mrm_in_lane_stop_operator.launch.xml
```

### Custom Parameters

```bash
ros2 launch autoware_mrm_in_lane_stop_operator mrm_in_lane_stop_operator.launch.xml \
  jerk_deceleration_trigger_topic:=/custom/trigger \
  relay_service_name:=/custom/relay_service
```
