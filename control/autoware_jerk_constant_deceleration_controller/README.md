# autoware_jerk_constant_deceleration_controller

## Overview

This node overrides the control command with a constant-jerk deceleration profile when triggered.
It is used as part of the MRM (Minimum Risk Maneuver) pipeline to bring the vehicle to a stop
smoothly using a fixed target acceleration and jerk.

When the trigger is inactive, the node passes through the upstream control command unchanged.
When activated, it replaces the longitudinal velocity and acceleration using the configured
target values, applying a constant jerk until the target acceleration is reached.
In addition, it always publishes a fixed gear (DRIVE), hazard lights (ENABLE), and
turn indicators (DISABLE) command alongside the control output.

---

## Behavior

```
trigger.trigger = false:
  input/control  ──────────────────────────────▶  output/control  (pass-through)

trigger.trigger = true:
  input/control (stamp only)  ──▶  constant-jerk deceleration  ──▶  output/control
  target_acceleration, target_jerk from trigger message
```

Longitudinal override:

```
velocity     = max(prev_velocity + prev_acceleration * dt, 0)
acceleration = max(prev_acceleration + target_jerk * dt, target_acceleration)
jerk         = target_jerk  (0 once target_acceleration is reached)
```

---

## Node: `jerk_constant_deceleration_controller`

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `~/input/control` | `autoware_control_msgs/Control` | Upstream control command |
| `~/input/jerk_constant_deceleration_trigger` | `tier4_control_msgs/JerkConstantDecelerationTrigger` | Trigger and deceleration parameters |

### Publications

| Topic | Type | Description |
|---|---|---|
| `~/output/control` | `autoware_control_msgs/Control` | Overridden (or passed-through) control command |
| `~/output/gear_command` | `autoware_vehicle_msgs/GearCommand` | Always DRIVE |
| `~/output/hazard_lights_command` | `autoware_vehicle_msgs/HazardLightsCommand` | Always ENABLE |
| `~/output/turn_indicators_command` | `autoware_vehicle_msgs/TurnIndicatorsCommand` | Always DISABLE |

### Parameters

No parameters are currently defined.

---

## Launch

```xml
<include file="$(find-pkg-share autoware_jerk_constant_deceleration_controller)/launch/jerk_constant_deceleration_controller.launch.xml">
  <arg name="input_control" value="/control/trajectory_follower/control_cmd"/>
  <arg name="input_jerk_constant_deceleration_trigger" value="/control/jerk_constant_deceleration_trigger"/>
  <arg name="output_control" value="/control/control_command_gate/inputs/in_lane_stop/control"/>
</include>
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `input_control` | `/control/trajectory_follower/control_cmd` | Upstream control command topic |
| `input_jerk_constant_deceleration_trigger` | `/control/jerk_constant_deceleration_trigger` | Trigger topic |
| `output_control` | `/control/control_command_gate/inputs/in_lane_stop/control` | Output control command topic |
| `output_gear_command` | `/control/control_command_gate/inputs/in_lane_stop/gear` | Output gear command topic |
| `output_hazard_lights_command` | `/control/control_command_gate/inputs/in_lane_stop/hazard_lights` | Output hazard lights topic |
| `output_turn_indicators_command` | `/control/control_command_gate/inputs/in_lane_stop/turn_indicators` | Output turn indicators topic |
