# autoware_redundancy_command_selector

## Purpose

This node simulates ECU redundancy switching behavior in `autoware_simple_planning_simulator`.
Instead of switching the output ECU itself in simulation, this node switches between Main ECU and Sub ECU control commands and forwards the selected stream.

## Inner-workings / Algorithms

- Subscribe to command topics from both Main ECU and Sub ECU
- Subscribe to `/system/redundancy/active_control_unit`
- If `main_ecu_id` is active, forward Main commands
- If `sub_ecu_id` is active, forward Sub commands
- If both (or neither) are active, keep the current selection to avoid unnecessary switching

## Inputs / Outputs

### Input

| Name | Type | Description |
| --- | --- | --- |
| `~/input/main/control_command` | `autoware_control_msgs/msg/Control` | Main ECU control command |
| `~/input/main/gear_command` | `autoware_vehicle_msgs/msg/GearCommand` | Main ECU gear command |
| `~/input/main/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Main ECU hazard lights command |
| `~/input/main/turn_indicators_command` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Main ECU turn indicators command |
| `~/input/sub/control_command` | `autoware_control_msgs/msg/Control` | Sub ECU control command |
| `~/input/sub/gear_command` | `autoware_vehicle_msgs/msg/GearCommand` | Sub ECU gear command |
| `~/input/sub/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Sub ECU hazard lights command |
| `~/input/sub/turn_indicators_command` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Sub ECU turn indicators command |
| `~/input/active_control_unit` | `tier4_system_msgs/msg/ActiveControlUnit` | Active ECU IDs |

### Output

| Name | Type | Description |
| --- | --- | --- |
| `~/output/control_command` | `autoware_control_msgs/msg/Control` | Selected control command |
| `~/output/gear_command` | `autoware_vehicle_msgs/msg/GearCommand` | Selected gear command |
| `~/output/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Selected hazard lights command |
| `~/output/turn_indicators_command` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | Selected turn indicators command |

## Parameters

| Name | Type | Description |
| --- | --- | --- |
| `main_ecu_id` | `uint8` | ECU ID treated as Main |
| `sub_ecu_id` | `uint8` | ECU ID treated as Sub |

## Launch

- XML launch: `launch/redundancy_command_selector.launch.xml`

Include this launch file from the simulator-side launch to reproduce redundant ECU switching behavior by switching Main/Sub command streams.
