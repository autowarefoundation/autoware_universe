# Autoware Hazard Lights Selector

## Purpose

The Hazard Lights Selector is a node that manages and selects hazard light commands from multiple sources (planning and system) to determine the final hazard lights state of the vehicle.

## Inner-workings / Algorithms

The node operates on a timer-based update cycle and implements the following logic:

1. Receives hazard light commands from both planning and system sources
2. If either source requests hazard lights to be enabled, the final command will be to enable hazard lights
3. If neither source requests hazard lights to be enabled, the final command will be to disable hazard lights
4. The selected command is published at a configurable update rate

## Inputs / Outputs

### Input

| Name                                   | Type                                            | Description                                    |
| -------------------------------------- | ----------------------------------------------- | ---------------------------------------------- |
| `input/planning/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard lights command from the planning system |
| `input/system/hazard_lights_command`   | `autoware_vehicle_msgs/msg/HazardLightsCommand` | Hazard lights command from the system          |

### Output

| Name                           | Type                                            | Description                        |
| ------------------------------ | ----------------------------------------------- | ---------------------------------- |
| `output/hazard_lights_command` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | The selected hazard lights command |

## Parameters

| Name          | Type | Default Value | Description                                                 |
| ------------- | ---- | ------------- | ----------------------------------------------------------- |
| `update_rate` | int  | 10            | The update rate in Hz for publishing hazard lights commands |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
