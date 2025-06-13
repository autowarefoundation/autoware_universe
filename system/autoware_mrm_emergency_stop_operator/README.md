# mrm_emergency_stop_operator

## Purpose

MRM emergency stop operator is a node that generates emergency stop commands according to the emergency stop MRM order.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                 | Type                                  | Description                                                                                                                   |
| ------------------------------------ | ------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `~/input/mrm/emergency_stop/operate` | `tier4_system_msgs::srv::OperateMrm`  | MRM execution order                                                                                                           |
| `~/input/control/control_cmd`        | `autoware_control_msgs::msg::Control` | Control command output from the last node of the control component. Used for the initial value of the emergency stop command. |
|                                      |                                       |                                                                                                                               |

### Output

| Name                                      | Type                                        | Description            |
| ----------------------------------------- | ------------------------------------------- | ---------------------- |
| `~/output/mrm/emergency_stop/status`      | `tier4_system_msgs::msg::MrmBehaviorStatus` | MRM execution status   |
| `~/output/mrm/emergency_stop/control_cmd` | `autoware_control_msgs::msg::Control`       | Emergency stop command |

## Parameters

{{ json_to_markdown("system/mrm_emergency_stop_operator/schema/mrm_emergency_stop_operator.schema.json") }}

## Assumptions / Known limits

TBD.
