# mrm_comfortable_stop_operator

## Purpose

MRM comfortable stop operator is a node that generates comfortable stop commands according to the comfortable stop MRM order.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                   | Type                                 | Description         |
| -------------------------------------- | ------------------------------------ | ------------------- |
| `~/input/mrm/comfortable_stop/operate` | `tier4_system_msgs::srv::OperateMrm` | MRM execution order |

### Output

| Name                                   | Type                                                              | Description                  |
| -------------------------------------- | ----------------------------------------------------------------- | ---------------------------- |
| `~/output/mrm/comfortable_stop/status` | `tier4_system_msgs::msg::MrmBehaviorStatus`                       | MRM execution status         |
| `~/output/velocity_limit`              | `autoware_internal_planning_msgs::msg::VelocityLimit`             | Velocity limit command       |
| `~/output/velocity_limit/clear`        | `autoware_internal_planning_msgs::msg::VelocityLimitClearCommand` | Velocity limit clear command |

## Parameters

{{ json_to_markdown("system/mrm_comfortable_stop_operator/schema/mrm_comfortable_stop_operator.schema.json") }}

## Assumptions / Known limits

TBD.
