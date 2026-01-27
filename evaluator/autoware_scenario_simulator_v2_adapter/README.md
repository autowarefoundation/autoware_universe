# scenario_simulator_v2 Adapter

## Purpose

This package provides a node to convert various messages from the Autoware into `tier4_simulation_msgs::msg::UserDefinedValue` messages for the scenario_simulator_v2.
Currently, this node supports conversion of:

- `tier4_metric_msgs::msg::MetricArray` for metric topics
- Diagnostic topics passed from `.webauto-ci.yml` via `autoware.diagnostic_config`

## Inner-workings / Algorithms

- For `tier4_metric_msgs::msg::MetricArray`,
  The node subscribes to all topics listed in the parameter `metric_topic_list`.
  Each time such message is received, it is converted into as many `UserDefinedValue` messages as the number of `Metric` objects.
  The format of the output topic is detailed in the _output_ section.

- For diagnostic topics from `autoware.diagnostic_config`,
  The node subscribes to `/diagnostics`
  Each time such message is received, it is converted into as many `UserDefinedValue` messages as the number of `DiagnosticStatus` objects in the `DiagnosticArray`.
  The format of the input `autoware.diagnostic_config` is detailed in the _input_ section.
  The format of the output topic is detailed in the _output_ section.

## Metric Array Inputs / Outputs

### Metric Array Inputs

The node listens to `MetricArray` messages on the topics specified in `metric_topic_list`.

### Metric Array Outputs

The node outputs `UserDefinedValue` messages that are converted from the received messages.

The name of the output topics are generated from the corresponding input topic, the name of the metric.

- For example, we might listen to topic `/planning/planning_evaluator/metrics` and receive a `MetricArray` with 2 metrics:
  - metric with `name: "metricA/x"`
  - metric with `name: "metricA/y"`
- The resulting topics to publish the `UserDefinedValue` are as follows:
  - `/planning/planning_evaluator/metrics/metricA/x`
  - `/planning/planning_evaluator/metrics/metricA/y`

## Diagnostics Inputs / Outputs

Diagnostics extracts the `level` field from `DiagnosticStatus` and outputs it as a single value to correspond with `tier4_simulation_msgs::msg::UserDefinedValue` messages.

### Diagnostics Inputs

- The node listens to `/diagnostics`.
  Multiple `DiagnosticStatus` objects are extracted from the `DiagnosticArray` within `/diagnostics`.

- About `autoware.diagnostic_config`:
  - The configuration file is specified from `.webauto-ci.yml`.
  - Under `diagnostic_groups`, create a group name and define `output_topic_name` and `aggregation_list` under it.
    - `output_topic_name`
      - Specifies the topic name to publish to.
      - The format `/diagnostics/scenario_simulator_v2_adapter/***` is recommended.
    - `aggregation_list`
      - Lists the diagnostic topics to be grouped.
      - Can include `output_topic_name` declared in the same format (recursively expanded).
  - A sample configuration can be found in `autoware_scenario_simulator_v2_adapter/config/diagnostic_config.param.yaml`.

```yml
/**:
  ros__parameters:
    diagnostic_groups:
      overall_diagnostics:
        output_topic_name: /diagnostics/scenario_simulator_v2_adapter/overall_diagnostics
        aggregation_list:
          - /diagnostics/scenario_simulator_v2_adapter/topic_state_monitor_diagnostics

      topic_state_monitor_diagnostics:
        output_topic_name: /diagnostics/scenario_simulator_v2_adapter/topic_state_monitor_diagnostics
        aggregation_list:
          - /diagnostics/topic_state_monitor_mission_planning_route/planning_topic_status
          - /diagnostics/topic_state_monitor_scenario_planning_trajectory/planning_topic_status
```

### Diagnostics Outputs

The node outputs `UserDefinedValue` messages that are converted from the received messages.
Diagnostics output has two patterns:

1. One-to-one correspondence with diagnostic status names.
2. Grouped output based on `autoware.diagnostic_config` that aggregates diagnostic status names.

- For individual publish:
  The name of the output topics are generated from `/diagnostics/` prefix and the `status.name` field of each `DiagnosticStatus`.
  The `": "` in the status name is replaced with `/` to form a valid topic name.
  - status name: `planning_validator: intersection_validation_collision_check`
  - publish topic name: `/diagnostics/planning_validator/intersection_validation_collision_check`

- For grouped topic publish:
  The name of the output topics is used `output_topic_name` in `autoware.diagnostic_config`.
  Each time a `DiagnosticStatus` that matches any topic in the `aggregation_list` is received, its level value is published to the `output_topic_name` topic.
  If the level is `ERROR`, a warning is logged with the diagnostic topic name and group name.

## Parameters

{{ json_to_markdown("evaluator/autoware_scenario_simulator_v2_adapter/schema/scenario_simulator_v2_adapter.schema.json") }}

## Assumptions / Known limits

Values in the `Metric` objects of a `MetricArray` are assumed to be of type `double`.

## Future extensions / Unimplemented parts
