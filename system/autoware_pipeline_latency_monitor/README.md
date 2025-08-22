# pipeline_latency_monitor

## Purpose

This package provides a node for calculating the latency introduced by a sequence of Autoware processing steps. For example, the pipeline from receiving sensor data up to the generation of the control command is made of several steps (e.g., perception, prediction, planning, control). This node calculates the total end-to-end latency by subscribing to messages that report the processing time of each step, combining them based on their timestamps, and publishing the total latency.

## Inner-workings / Algorithms

The node is configured with a sequence of processing steps. For each step, it subscribes to a topic that publishes its execution latency.

To calculate the total latency, the node works backward from the last step in the defined sequence. It takes the most recent latency measurement from the final step. Then, for each preceding step, it finds a corresponding latency measurement from its history whose timestamp is consistent with the subsequent step. Specifically, it ensures that the end time of a given step occurs before the start time of the next step in the chain.

The total latency is the sum of these chronologically-consistent individual latencies. The node also allows for adding fixed offset values to the final sum.

The calculated total latency is published continuously and also used to report diagnostic information. A WARN status is published if the latency exceeds a configurable threshold.

## Inputs / Outputs

### Input

Subscribes to topics containing latency information for each processing step. The topic names and types are configured via parameters.
Only 2 message types are currently supported.

| Message Type                                              | Timestamp field | Latency value field |
| --------------------------------------------------------- | --------------- | ------------------- |
| `autoware_internal_debug_msgs/msg/Float64Stamped`         | `stamp`         | `data`              |
| `autoware_planning_validator/msg/PlanningValidatorStatus` | `stamp`         | `latency`           |

### Output

| Name                                | Type                                              | Description                                                                                                         |
| ----------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `~/output/total_latency_ms`         | `autoware_internal_debug_msgs/msg/Float64Stamped` | The calculated total pipeline latency in milliseconds.                                                              |
| `/diagnostics`                      | `diagnostic_msgs/DiagnosticArray`                 | Publishes the diagnostic status. Reports `OK` if the latency is within the threshold, and `WARN` if it is exceeded. |
| `~/debug/<step_name>_latency_ms`    | `autoware_internal_debug_msgs/msg/Float64Stamped` | For each processing step, publishes the latest latency value received from its input topic.                         |
| `~/debug/pipeline_total_latency_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | A debug topic that also publishes the calculated total latency.                                                     |

## Parameters

| Name                                              | Type       | Default Value | Description                                                                                                                                                          |
| ------------------------------------------------- | ---------- | ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `update_rate`                                     | `double`   | 10            | The rate [Hz] at which the total latency is calculated and published.                                                                                                |
| `latency_threshold_ms`                            | `double`   | 1000          | The latency threshold in milliseconds. If the total latency exceeds this value, a `WARN` diagnostic is reported.                                                     |
| `window_size`                                     | `int`      | 10            | The number of historical latency messages to store for each processing step.                                                                                         |
| `processing_steps.sequence`                       | `string[]` | `[]`          | An array of names defining the ordered sequence of processing steps to measure.                                                                                      |
| `processing_steps.<step_name>.topic`              | `string`   | -             | The input topic that provides the latency for this step.                                                                                                             |
| `processing_steps.<step_name>.topic_type`         | `string`   | -             | The message type of the input topic. Supported types: `autoware_internal_debug_msgs/msg/Float64Stamped`, `autoware_planning_validator/msg/PlanningValidatorStatus`.  |
| `processing_steps.<step_name>.timestamp_meaning`  | `string`   | `"end"`       | Defines if the timestamp in the message represents the `start` or `end` of the processing interval for that step.                                                    |
| `processing_steps.<step_name>.latency_multiplier` | `double`   | `1.0`         | A multiplier to convert the received latency value into milliseconds (e.g., use 1000.0 if the input is in seconds).                                                  |
| `latency_offsets_ms`                              | `double[]` | `[]`          | A list of fixed latency values (in milliseconds) to be added to the calculated total latency. Useful for accounting for steps that do not publish their own latency. |

## Assumptions / Known limits

- The accuracy of the latency calculation depends on the timestamps of all input messages being synchronized to a common clock.
- The algorithm for combining latencies assumes a sequential, non-parallel processing pipeline.
- The node must be configured with the correct sequence of processing steps and their corresponding topics for the calculation to be meaningful.
