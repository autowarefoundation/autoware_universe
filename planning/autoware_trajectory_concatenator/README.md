# Trajectory concatenator

## Purpose/Role

This node aggregates trajectory candidates from multiple trajectory generators into a single [autoware_internal_planning_msgs/msg/Trajectories](../autoware_internal_planning_msgs/msg/Trajectories.msg) message. It is intended to be placed between trajectory generators and the selector/ranker in the new planning framework stack.

## Algorithm Overview

When a message arrives on `~/input/trajectories`, the node splits it by `generator_id` and updates an in‑memory buffer so that only the most recent trajectory set for each generator is retained.

A 100 ms timer then scans this buffer and drops any entry whose header stamp is older than the configured duration_time. Immediately after pruning, the timer concatenates all remaining trajectories and their accompanying `generator_info` arrays, publishes the aggregated message.

## Interface

### Topics

| Direction  | Topic name                    | Message Type                                       | Description                                                |
| ---------- | ----------------------------- | -------------------------------------------------- | ---------------------------------------------------------- |
| Subscriber | `~/input/trajectories`        | `autoware_internal_planning_msgs/msg/Trajectories` | Trajectory sets produced by each generator                 |
| Subscriber | `~/input/selected_trajectory` | `autoware_planning_msgs/msg/Trajectory`            | Current selector output (optional feedback)                |
| Subscriber | `~/input/odometry`            | `nav_msgs/msg/Odometry`                            | Ego pose needed to trim and extend the feedback trajectory |
| Publisher  | `~/output/trajectories`       | `autoware_internal_planning_msgs/msg/Trajectories` | Concatenated list of all buffered trajectories             |

### Parameters

| Name                                     | Type   | Default | Description                                                           |
| ---------------------------------------- | ------ | ------- | --------------------------------------------------------------------- |
| `duration_time`                          | double | 0.2     | Maximum age of trajectories kept in the buffer (seconds)              |
| `selected_trajectory.use`                | bool   | false   | Enable feedback mode                                                  |
| `selected_trajectory.endpoint_time_min`  | double | 10.0    | Minimum horizon length required for the feedback trajectory (seconds) |
| `selected_trajectory.extension_interval` | double | 0.1     | Δt used when extending the feedback trajectory (seconds)              |
