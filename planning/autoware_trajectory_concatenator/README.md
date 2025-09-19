# Trajectory Concatenator

## Purpose/Role

This node aggregates trajectory candidates from multiple trajectory generators into a single `autoware_internal_planning_msgs/msg/CandidateTrajectories` message. It is intended to be placed between trajectory generators and the selector/ranker.

## Algorithm Overview

When a message arrives on `~/input/trajectories`, the node splits it by `generator_id` and updates an in-memory buffer so that only the most recent trajectory set for each generator is retained.

A 100 ms timer then scans this buffer and drops any entry whose header stamp is older than the configured duration_time. Immediately after pruning, the timer concatenates all remaining trajectories and their accompanying `generator_info` arrays, publishes the aggregated message.

## Interface

### Topics

| Direction  | Topic name              | Message Type                                                | Description                                    |
| ---------- | ----------------------- | ----------------------------------------------------------- | ---------------------------------------------- |
| Subscriber | `~/input/trajectories`  | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectory sets produced by each generator     |
| Publisher  | `~/output/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Concatenated list of all buffered trajectories |

### Parameters

{{ json_to_markdown("planning/autoware_trajectory_concatenator/schema/trajectory_concatenator.schema.json") }}
