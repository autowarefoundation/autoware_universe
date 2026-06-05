# Trajectory Selector

## Purpose/Role

`trajectory_selector_node` is the ROS 2 node that orchestrates the full candidate trajectory pipeline. It buffers trajectory candidates from multiple upstream planners via `autoware_trajectory_concatenator`, validates each candidate against the safety and traffic-rule plugins of `autoware_trajectory_validator`, and publishes the surviving trajectories for downstream processing.

## Algorithm Overview

1. Candidate trajectories arrive on two separate input topics — `~/input/trajectories_generative` for generative planners and `~/input/trajectories_backup` for backup planners. Each received message is forwarded to the internal concatenator buffer.
2. A 100 ms timer fires: the concatenator prunes stale entries (older than `duration_time`) and returns the merged trajectory set from all registered generators.
3. If any mandatory sensor input (odometry, predicted objects, acceleration, or HD map) is unavailable, the timer returns early without publishing.
4. The validator runs all loaded plugins against each trajectory. Trajectories rejected by any enforced plugin (listed in `filter_names`) are removed from the output set.
5. The surviving trajectories are published to `~/output/trajectories`.

## Interface

### Topics

| Direction  | Topic name                                              | Message Type                                                | Description                                                            |
| ---------- | ------------------------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------- |
| Subscriber | `~/input/trajectories_generative`                       | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from generative planners                                  |
| Subscriber | `~/input/trajectories_backup`                           | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from backup planners                                      |
| Subscriber | `~/input/lanelet2_map`                                  | `autoware_map_msgs/msg/LaneletMapBin`                       | HD map (transient local QoS; loaded once at startup)                   |
| Subscriber | `~/input/odometry`                                      | `nav_msgs/msg/Odometry`                                     | Current ego pose and velocity                                          |
| Subscriber | `~/input/objects`                                       | `autoware_perception_msgs/msg/PredictedObjects`             | Surrounding dynamic obstacles                                          |
| Subscriber | `~/input/acceleration`                                  | `geometry_msgs/msg/AccelWithCovarianceStamped`              | Current ego acceleration                                               |
| Subscriber | `~/input/traffic_signals`                               | `autoware_perception_msgs/msg/TrafficLightGroupArray`       | Traffic light states (optional; missing data does not block the timer) |
| Publisher  | `~/output/trajectories`                                 | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories that passed all enforced validator plugins                |
| Publisher  | `~/debug/processing_time_detail_ms/trajectory_selector` | `autoware_internal_debug_msgs/msg/ProcessingTimeTree`       | Per-function processing time breakdown                                 |

### Parameters

This node does not declare parameters of its own. Configuration is provided through the sub-package parameter files loaded at launch:

- Concatenation parameters: see [`autoware_trajectory_concatenator`](../autoware_trajectory_concatenator/README.md#parameters)
- Validation parameters: see [`autoware_trajectory_validator`](../autoware_trajectory_validator/README.md#parameters)

{{ json_to_markdown("planning/autoware_trajectory_selector/schema/trajectory_selector.schema.json") }}
