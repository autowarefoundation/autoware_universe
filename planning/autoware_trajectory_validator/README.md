# Trajectory Validator

## Purpose/Role

This package provides a pluginlib-based C++ library for evaluating candidate trajectories against a configurable set of safety and traffic-rule filters. Each filter plugin receives a world-state context snapshot (`ValidatorContext`) and returns a feasibility verdict for a single trajectory. Trajectories rejected by any _enforced_ plugin are removed from the output set. The library is embedded in `autoware_trajectory_selector`.

## Algorithm Overview

For each input `CandidateTrajectories` message the library runs every loaded plugin against every trajectory:

1. **Plugin evaluation**: each plugin's `is_feasible()` is called with the trajectory points and the current `ValidatorContext` (odometry, predicted objects, acceleration, HD map, traffic light states).
2. **Feasibility decision**: a trajectory survives if every plugin listed in `filter_names` returns `is_feasible = true`. Plugins listed in `shadow_mode_filter_names` are evaluated and reported but never remove a trajectory.
3. **Diagnostics**: the wrapper publishes an Autoware diagnostics status — `OK` if all trajectories pass all enforced plugins, `WARN` if at least one is rejected, `ERROR` if none survive.

## Built-in Plugins

| Class name                                   | Category       | Description                                                                                                   |
| -------------------------------------------- | -------------- | ------------------------------------------------------------------------------------------------------------- |
| `safety::VehicleConstraintFilter`            | `safety`       | Rejects trajectories that exceed maximum speed, acceleration, deceleration, steering angle, or steering rate. |
| `safety::UncrossableBoundaryDepartureFilter` | `safety`       | Rejects trajectories whose footprint crosses an uncrossable map boundary (e.g., road borders).                |
| `traffic_rule::TrafficLightFilter`           | `traffic_rule` | Rejects trajectories that cross a red or amber stop line when the vehicle could safely stop.                  |

## Interface

This package is a C++ library. The topics below are subscribed to by the hosting node (`trajectory_selector_node`) and passed to the validator as a `ValidatorContext`.

### Topics

| Direction  | Topic name                                 | Message Type                                              | Description                                                            |
| ---------- | ------------------------------------------ | --------------------------------------------------------- | ---------------------------------------------------------------------- |
| Subscriber | `~/input/odometry`                         | `nav_msgs/msg/Odometry`                                   | Current ego pose and velocity (mandatory)                              |
| Subscriber | `~/input/acceleration`                     | `geometry_msgs/msg/AccelWithCovarianceStamped`            | Current ego acceleration (mandatory)                                   |
| Subscriber | `~/input/objects`                          | `autoware_perception_msgs/msg/PredictedObjects`           | Surrounding dynamic obstacles (mandatory)                              |
| Subscriber | `~/input/traffic_signals`                  | `autoware_perception_msgs/msg/TrafficLightGroupArray`     | Traffic light states (optional; absent data does not block validation) |
| Subscriber | `~/input/lanelet2_map`                     | `autoware_map_msgs/msg/LaneletMapBin`                     | HD map loaded once at startup (transient local QoS; mandatory)         |
| Publisher  | `~/debug/validation_reports`               | `autoware_trajectory_validator/msg/ValidationReportArray` | Per-trajectory validation verdict and per-metric values                |
| Publisher  | `~/debug/markers/<plugin_name>`            | `visualization_msgs/msg/MarkerArray`                      | Per-plugin debug visualization markers                                 |
| Publisher  | `~/debug/plugin_report_text`               | `visualization_msgs/msg/MarkerArray`                      | Text overlay summarizing how many paths each plugin filtered           |
| Publisher  | `~/debug/processing_time_ms`               | `autoware_internal_debug_msgs/msg/Float64Stamped`         | Total validator processing time [ms]                                   |
| Publisher  | `~/debug/<plugin_name>/processing_time_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped`         | Per-plugin processing time [ms]                                        |
| Publisher  | `~/debug/processing_time_text`             | `autoware_internal_debug_msgs/msg/StringStamped`          | Human-readable processing time breakdown                               |

### Parameters

{{ json_to_markdown("planning/autoware_trajectory_validator/schema/trajectory_validator.schema.json") }}
