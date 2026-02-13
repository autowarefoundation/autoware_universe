# Autoware Trajectory Traffic Rule Filter

## Purpose

The `autoware_trajectory_traffic_rule_filter` package provides a plugin-based filtering system for candidate trajectories based on traffic rules. It evaluates trajectories against various traffic regulations and safety constraints to ensure compliance with traffic laws.

## Inner-workings / Algorithms

### Architecture

The package uses a plugin architecture that allows for flexible and extensible traffic rule checking:

1. **Main Node**: `TrajectoryTrafficRuleFilter` - Manages plugins and coordinates filtering
2. **Plugin Interface**: `TrafficRuleFilterInterface` - Base class for all filter plugins
3. **Filter Plugins**: Individual filters that implement specific traffic rule checks

### Filter Plugins

#### TrafficLightFilter

- Validates trajectory compliance with traffic signals
- Monitors traffic light states from perception system
- Reject trajectories when:
  - they cross a red traffic light;
  - they cross a amber traffic light and it is determined that either 1) ego has enough time to comfortably stop or 2) ego has enough time to cross before the light turns red (see [autoware_behavior_velocity_traffic_light_module](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module/#dilemma-zone) for some similar logic).

## Interface

### Topics

| Direction | Topic Name                        | Type                                                          | Description                                          |
| --------- | --------------------------------- | ------------------------------------------------------------- | ---------------------------------------------------- |
| Input     | `~/input/candidate_trajectories`  | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | Candidate trajectories to be filtered                |
| Input     | `~/input/lanelet2_map`            | `autoware_map_msgs::msg::LaneletMapBin`                       | Lanelet2 map containing traffic rule info            |
| Input     | `~/input/traffic_signals`         | `autoware_perception_msgs::msg::TrafficLightGroupArray`       | Current traffic light states                         |
| Output    | `~/output/candidate_trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | Filtered trajectories that comply with traffic rules |
| Output    | `/diagnostics`                    | `diagnostic_msgs::msg::DiagnosticArray`                       | Diagnostic status of the filtering process           |

### Diagnostics

The node publishes diagnostic information to monitor the status of the trajectory filtering process.

| Level   | Message                                             | Condition                                                                                                |
| ------- | --------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `OK`    | ""                                                  | At least one trajectory is feasible, including at least one from a diffusion-based planner if present.   |
| `WARN`  | "All diffusion planner trajectories are infeasible" | Feasible trajectories exist, but all trajectories from generators named "Diffusion\*" were filtered out. |
| `ERROR` | "No feasible trajectories found"                    | All input candidate trajectories were filtered out by the active plugins.                                |

### Parameters

| Name           | Type         | Description                    | Default Value   |
| -------------- | ------------ | ------------------------------ | --------------- |
| `filter_names` | string array | List of filter plugins to load | See config file |

#### Plugin Configuration

The active filters are specified in `config/trajectory_traffic_rule_filter.param.yaml`:

```yaml
/**:
  ros__parameters:
    filter_names:
      - "autoware::trajectory_traffic_rule_filter::plugin::TrafficLightFilter"
```

#### Traffic Light Filter parameters

| Name                                             | Type   | Description                                                                                                | Default Value |
| ------------------------------------------------ | ------ | ---------------------------------------------------------------------------------------------------------- | ------------- |
| `traffic_light_filter.enable_pass_judge`         | bool   | Enable pass judge logic. When false, trajectories that cross a amber light are always rejected             | true          |
| `traffic_light_filter.max_accel`                 | double | Maximum deceleration [m/s^2] used for pass judge.                                                          | -2.8          |
| `traffic_light_filter.max_jerk`                  | double | Maximum jerk [m/s^3] used for pass judge.                                                                  | -5.0          |
| `traffic_light_filter.delay_response_time`       | double | Delay response time [s] used for pass judge.                                                               | 0.5           |
| `traffic_light_filter.amber_lamp_period`         | double | Amber lamp period [s] used to estimate if ego has time to cross before the light turns red.                | 2.75          |
| `traffic_light_filter.amber_light_stop_velocity` | double | Current ego velocity threshold [m/s] below which trajectories crossing an amber light are always rejected. | 1.0           |
