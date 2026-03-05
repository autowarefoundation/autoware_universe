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

- Validates trajectory compliance with traffic signals.
- Monitors traffic light states from the perception system.
- **Limitation**: Currently, only circle light signals (RED, AMBER) are handled; arrow signals are ignored by this filter.
- Reject trajectories when:
  - the vehicle's footprint (front-end extension) crosses a red traffic light stop line.
  - the vehicle's footprint crosses an amber traffic light stop line and it is determined that either:
    1. ego can safely stop at the stop line without breaking the deceleration/jerk limits, considering a `delay_response_time`.
    2. ego's `base_link` point cannot cross the stop line within the `crossing_time_limit`.

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

| Name                                                  | Type   | Description                                                                                                                                                        | Default Value |
| ----------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------- |
| `traffic_light_filter.deceleration_limit`             | double | Trajectories crossing an amber light are rejected if ego can stop at the stop line without breaking this limit                                                     | 2.8           |
| `traffic_light_filter.jerk_limit`                     | double | Trajectories crossing an amber light are rejected if ego can stop at the stop line without breaking this limit                                                     | 5.0           |
| `traffic_light_filter.delay_response_time`            | double | Delay response time (reaction time) used to estimate the minimum ego stopping distance from its current state                                                      | 0.5           |
| `traffic_light_filter.crossing_time_limit`            | double | Trajectories crossing an amber light are rejected if they cannot cross (reach the stop line with `base_link`) before this time                                     | 2.75          |
| `traffic_light_filter.treat_amber_light_as_red_light` | bool   | When true, amber lights are handled like red lights (using vehicle footprint crossing). If false, amber lights use the `base_link` crossing logic described above. | true          |
