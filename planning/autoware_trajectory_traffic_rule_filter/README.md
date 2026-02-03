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
- Checks if trajectory would pass through red lights
- Allows trajectories only when traffic lights permit passage

## Interface

### Topics

| Direction | Topic Name                        | Type                                                          | Description                                          |
| --------- | --------------------------------- | ------------------------------------------------------------- | ---------------------------------------------------- |
| Input     | `~/input/candidate_trajectories`  | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | Candidate trajectories to be filtered                |
| Input     | `~/input/lanelet2_map`            | `autoware_map_msgs::msg::LaneletMapBin`                       | Lanelet2 map containing traffic rule info            |
| Input     | `~/input/traffic_signals`         | `autoware_perception_msgs::msg::TrafficLightGroupArray`       | Current traffic light states                         |
| Output    | `~/output/candidate_trajectories` | `autoware_internal_planning_msgs::msg::CandidateTrajectories` | Filtered trajectories that comply with traffic rules |

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
