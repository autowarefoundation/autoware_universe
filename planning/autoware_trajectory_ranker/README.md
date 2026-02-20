# autoware_trajectory_ranker

## Purpose

The `autoware_trajectory_ranker` package provides a flexible and extensible framework for evaluating and ranking multiple trajectory candidates in autonomous driving scenarios. It transforms a set of viable candidate paths into an ordered list by attaching a scalar score to every trajectory based on safety, comfort, and efficiency criteria.

This decoupling of upstream path-generation logic from downstream motion-control logic enables:

- Diverse trajectory generators (rule-based, ML-based, optimization-based) to coexist
- Controllers to simply follow the highest-ranked path
- Improved overall driving performance through multi-criteria optimization
- Architectural flexibility through a plugin-based metric system

## Features

- **Plugin-based Architecture**: Dynamically load and configure evaluation metrics via pluginlib
- **Multi-criteria Evaluation**: Score trajectories based on multiple metrics simultaneously
- **Time-based Weighting**: Apply temporal decay to metrics for better near-term decision making
- **Extensible Framework**: Easy to add new metrics through the plugin system
- **Real-time Processing**: Efficient evaluation of multiple trajectories per planning cycle

## Algorithm Overview

At every planning period the node latches the most recent odometry and perception context, then processes trajectories through five distinct stages:

1. **Ingestion**: Receives candidate trajectories from upstream generators
2. **Resampling**: Converts each path into a common frame and resamples to fixed resolution
3. **Evaluation**: Applies configured metric plugins to compute cost vectors
4. **Aggregation**: Combines metrics using weights and temporal decay
5. **Publication**: Outputs scored trajectories and debug information

### Trajectory Resampling

Every incoming trajectory is re-interpolated relative to the ego pose so that:

- First point aligns with current vehicle position
- Path comprises exactly `sample_num` points
- Points are spaced at fixed temporal intervals
- All metrics operate on consistent trajectory representations

This ensures that metric plugins can assume identical temporal spacing even when generators produce paths of different lengths or densities.

### Metric Evaluation

For each resampled path, the evaluator executes a plugin chain specified in the configuration. Each metric plugin:

- Computes a vector of values (one per trajectory point)
- Returns costs reflecting safety, comfort, or efficiency considerations
- Can be classified as either maximization or deviation metrics

### Score Aggregation

The aggregation stage:

1. Applies per-metric weights from configuration
2. Multiplies by temporal decay weights (exponential decay for near-term bias)
3. Normalizes across trajectory candidates
4. Sums to produce a single scalar score per trajectory

## Available Metrics

The package includes the following built-in metric plugins:

| Metric                  | Type         | Description                                | Class Name                                                  |
| ----------------------- | ------------ | ------------------------------------------ | ----------------------------------------------------------- |
| **TravelDistance**      | Maximization | Measures progress along trajectory         | `autoware::trajectory_ranker::metrics::TravelDistance`      |
| **LateralAcceleration** | Deviation    | Evaluates lateral acceleration for comfort | `autoware::trajectory_ranker::metrics::LateralAcceleration` |
| **LongitudinalJerk**    | Deviation    | Measures longitudinal jerk for smoothness  | `autoware::trajectory_ranker::metrics::LongitudinalJerk`    |
| **TimeToCollision**     | Maximization | Calculates time to collision with objects  | `autoware::trajectory_ranker::metrics::TimeToCollision`     |
| **LateralDeviation**    | Deviation    | Measures deviation from preferred lanes    | `autoware::trajectory_ranker::metrics::LateralDeviation`    |
| **SteeringConsistency** | Deviation    | Evaluates steering command consistency     | `autoware::trajectory_ranker::metrics::SteeringConsistency` |

### Metric Types

- **Maximization**: Higher values are better (e.g., distance traveled, time to collision)
- **Deviation**: Lower values are better (e.g., lateral acceleration, jerk)

## Input / Output

### Input Topics

| Topic                  | Type                                                        | Description                              |
| ---------------------- | ----------------------------------------------------------- | ---------------------------------------- |
| `~/input/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Candidate trajectories to evaluate       |
| `~/input/objects`      | `autoware_perception_msgs/msg/PredictedObjects`             | Predicted objects for collision checking |
| `~/input/odometry`     | `nav_msgs/msg/Odometry`                                     | Current vehicle state                    |
| `~/input/map`          | `autoware_map_msgs/msg/LaneletMapBin`                       | Lanelet2 HD map                          |
| `~/input/route`        | `autoware_planning_msgs/msg/LaneletRoute`                   | Current route                            |

### Output Topics

| Topic                            | Type                                                              | Description                   |
| -------------------------------- | ----------------------------------------------------------------- | ----------------------------- |
| `~/output/trajectories`          | `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories` | Scored candidate trajectories |
| `~/debug/processing_time_detail` | `autoware_utils_debug/msg/ProcessingTimeDetail`                   | Processing time statistics    |

## Parameters

### Core Parameters

The package uses the Generate Parameter Library for configuration. Parameters are defined in `param/trajectory_ranker_parameters.yaml`:

### Parameter Structure

- **evaluation.sample_num**: Number of points in resampled trajectories (default: 16)
- **evaluation.resolution**: Time interval between trajectory points in seconds (default: 0.5)
- **evaluation.metrics.name**: Array of metric plugin class names (fully qualified)
- **evaluation.metrics.maximum**: Maximum expected values for each metric (used for normalization)
- **evaluation.score_weight**: Weight for each metric in final score calculation
- **evaluation.time_decay_weight.s0-s5**: Temporal weights for metrics (s0=first metric, s1=second, etc.)

## Simple Trajectory Ranker

The `SimpleTrajectoryRanker` is a lightweight alternative to the main trajectory ranker. It ranks trajectory candidates based on a prioritized list of generator name prefixes rather than complex geometric or physical metrics.

### Logic

The node matches the generator name of each candidate trajectory (retrieved from `generator_info` using the `generator_id`) against a user-defined list of prefixes in `ranked_generator_name_prefixes`.

### Scoring

Scores are assigned based on the inverse of the rank of the matching prefix:

- If there are $N$ ranked prefixes, a trajectory matching the first prefix gets a score of $N$.
- A trajectory matching the second prefix gets a score of $N-1$, and so on.
- Trajectories that do not match any prefix, or whose generator info is missing, are assigned a score of `0.0`.

The output list is ordered by these scores (highest score first), followed by unranked trajectories which maintain their original relative order.

### Input / Output

| Topic                            | Type                                                              | Description                   |
| -------------------------------- | ----------------------------------------------------------------- | ----------------------------- |
| `~/input/candidate_trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories`       | Candidate trajectories        |
| `~/output/scored_trajectories`   | `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories` | Scored candidate trajectories |

### Parameters

| Parameter                        | Type                       | Description                                              |
| -------------------------------- | -------------------------- | -------------------------------------------------------- |
| `ranked_generator_name_prefixes` | `std::vector<std::string>` | Ordered list of generator name prefixes for prioritizing |

## Usage

### Basic Launch

```bash
ros2 launch autoware_trajectory_ranker trajectory_ranker.launch.xml
```

### Launch with Custom Parameters

```bash
ros2 launch autoware_trajectory_ranker trajectory_ranker.launch.xml \
    trajectory_ranker_param_file:=/path/to/your/config.yaml
```
