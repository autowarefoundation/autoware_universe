# Planning Evaluator

## Purpose

This package provides nodes that generate various metrics to evaluate the quality of planning.

Metrics can be published in real time and saved to a JSON file when the node is shut down:

- `metrics_for_publish`:
  - Metrics listed in `metrics_for_publish` are calculated and published to the topic.

- `metrics_for_output`:
  - Metrics listed in `metrics_for_output` are saved to a JSON file when the node shuts down (if `output_metrics` is set to `true`).
  - These metrics include statistics derived from `metrics_for_publish` and additional information such as parameters and descriptions.

## Metrics

All possible metrics are defined in the `Metric` enumeration located in:

- `include/autoware/planning_evaluator/metrics/metric.hpp`
- `include/autoware/planning_evaluator/metrics/output_metric.hpp`

These files also provide string conversions and human-readable descriptions for use in output files.

### Metric Classifications

#### By Data Type

1. **Statistics-based Metrics**:
   - Calculated using `autoware_utils::Accumulator`, which tracks minimum, maximum, mean, and count values.
   - Sub-metrics: `/mean`, `/min`, `/max`, and `/count`.

2. **Value-based Metrics**:
   - Metrics with a single value.
   - Sub-metrics: `/value`.
   - Some metrics with older implementations use the statistics-based format of `/mean`, `/min`, `/max`, but all values are the same.

3. **Count-based Metrics**:
   - Count occurrences of specific events over a time period.
   - Sub-metrics: `/count`, or `/count_in_duration`.

#### By Purpose

1. [Trajectory Metrics](#trajectory-metrics)
2. [Trajectory Deviation Metrics](#trajectory-deviation-metrics)
3. [Trajectory Stability Metrics](#trajectory-stability-metrics)
4. [Trajectory Obstacle Metrics](#trajectory-obstacle-metrics)
5. [Modified Goal Metrics](#modified-goal-metrics)
6. [Planning Factor Metrics](#planning-factor-metrics)
7. [Steering Metrics](#steering-metrics)
8. [Blinker Metrics](#blinker-metrics)
9. [Other Information](#other-information)

## Detailed Metrics

### Trajectory Metrics

Evaluates the current trajectory `T(0)` itself.
Metrics are calculated and published when a trajectory is received.

#### Implemented metrics

- **`curvature`**: Statistics of curvature at each trajectory point.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`point_interval`**: Statistics of distances between consecutive trajectory points.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`relative_angle`**: Statistics of angles between consecutive trajectory points.
  - Parameters: `trajectory.min_point_dist_m` (minimum distance between points).
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`resampled_relative_angle`**: Similar to `relative_angle`, but considers a point at a fixed distance (e.g., half the vehicle length) for angle calculation from the current point as the next point to calculate the relative angle, instead of using the immediately adjacent point.

- **`length`**: Total trajectory length.
  - Sub-metrics: Value-based metric, but using the statistics-based format of `/mean`, `/min`, `/max` with the same value.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

- **`duration`**: Expected driving time to travel the trajectory.
  - Sub-metrics: Value-based metric, but using the statistics-based format of `/mean`, `/min`, `/max` with the same value.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

- **`velocity`**: Statistics of velocity at each trajectory point.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`acceleration`**: Statistics of acceleration at each trajectory point.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`jerk`**: Statistics of jerk at each trajectory point.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

### Trajectory Deviation Metrics

Evaluates the trajectory deviation by comparing the trajectory `T(0)` and the reference trajectory.

Metrics are calculated and publish only when the node receives a trajectory.

The following information are used to calculate metrics:

- the trajectory `T(0)`.
- the _reference_ trajectory assumed to be used as the reference to plan `T(0)`.

#### Implemented metrics

- **`lateral_deviation`**: Statistics of the lateral deviation between trajectory points and the closest reference trajectory points.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`yaw_deviation`**: Statistics of the yaw deviation between trajectory points and the closest reference trajectory points.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`velocity_deviation`**: Statistics of the velocity deviation between trajectory points and the closest reference trajectory points.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

### Trajectory Stability Metrics

Evaluates the trajectory stability by comparing the trajectory `T(0)` and previous trajectory `T(-1)`.

Metrics are calculated and publish only when the node receives a trajectory.

The following information are used to calculate metrics:

- the trajectory `T(0)` itself.
- the previous trajectory `T(-1)`.
- the current ego odometry.

#### Implemented metrics

**`stability`**: Statistics of the lateral deviation between `T(0)` and `T(-1)` within a lookahead duration and distance.

- Parameters: `trajectory.lookahead.max_time_s`, `trajectory.lookahead.max_dist_m`.
- Sub-metrics to publish: `/mean`, `/min`, `/max`.
- Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`stability_frechet`**: Frechet distance between `T(0)` and `T(-1)` within a lookahead duration and distance.
  - Parameters: Same as `stability`.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`lateral_trajectory_displacement_local`**: Absolute lateral displacement between `T(0)` and `T(-1)` at the ego position.
  - Sub-metrics to publish: Value-based metric, but using the statistics-based format.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

- **`lateral_trajectory_displacement_lookahead`**: Statistics of absolute lateral displacement between `T(0)` and `T(-1)` within a lookahead duration.
- Parameters: `trajectory.evaluation_time_s`.
- Sub-metrics to publish: `/mean`, `/min`, `/max`.
- Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

### Trajectory Obstacle Metrics

Evaluate the safety of `T(0)` with respect to obstacles.

Metrics are calculated and publish only when the node receives a trajectory.

The following information are used to calculate metrics:

- the trajectory `T(0)`.
- the set of objects in the environment.

#### Implemented metrics

- **`obstacle_distance`**: Statistics of the distance between the centroid of each object and the closest trajectory point.
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

- **`obstacle_ttc`**: Statistics of the time-to-collision (TTC) for those objects near the trajectory.
  - Parameters: `obstacle.dist_thr_m` (distance threshold to consider the object as close to the trajectory).
  - Sub-metrics to publish: `/mean`, `/min`, `/max`.
  - Sub-metrics to output: The same as above but take the published data as data point instead of each trajectory point.

### Modified Goal Metrics

Evaluate deviations between the modified goal and the ego position.

Metrics are calculated and publish only when the node receives a modified goal message.

#### Implemented metrics

- **`modified_goal_longitudinal_deviation`**: Statistics of the longitudinal deviation between the modified goal and the current ego position.
  - Sub-metrics to publish: Value-based metric, but using the statics-based format.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

- **`modified_goal_lateral_deviation`**: Statistics of the lateral deviation between the modified goal and the current ego position.
  - Sub-metrics to publish: Value-based metric, but using the statics-based format.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

- **`modified_goal_yaw_deviation`**: Statistics of the yaw deviation between the modified goal and the current ego position.
  - Sub-metrics to publish: Value-based metric, but using the statics-based format.
  - Sub-metrics to output: `/mean`, `/min`, `/max` for the published data.

### Planning Factor Metrics

Evaluates the behavior of each planning module by checking planning factors.

Metrics are calculated and publish only when the node receives that planning factors published by each module.

The modules listed in the `module_list` in the parameter file are evaluated.

#### Implemented metrics

- **`stop_decision`**: Evaluate stop decisions for each module.
  - Parameters:
    - `stop_decision.time_count_threshold_s`: time threshold to count a stop decision as a new one.
    - `stop_decision.dist_count_threshold_m`: distance threshold to count a stop decision as a new one.
    - `stop_decision.topic_prefix`: topic prefix for planning factors
    - `stop_decision.module_list`: list of modules' names to check, the `{topic_prefix}/{module_name}` should be a valid topic.
  - Sub-metrics to publish:
    - `/{module_name}/keep_duration`(value-based): the time current stop decision keeps.
    - `/{module_name}/distance_to_stop`(value-based): the distance to the stop line.
    - `/{module_name}/count`(count-based): the stop decision index, in other words, the number of stop decisions made by the module.
  - Sub-metrics to output:
    - `/{module_name}/keep_duration/mean`, `/{module_name}/keep_duration/min`, `/{module_name}/keep_duration/max`: the statistics of the published keep_duration.
    - `/{module_name}/count`: the total number of stop decisions.

- **`abnormal_stop_decision`**: Evaluate abnormal stop decisions for each module.
  - A stop decision is considered as abnormal if the ego cannot stop with the current velocity and the maximum deceleration limit.
  - Parameters:
    - `stop_decision.abnormal_deceleration_threshold_mps2`: maximum deceleration limit to consider the stop decision as abnormal.
    - Other parameters are shared with `stop_decision`.
  - Sub-metrics to publish: The same as `stop_decision`.
  - Sub-metrics to output: The same as `stop_decision`.

### Blinker Metrics

Evaluates the blinker status of the vehicle.

Metrics are calculated and publish only when the node receives a turn indicators report message.

#### Implemented metrics

- **`blinker_change_count`**: Count blinker status changes.
  - A change is counted when the blinker status transitions from off/left to right, or from off/right to left.
- Parameters:
  - `blinker_change_count.window_duration_s`: duration to count the changes for publishing.
- Sub-metrics to publish: `/count_in_duration`.
- Sub-metrics to output:
  - `/count_in_duration/min`, `/count_in_duration/max`, `/count_in_duration/mean`: the statistics of the published keep_duration.
  - `/count`: total number of changes.

### Steering Metrics

Evaluates the steering status of the vehicle.

Metrics are calculated and publish only when the node receives a steering report message.

### Implemented metrics

- **`steer_change_count`**: Count steering rate changes.
  - A change is counted when the steering rate changes from positive/0 to negative or from negative/0 to positive.
  - Parameters:
    - `steer_change_count.window_duration_s`: duration to count the changes for publishing.
    - `steer_change_count.steer_rate_margin`: margin to consider the steer rate as 0.
  - Sub-metrics to publish: `/count_in_duration`.
  - Sub-metrics to output:
    - `/count_in_duration/min`, `/count_in_duration/max`, `/count_in_duration/mean`: the statistics of the published keep_duration.
    - `/count`: total number of changes.

### Other Information

Additional useful information related to planning:

#### Implemented metrics

- **`kinematic_state`**: Current kinematic state of the vehicle
  - Sub-metrics to publish:
    - `/velocity`: current ego velocity.
    - `/acceleration`: current ego acceleration.
    - `/jerk`: current ego jerk.

- **`ego_lane_info`**: Lanelet information.
  - Sub-metrics to publish:
    - `/lanelet_id`: ID of the lanelet where the ego is located.
    - `/s`: Arc length of ego position in the lanelet.
    - `/t`: Lateral offset of ego position in the lanelet.

## Inputs / Outputs

### Inputs

| Name                             | Type                                                        | Description                                       |
| -------------------------------- | ----------------------------------------------------------- | ------------------------------------------------- |
| `~/input/trajectory`             | `autoware_planning_msgs::msg::Trajectory`                   | Main trajectory to evaluate                       |
| `~/input/reference_trajectory`   | `autoware_planning_msgs::msg::Trajectory`                   | Reference trajectory to use for deviation metrics |
| `~/input/objects`                | `autoware_perception_msgs::msg::PredictedObjects`           | Obstacles                                         |
| `~/input/modified_goal`          | `autoware_planning_msgs::msg::PoseWithUuidStamped`          | Modified goal                                     |
| `~/input/odometry`               | `nav_msgs::msg::Odometry`                                   | Current odometry of the vehicle                   |
| `~/input/route`                  | `autoware_planning_msgs::msg::LaneletRoute`                 | Route information                                 |
| `~/input/vector_map`             | `autoware_map_msgs::msg::LaneletMapBin`                     | Vector map information                            |
| `~/input/acceleration`           | `geometry_msgs::msg::AccelWithCovarianceStamped`            | Current acceleration of the vehicle               |
| `~/input/steering_status`        | `autoware_vehicle_msgs::msg::SteeringReport`                | Current steering of the vehicle                   |
| `~/input/turn_indicators_status` | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`          | Current blinker status of the vehicle             |
| `{topic_prefix}/{module_name}`   | `autoware_internal_planning_msgs::msg::PlanningFactorArray` | Planning factors of each module to evaluate       |

### Outputs

Each publishing-based metric is published on the same topic.

| Name                         | Type                                                | Description                            |
| ---------------------------- | --------------------------------------------------- | -------------------------------------- |
| `~/metrics`                  | `tier4_metric_msgs::msg::MetricArray`               | MetricArray with all published metrics |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Node processing time in milliseconds   |

- If `output_metrics = true`, the evaluation node writes the output-based metrics measured during its lifetime
  to `<ros2_logging_directory>/autoware_metrics/<node_name>-<time_stamp>.json` when shut down.

## Parameters

{{ json_to_markdown("evaluator/autoware_planning_evaluator/schema/autoware_planning_evaluator.schema.json") }}

## Assumptions / Known limits

There is a strong assumption that when receiving a trajectory `T(0)`,
it has been generated using the last received reference trajectory and objects.
This can be wrong if a new reference trajectory or objects are published while `T(0)` is being calculated.

Precision is currently limited by the resolution of the trajectories.
It is possible to interpolate the trajectory and reference trajectory to increase precision but would make computation significantly more expensive.

## Future extensions / Unimplemented parts

- Use `Route` or `Path` messages as reference trajectory.
- RSS metrics (done in another node <https://tier4.atlassian.net/browse/AJD-263>).
- `motion_evaluator_node`.
  - Node which constructs a trajectory over time from the real motion of ego.
  - Only a proof of concept is currently implemented.
- Take into account the shape, not only the centroid of the object for the obstacle metrics.
