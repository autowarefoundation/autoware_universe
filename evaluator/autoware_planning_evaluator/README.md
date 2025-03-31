# Planning Evaluator

## Purpose

This package provides nodes that generate various metrics to evaluate the quality of planning.

Metrics can be published in real time and saved to a JSON file when the node is shut down:
- `metrics_for_publish`:
  - The metrics listed in `metrics_for_publish` are calclulated and published on the topic.

- `metrics_for_output`:
  - The metrics listed in `metrics_for_output` are saved to a JSON file when the node is shut down if `output_metrics` is set to `true`. 
  - Most of `metrics_for_output` are the statistics of the `metrics_for_publish`.
  - Additional information such as the `parameters` and `description` for metrics are also saved in the JSON file.

## Metrics

All possible metrics are defined in the `Metric` enumeration defined `include/autoware/planning_evaluator/metrics/metric.hpp` and `include/autoware/planning_evaluator/metrics/output_metric.hpp`. Those files also defines conversions from/to string as well as human readable descriptions to be used as header of the output file.

- From the view of data types, the metrics can be classified into three categories:
  - Statics-based metric:
    - It is calculated using a `autoware_utils::Accumulator` instance which contains the minimum, maximum, and mean values calculated for the metric as well as the number of values measured.
    - Its sub-metrics may contains `/mean`, `/min`, `/max` and/or `/count`.

  - Value-based metric:
    - The metric with the single value.
    - Its sub-metrics may contains `/value`.
    - Some metrics with the old implementation use the statics-based format of `/mean`, `/min`, `/max`, but all values are the same.

  - Count-based metric:
    - It count the number of occurrences of the certain event in the given time period.
    - Its sub-metrics may contains `/count` or `/count_in_duration`.

- From the view of the purpose, we classify the metrics into the following categories:
  - [Trajectory metrics](#trajectory-metrics)
  - [Trajectory deviation metrics](#trajectory-deviation-metrics)
  - [Trajectory stability metrics](#trajectory-stability-metrics)
  - [Trajectory obstacle metrics](#trajectory-obstacle-metrics)
  - [Modified goal metrics](#modified-goal-metrics)
  - [Planning factor metrics](#planning-factor-metrics)
  - [Steering metrics](#steering-metrics)
  - [Blinker metrics](#blinker-metrics)
  - [Other information](#other-information)

### Trajectory Metrics

Evaluates the trajectory `T(0)` itself.

Metrics are calculated and publish only when the node receives a trajectory.

#### Implemented metrics:

`curvature`: Statics of curvature of each point in the trajectory.
- Sub-metrics to publish: 
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output: 
  - The same as above but take the published data as data point instead of each trajectory point.

`point_interval`: Statics of the distance between each point in the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`relative_angle`: Statics of the angle between each point in the trajectory and the previous point.
- Parameters:
  - `trajectory.min_point_dist_m`: minimum distance between two successive points.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`resampled_relative_angle`: similar as `relative_angle`, but considers a point at a certain distance (e.g., half the vehicle length) from the current point as the next point to calculate the relative angle, instead of using the immediately adjacent point.

`length`: Length of the trajectory.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

`duration`: Expected driving time to travel the trajectory.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

`velocity`: Statics of the velocity of each point in the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`acceleration`: Statics of the acceleration of each point in the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`jerk`: Statics of the jerk of each point in the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

### Trajectory Deviation Metrics

Evaluates the trajectory deviation by comparing the trajectory `T(0)` and the reference trajectory.

Metrics are calculated and publish only when the node receives a trajectory.

The following information are used to calculate metrics:
- the trajectory `T(0)`.
- the _reference_ trajectory assumed to be used as the reference to plan `T(0)`.

#### Implemented metrics:

`lateral_deviation`: Statics of the lateral deviation between the point on the trajectory and the closest point on the reference trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`yaw_deviation`: Statics of the yaw deviation between the point on the trajectory and the closest point on the reference trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`velocity_deviation`: Statics of the velocity deviation between the point on the trajectory and the closest point on the reference trajectory.
- Sub-metrics to publish:
  - `/mean`, `/min`, `/max`: mean, min, and max of data.
- Sub-metrics to output: the same as above but take the statistics of each published data instead of each point.


### Trajectory Stability Metrics

Evaluates the trajectory stability by comparing the trajectory `T(0)` and previous trajectory `T(-1)`.

Metrics are calculated and publish only when the node receives a trajectory.

The following information are used to calculate metrics, which are maintained by an instance of class `MetricsCalculator`:
- the trajectory `T(0)` itself.
- the previous trajectory `T(-1)`.
- the current ego odometry.

#### Implemented metrics:

`stability`: Statistics of the lateral deviation between the current trajectory and the previous trajectory within the certain lookahead duration and lookahead distance.
- Parameters:
  - `trajectory.lookahead.max_time_s`: lookahead distance.
  - `trajectory.lookahead.max_dist_m`: lookahead time duration.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`stability_frechet`: Statistics of the Frechet distance between the current trajectory and the previous trajectory within the certain lookahead duration and lookahead distance.
- Parameters:
  - `trajectory.lookahead.max_time_s`: lookahead distance.
  - `trajectory.lookahead.max_dist_m`: lookahead time duration.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`lateral_trajectory_displacement_local`: Absolute lateral displacement between the current trajectory and the previous trajectory at the current ego position.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

`lateral_trajectory_displacement_lookahead`: Statistics of absolute lateral displacement between the current trajectory and the previous trajectory within the certain lookahead duration.
- Parameters:
  - `trajectory.evaluation_time_s`: lookahead time duration.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.


### Trajectory Obstacle Metrics

Evaluates if the trajectory `T(0)` is safe for obstacles in the environment.

Metrics are calculated and publish only when the node receives a trajectory.
The following information are used to calculate metrics:
- the trajectory `T(0)`.
- the set of objects in the environment.

#### Implemented metrics:

`obstacle_distance`: Statistics of the distance between the centroid of objects and the closest point on the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

`obstacle_ttc`: Statistics of the time-to-collision (TTC) for those objects which is close to the trajectory.
- Parameters:
  - `obstacle.dist_thr_m`: distance threshold to consider the object as close to the trajectory.
- Sub-metrics to publish:
  - Statics-based metrics of `/mean`, `/min`, `/max`.
- Sub-metrics to output:
  - The same as above but take the published data as data point instead of each trajectory point.

### Modified Goal Metrics

Evaluates the deviation between the modified goal and the current ego position.

Metrics are calculated and publish only when the node receives a modified goal message.

#### Implemented metrics:

`modified_goal_longitudinal_deviation`: Statistics of the longitudinal deviation between the modified goal and the current ego position.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

`modified_goal_lateral_deviation`: Statistics of the lateral deviation between the modified goal and the current ego position.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

`modified_goal_yaw_deviation`: Statistics of the yaw deviation between the modified goal and the current ego position.
- Sub-metrics to publish:
  - Value-based metric, but using the statics-based format.
- Sub-metrics to output:
  - Statics-based metrics of `/mean`, `/min`, `/max` for the published data.

### Planning Factor Metrics

Evaluates the behavior of each planning module by checking their planning factors.

Metrics are calculated and publish only when the node receives that module's planning factors.

The modules listed in the `module_list` in the parameter file are evaluated. 

#### Implemented metrics:

`stop_decision`: Evaluates the stop decision of each module.
- Parameters:
  - `stop_decision.time_count_threshold_s`: time threshold to count a stop decision as a new one.
  - `stop_decision.dist_count_threshold_m`: distance threshold to count a stop decision as a new one.
  - `stop_decision.topic_prefix`: topic prefix for planning factors
  - `stop_decision.module_list`: list of modules' names  to check, the `{topic_prefix}/{module_name}` should be a valid topic.
- Sub-metrics to publish:
  - Value-based metric of `/{module_name}/keep_duration`.
  - Value-based metric of `/{module_name}/distance_to_stop`.
- Sub-metrics to output:
  - Statics-based metric of `/{module_name}/keep_duration/mean`, `/{module_name}/keep_duration/min`, `/{module_name}/keep_duration/max` for the published data.
  - Count-based metric of `/{module_name}/count` for the total number of stop decisions.

`abnormal_stop_decision`: Evaluates the abnormal stop decision of each module.
- A stop decision is considered as abnormal if the ego cannot stop with the current velocity and the maximum deceleration limit.
- Parameters:
  - `stop_decision.abnormal_deceleration_threshold_mps2`: maximum deceleration limit to consider the stop decision as abnormal.
  - Other parameters are shared with `stop_decision`.
- Sub-metrics to publish:
  - The same as `stop_decision`.
- Sub-metrics to output:
  - The same as `stop_decision`.

### Blinker Metrics

Evaluates the blinker status of the vehicle.

Metrics are calculated and publish only when the node receives a turn indicators report message.

#### Implemented metrics:

`blinker_change_count`: Count the number of times the blinker status changes.
- When the blinker status changes from off/left to right or from off/right to left, it is counted as a change.
- Parameters:
  - `blinker_change_count.window_duration_s`: duration to count the changes.
- Sub-metrics to publish:
  - Count-based metric of `/count_in_duration`.
- Sub-metrics to output:
  - Count-based metric of `/count` for total number of changes.
  - Statics-based metric of `/count_in_duration/min`, `/count_in_duration/max`, `/count_in_duration/mean` for statistics of the published data.


### Steering Metrics

Evaluates the steering status of the vehicle.

Metrics are calculated and publish only when the node receives a steering report message.

### Implemented metrics:

`steer_change_count`: Count the changes of steer_rate changes positive and negative within the past certain duration.
- When the steer rate changes from positive/0 to negative or from negative/0 to positive, it is counted as a change.
- Parameters:
  - `steer_change_count.window_duration_s`: duration to count the changes.
  - `steer_change_count.steer_rate_margin`: margin to consider the steer rate as 0.
- Sub-metrics to publish:
  - Count-based metric of `/count_in_duration`.
- Sub-metrics to output:
  - Count-based metric of `/count` for total number of changes.
  - Statics-based metric of `/count_in_duration/min`, `/count_in_duration/max`, `/count_in_duration/mean` for statistics of the published data.

### Other Information

Some basic information related to the planning which are not metrics but useful are published as well.

#### Implemented metrics:
- `kinematic_state`: Kinematic state of the vehicle.
  - Sub-metrics to publish:
    - `/vel`: current ego velocity.
    - `/acc`: current ego acceleration.
    - `/jerk`: current ego jerk.

- `ego_lane_info`: Lanelet information.
  - Sub-metrics to publish:
    - `/lanelet_id`: ID of the lanelet where the ego is located.
    - `/s`: Arc length of ego position in the lanelet.
    - `/t`: Lateral offset of ego position in the lanelet.

## Inputs / Outputs

### Inputs

| Name                                                                       | Type                                                        | Description                                       |
| -------------------------------------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------- |
| `~/input/trajectory`                                                       | `autoware_planning_msgs::msg::Trajectory`                   | Main trajectory to evaluate                       |
| `~/input/reference_trajectory`                                             | `autoware_planning_msgs::msg::Trajectory`                   | Reference trajectory to use for deviation metrics |
| `~/input/objects`                                                          | `autoware_perception_msgs::msg::PredictedObjects`           | Obstacles                                         |
| `~/input/modified_goal`                                                    | `autoware_planning_msgs::msg::PoseWithUuidStamped`          | Modified goal                                     |
| `~/input/odometry`                                                         | `nav_msgs::msg::Odometry`                                   | Current odometry of the vehicle                   |
| `~/input/route`                                                            | `autoware_planning_msgs::msg::LaneletRoute`                 | Route information                                 |
| `~/input/vector_map`                                                       | `autoware_map_msgs::msg::LaneletMapBin`                     | Vector map information                            |
| `~/input/acceleration`                                                     | `geometry_msgs::msg::AccelWithCovarianceStamped`            | Current acceleration of the vehicle               |
| `~/input/steering_status`                                                  | `autoware_vehicle_msgs::msg::SteeringReport`                | Current steering of the vehicle                   |
| `~/input/turn_indicators_status`                                           | `autoware_vehicle_msgs::msg::TurnIndicatorsReport`          | Current blinker status of the vehicle             |
| `topic_prefix`/`module` defined in `.config/planning_evaluator.param.yaml` | `autoware_internal_planning_msgs::msg::PlanningFactorArray` | Planning factors for each module to evaluate      |

### Outputs

Each publishing-based metric is published on the same topic.

| Name                         | Type                                                | Description                                                                       |
| ---------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------- |
| `~/metrics`                  | `tier4_metric_msgs::msg::MetricArray`               | MetricArray with all publishing-based metrics of `tier4_metric_msgs::msg::Metric` |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Processing time of the evaluation node in milliseconds                            |

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