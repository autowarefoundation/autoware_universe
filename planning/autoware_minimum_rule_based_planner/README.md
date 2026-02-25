# autoware_minimum_rule_based_planner

## Overview

A minimum rule-based trajectory planner that generates safe and feasible trajectories for autonomous driving. It follows the planned route by constructing trajectories from lanelet centerline and applies multi-stage optimization for geometric smoothness, velocity profiles, and obstacle avoidance.

## Features

- **Centerline-based path planning**: Generates paths along lanelet centerline from the HD map, extending backward and forward from the ego vehicle's position
- **Smooth goal connection**: Refines the path near the goal pose for smooth stopping
- **Path shifting**: Shifts the centerline path to start from the ego vehicle's current pose, using curvature-aware shift distance calculation based on ego velocity and lateral acceleration limits
- **Trajectory smoothing**: Applies an Elastic Band smoother for geometric smoothing via a plugin interface
- **Trajectory modification**: Applies modifier plugins (e.g., obstacle stop) for safety modifications
- **Velocity optimization**: Computes a jerk-filtered velocity profile respecting constraints on acceleration, jerk, and lateral acceleration
- **Test mode**: Supports bypassing path planning by directly receiving a `PathWithLaneId` topic

## Inputs / Outputs

### Inputs

| Topic                            | Type                         | Description                       |
| -------------------------------- | ---------------------------- | --------------------------------- |
| `~/input/route`                  | `LaneletRoute`               | Planned route                     |
| `~/input/vector_map`             | `LaneletMapBin`              | HD map                            |
| `~/input/odometry`               | `Odometry`                   | Ego pose and velocity             |
| `~/input/acceleration`           | `AccelWithCovarianceStamped` | Ego acceleration                  |
| `~/input/objects`                | `PredictedObjects`           | Surrounding obstacles             |
| `~/input/test/path_with_lane_id` | `PathWithLaneId`             | Test mode: bypasses path planning |

### Outputs

| Topic                                 | Type                    | Description                                   |
| ------------------------------------- | ----------------------- | --------------------------------------------- |
| `~/output/candidate_trajectories`     | `CandidateTrajectories` | Planned trajectory                            |
| `~/debug/path_with_lane_id`           | `PathWithLaneId`        | Debug: planned path                           |
| `~/debug/trajectory`                  | `Trajectory`            | Debug: final output trajectory                |
| `~/debug/shifted_trajectory`          | `Trajectory`            | Debug: trajectory after path shifting         |
| `~/debug/optimizer/{name}/trajectory` | `Trajectory`            | Debug: trajectory after each optimizer plugin |
| `~/debug/modifier/{name}/trajectory`  | `Trajectory`            | Debug: trajectory after each modifier plugin  |
| `~/debug/processing_time_detail_ms`   | `ProcessingTimeDetail`  | Debug: processing time breakdown              |

## Parameters

### Main parameters (`param/minimum_rule_based_planner_parameters.yaml`)

| Parameter                                                  | Default | Description                                                |
| ---------------------------------------------------------- | ------- | ---------------------------------------------------------- |
| `planning_frequency_hz`                                    | `10.0`  | Planning loop frequency [Hz]                               |
| `path_planning.path_length.backward`                       | `5.0`   | Path length behind ego [m]                                 |
| `path_planning.path_length.forward`                        | `100.0` | Path length ahead of ego [m]                               |
| `path_planning.output.delta_arc_length`                    | `0.5`   | Resampling interval for path-to-trajectory conversion [m]  |
| `path_planning.path_shift.enable`                          | `true`  | Enable path shifting to ego pose                           |
| `path_planning.path_shift.minimum_shift_length`            | `0.1`   | Lateral offset threshold to trigger shift [m]              |
| `path_planning.path_shift.minimum_shift_yaw`               | `0.1`   | Yaw deviation threshold to trigger shift [rad]             |
| `path_planning.path_shift.minimum_shift_distance`          | `5.0`   | Floor for shift distance [m]                               |
| `path_planning.path_shift.min_speed_for_curvature`         | `2.77`  | Lower bound on speed for curvature computation [m/s]       |
| `path_planning.path_shift.lateral_accel_limit`             | `0.5`   | Lateral acceleration limit for shift [m/s^2]               |
| `path_planning.waypoint_group.separation_threshold`        | `1.0`   | Max distance between waypoints in same group [m]           |
| `path_planning.waypoint_group.interval_margin_ratio`       | `10.0`  | Ratio to expand group interval by lateral distance         |
| `path_planning.smooth_goal_connection.search_radius_range` | `7.5`   | Search radius for goal connection [m]                      |
| `path_planning.smooth_goal_connection.pre_goal_offset`     | `1.0`   | Offset before goal for smooth connection [m]               |
| `path_planning.ego_nearest_lanelet.dist_threshold`         | `3.0`   | Distance threshold for ego nearest lanelet [m]             |
| `path_planning.ego_nearest_lanelet.yaw_threshold`          | `1.046` | Yaw threshold for ego nearest lanelet [rad]                |
| `post_resample.max_trajectory_length`                      | `300.0` | Max trajectory length for post-optimization resampling [m] |
| `post_resample.min_trajectory_length`                      | `30.0`  | Min trajectory length for post-optimization resampling [m] |
| `post_resample.resample_time`                              | `10.0`  | Resample total time [s]                                    |
| `post_resample.dense_resample_dt`                          | `0.1`   | Dense sampling time interval [s]                           |
| `post_resample.dense_min_interval_distance`                | `0.1`   | Dense sampling min interval [m]                            |
| `post_resample.sparse_resample_dt`                         | `0.1`   | Sparse sampling time interval [s]                          |
| `post_resample.sparse_min_interval_distance`               | `1.0`   | Sparse sampling min interval [m]                           |

### Velocity smoother parameters (`config/minimum_rule_based_planner.param.yaml`)

| Parameter                                      | Default | Description                            |
| ---------------------------------------------- | ------- | -------------------------------------- |
| `velocity_smoother.max_speed_mps`              | `13.88` | Max velocity [m/s]                     |
| `velocity_smoother.max_lateral_accel_mps2`     | `1.5`   | Max lateral acceleration [m/s^2]       |
| `velocity_smoother.target_pull_out_speed_mps`  | `0.25`  | Target pull-out speed [m/s]            |
| `velocity_smoother.target_pull_out_acc_mps2`   | `0.5`   | Target pull-out acceleration [m/s^2]   |
| `velocity_smoother.set_engage_speed`           | `true`  | Set engage speed at start              |
| `velocity_smoother.limit_speed`                | `true`  | Limit max speed                        |
| `velocity_smoother.limit_lateral_acceleration` | `false` | Limit lateral acceleration             |
| `velocity_smoother.smooth_velocities`          | `true`  | Apply jerk-filtered velocity smoothing |

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.
