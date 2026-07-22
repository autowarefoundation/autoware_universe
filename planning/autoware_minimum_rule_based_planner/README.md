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

{{ json_to_markdown("planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.
