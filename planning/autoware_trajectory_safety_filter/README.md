# Trajectory safety filter

## Purpose/Role

This node performs a safety check on each candidate trajectory before it enters the final ranking stage. It drops paths that are physically impossible for the ego vehicle or that create an obvious driving risk.

## Algorithm Overview

The node operates in three broad steps: collect the latest environment inputs, screen trajectories through a set of feasibility checks, then republish whichever paths survive.

Checks applied to each trajectory:

- Data validity: removes trajectories that contain NaNs, non‑finite numbers, inconsistent timestamps, or are too short.
- Lane adherence: removes trajectories that will exit all lanelets within the configured look‑ahead time.
- Collision risk: removes trajectories whose estimated time‑to‑collision with any predicted object falls below threshold in the look‑ahead time.

After these checks, the remaining trajectories, along with their original `generator_info`, are published.

## Interface

### Topics

| Direction | Topic name              | Message type                                            | Description                                   |
| --------- | ----------------------- | ------------------------------------------------------- | --------------------------------------------- |
| Subscribe | `~/input/trajectories`  | `autoware_internal_planning_msgs/CandidateTrajectories` | Candidate trajectories                        |
| Subscribe | `~/input/lanelet2_map`  | `autoware_map_msgs/msg/LaneletMapBin`                   | HD map                                        |
| Subscribe | `~/input/odometry`      | `nav_msgs/msg/Odometry`                                 | Current ego pose                              |
| Subscribe | `~/input/objects`       | `autoware_perception_msgs/msg/PredictedObjects`         | Obstacles for collision checking              |
| Publish   | `~/output/trajectories` | `autoware_internal_planning_msgs/CandidateTrajectories` | Trajectories that pass all feasibility checks |

### Parameters

| Parameter name          | Type         | Default | Description                                                                   |
| ----------------------- | ------------ | ------- | ----------------------------------------------------------------------------- |
| `filter_names`          | string array | []      | List of safety filter plugins to use (e.g., OutOfLaneFilter, CollisionFilter) |
| `out_of_lane.time`      | double       | 3.0     | Look-ahead time [s] during which the trajectory must stay inside a lane       |
| `out_of_lane.min_value` | double       | 0.0     | Minimum distance [m] from lane boundary                                       |
| `collision.time`        | double       | 3.0     | Look-ahead time [s] for collision search                                      |
| `collision.min_value`   | double       | 2.0     | Minimum acceptable time to collision [s]                                      |

## Future Work

### Performance Optimization

The current implementation can be further optimized for computational efficiency:

- **Caching Strategy**: Implement smarter caching mechanisms for lanelet queries and boundary checks to avoid redundant computations across similar trajectories
- **Adaptive Resolution**: Dynamically adjust the checking resolution based on vehicle speed and trajectory curvature to balance accuracy and performance
