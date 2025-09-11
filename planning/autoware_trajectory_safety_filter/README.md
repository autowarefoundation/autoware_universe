# Feasible Trajectory Filter

## Purpose/Role

This node performs a feasibility check on each candidate trajectory before it enters the final ranking stage. It drops paths that are physically impossible for the ego vehicle or that create an obvious driving risk.

## Algorithm Overview

The node operates in three broad steps: collect the latest environment inputs, screen trajectories through a set of feasibility checks, then republish whichever paths survive.

Checks applied to each trajectory:

- Data validity: removes trajectories that contain NaNs, non‑finite numbers, inconsistent timestamps, or are too short.
- Proximity to ego: removes trajectories whose nearest point is farther than 5 m from the current ego position.
- Lane adherence: removes trajectories that will exit all lanelets within the configured look‑ahead time.
- Collision risk: removes trajectories whose estimated time‑to‑collision with any predicted object falls below threshold in the look‑ahead time.

After these checks, the remaining trajectories, along with their original `generator_info`, are published.

## Interface

### Topics

| Direction | Topic name              | Message type                                    | Description                                   |
| --------- | ----------------------- | ----------------------------------------------- | --------------------------------------------- |
| Subscribe | `~/input/trajectories`  | `autoware_new_planning_msgs/msg/Trajectories`   | Candidate trajectory                          |
| Subscribe | `~/input/lanelet2_map`  | `autoware_lanelet2_msgs/msg/LaneletMapBin`      | HD map                                        |
| Subscribe | `~/input/odometry`      | `nav_msgs/msg/Odometry`                         | Current ego pose                              |
| Subscribe | `~/input/objects`       | `autoware_perception_msgs/msg/PredictedObjects` | Obstacles for collision checking              |
| Publish   | `~/output/trajectories` | `autoware_new_planning_msgs/msg/Trajectories`   | Trajectories that pass all feasibility checks |

### Parameters

| Parameter name       | Type   | Default | Description                                                             |
| -------------------- | ------ | ------- | ----------------------------------------------------------------------- |
| `out_of_lane.enable` | bool   | true    | Turn the lane‑deviation check on/off                                    |
| `out_of_lane.time`   | double | 3.0     | Look‑ahead time [s] during which the trajectory must stay inside a lane |
| `collision.enable`   | bool   | true    | Enable the obstacle‑collision check                                     |
| `collision.time`     | double | 3.0     | Look‑ahead time[s] for collision search                                 |
