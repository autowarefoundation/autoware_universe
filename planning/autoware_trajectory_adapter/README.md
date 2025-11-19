# Trajectory Adapter

## Purpose/Role

This node converts a set of ranked candidate trajectories into a single, cleaned‑up [autoware_planning_msgs/msg/Trajectory](https://github.com/autowarefoundation/autoware_msgs/blob/main/autoware_planning_msgs/msg/Trajectory.msg).

## Algorithm Overview

Upon receiving a message, the node finds the candidate with the highest score.
If no candidate is found an empty list would be published. The selected trajectory’s points are then processed, ensuring that successive points match the conditions to conduct control modules.

## Interface

### Topics

| Direction | Topic Name             | Message Type                                                      | Description                        |
| --------- | ---------------------- | ----------------------------------------------------------------- | ---------------------------------- |
| Subscribe | `~/input/trajectories` | `autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories` | Candidate trajectories with scores |
| Publish   | `~/output/trajectory`  | `autoware_planning_msgs/msg/Trajectory`                           | Re-arranged best‑score trajectory  |

### Parameters

The current implementation does not expose any ROS parameters. All behavior is hard‑coded.

## Future Work

The following features and improvements are planned for future development:

- **Turn Signal Control**: Implement proper turn signal activation based on trajectory curvature and lane change intentions
- **Hazard Light Control**: Add logic to activate hazard lights in emergency situations or when the vehicle needs to warn other traffic participants
