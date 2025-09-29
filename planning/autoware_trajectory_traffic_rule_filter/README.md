# Valid Trajectory Filter

## Purpose/Role

This node acts as a safety gate between trajectory generation and ranking. It removes any candidate path that would clearly violate immediate traffic rules,so that only lawful, drivable trajectories proceed downstream.

## Algorithm Overview

The node latches the HD map (`LaneletMapBin`) and builds helper structures for routing and traffic‑rule evaluation as soon as it receives the vector map. Whenever a candidate trajectories message arrives, the node updates an internal table of traffic‑light states, and checks whether each candidate path obey the traffic rule. Candidate trajectories which obey the rules are published.

Currently 2 traffic validation are implemented:

- Traffic light validation: Candidates are removed if red traffic light is ignored.
- Stop line validation: Candidates are removed if the trajectory crosses the stop line

## Interface

### Topics

| Direction | Topic name                | Message type                                      | Description                               |
| --------- | ------------------------- | ------------------------------------------------- | ----------------------------------------- |
| Subscribe | `~/input/trajectories`    | `autoware_new_planning_msgs/msg/Trajectories`     | Candidate trajectories                    |
| Subscribe | `~/input/lanelet2_map`    | `autoware_lanelet2_msgs/msg/LaneletMapBin`        | Full HD map                               |
| Subscribe | `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficSignalArray` | Latest perception of traffic‑light states |
| Publish   | `~/output/trajectories`   | `autoware_new_planning_msgs/msg/Trajectories`     | Trajectories that obey traffic rule       |

### Parameters

This node currently exposes no ROS parameters; its behavior is fixed.
