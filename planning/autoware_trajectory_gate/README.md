# autoware_trajectory_gate

## Overview

This package subscribes to multiple trajectory, selects one and publish it.
The selector node monitors the interval of each trajectory topic, and publishes it as diagnostics.

## Parameters

| Name                      | Type      | Description                                  |
| ------------------------- | --------- | -------------------------------------------- |
| trajectory_warn_duration  | double    | Warning threshold of trajectory interval.    |
| trajectory_error_duration | double    | Error threshold of trajectory interval.      |
| source_ids                | list[int] | List of trajectory source IDs.               |
| source.&lt;id&gt;.name    | string    | Name of the corresponding trajectory source. |

## Interfaces

| Interface    | Name                                            | Type                                         | Description                |
| ------------ | ----------------------------------------------- | -------------------------------------------- | -------------------------- |
| Subscription | /trajectory_gate/inputs/&lt;name&gt;/trajectory | autoware_planning_msgs/msg/Trajectory        | Input trajectory.          |
| Publisher    | /trajectory_gate/output/trajectory              | autoware_planning_msgs/msg/Trajectory        | Output trajectory.         |
| Publisher    | /trajectory_gate/source/status                  | tier4_system_msgs/msg/TrajectorySourceStatus | Current trajectory source. |
| Service      | /trajectory_gate/source/change                  | tier4_system_msgs/srv/ChangeTrajectorySource | Change trajectory source.  |
