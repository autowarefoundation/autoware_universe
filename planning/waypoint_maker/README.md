# Waypoint Maker

The **Waypoint Maker** is a utility toolset within the Autoware ecosystem that enables recording and replaying of waypoint trajectories for autonomous driving. It includes two core components:

- `waypoint_saver`: Records vehicle poses and velocities into CSV files.
- `waypoint_loader`: Loads saved waypoints, generates a path, and publishes it for vehicle following.

These tools are useful for testing, demonstration, and repeatable path-based navigation.

## Use Cases

- Record waypoint trajectories via GUI by manually driving a route.
- Load saved CSV files to generate drivable paths for autonomous navigation.
- Visualize paths and determine completion of the route.
- Dynamically switch between paths via GUI.
- Conditionally activate path planning based on driving scenario.

## Features

### Waypoint Saver

- Subscribes to odometry and records waypoints at set intervals.
- Supports service-based recording control.
- Auto-generates timestamped backup CSVs.
- Records velocity in km/h along with pose data.

### Waypoint Loader

- Loads and parses CSV files into `Trajectory` and `Path` messages.
- Generates dynamic drivable bounds based on vehicle dimensions.
- Supports forward and reverse path selection.
- Publishes visualization route and waypoint following state.
- Publishes the path when scenario is `WAYPOINTFOLLOWING`.

## Scenario Logic

The Waypoint Loader node is **scenario-aware**, meaning it only activates trajectory execution when the current scenario is set to `WAYPOINTFOLLOWING`. This allows users to keep the loader node running without interfering with other planning modes.

- Subscribes to `/input/scenario` topic.
- Internally monitors the `current_scenario` field.
- Activates path generation and publication only when scenario is under `WAYPOINTFOLLOWING`. The `WAYPOINTFOLLOWING` is triggered by scenario selector only when the vehicle is close to or within in `waypoint_zone`.

## Inputs / Outputs

### Topics

#### Inputs

| Topic             | Type                                      | Description                                  |
|------------------|-------------------------------------------|----------------------------------------------|
| `/input/pose`     | `nav_msgs/msg/Odometry`                   | Ego vehicle pose and velocity                 |
| `/input/scenario` | `tier4_planning_msgs/msg/Scenario`        | Driving scenario type (e.g., waypoint mode)   |
| `/path_file`      | `std_msgs/msg/String`                     | Path to a CSV file to dynamically reload      |

#### Outputs

| Topic                             | Type                                                | Description                                  |
|----------------------------------|-----------------------------------------------------|----------------------------------------------|
| `output/path`                     | `autoware_planning_msgs/msg/Path`                  | Main driving path from waypoints             |
| `output/path_visualization`      | `autoware_planning_msgs/msg/Path`                  | Visual version of the path                   |
| `output/waypoint_following_state`| `std_msgs/msg/Bool`                                 | Flag indicating if goal has been reached     |
| `/file/location`                 | `std_msgs/msg/String`                              | Current loaded file path                     |
| `/planning/route`                | `autoware_planning_msgs/msg/LaneletRoute`          | Pseudo-route message for downstream modules  |

### Services

| Service                       | Type                               | Description                               |
|------------------------------|------------------------------------|-------------------------------------------|
| `/waypoint_maker/record`     | `waypoint_maker_msg/srv/Save`      | Start or stop recording to file           |

## Parameters

### Waypoint Loader

| Name                         | Description                                  | Default |
|------------------------------|----------------------------------------------|---------|
| `lane_csv_position`         | Path to directory of waypoint CSVs           | `./src/universe/.../data/` |
| `node_name_for_max_velocity`| Node to fetch velocity limit from            | `/planning/.../velocity_smoother` |
| `param_name_for_max_velocity`| Parameter name for max velocity             | `max_vel` |
| `bound_margin`              | Width margin for drivable area               | `1.0`    |
| `update_rate`               | Timer execution frequency (Hz)               | `10.0`   |
| `check_distance`            | Max lateral distance to consider aligned     | `0.5`    |
| `check_angle_deg`           | Max yaw angle difference (deg)               | `45.0`   |
| `stop_duration`             | Duration to remain stopped before completion | `1.0`    |

### Waypoint Saver

| Name              | Description                                   | Default              |
|-------------------|-----------------------------------------------|----------------------|
| `output_file`     | Path to save the CSV                          | `./src/universe/.../data/waypoint_default.csv`         |
| `save_interval`   | Minimum distance moved to log new waypoint    | `0.5`                |
| `map_frame`       | Target frame for all poses                    | `map`                |
| `base_link_frame` | Ego vehicle frame                             | `base_link`          |

## CSV Format

Waypoints are recorded in the following format:

```csv
x,y,z,yaw,velocity
1.23456,2.34567,0.00000,0.7854,5.0
...
