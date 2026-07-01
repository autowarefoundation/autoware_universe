# Obstacle Proximity Checker

## Purpose

This package provides a reusable library for detecting obstacles in close proximity to the ego vehicle.
It computes the minimum distance between an expanded ego footprint and nearby point cloud points or dynamic object polygons, and reports whether any obstacle is within a caller-defined distance threshold.

The core logic was extracted from [autoware_surround_obstacle_checker](../../planning/autoware_surround_obstacle_checker/README.md) so it can be shared across planning modules.
Current consumers include:

## Inner-workings / Algorithms

### Get distance to nearest obstacle

Calculate the distance between the ego vehicle and the nearest obstacle.
The minimum distance is computed between ego vehicle footprint and:

- all points in the input point cloud
- the polygons of enabled dynamic objects

The ego footprint is created based on vehicle info and user defined margins per obstacle type. For each given obstacle type (e.g car, pedestrian, pointcloud, etc) the following margins are defined:

- `surround_check_front_distance`
- `surround_check_side_distance`
- `surround_check_back_distance`

### Obstacle detection

An obstacle is considered nearby when:

```text
nearest_distance < contact_distance_threshold
```

`contact_distance_threshold` is provided by the caller on each `check()` call.
This allows callers to implement distance hysteresis (for example, a tight threshold when entering stop and a wider threshold when clearing stop).

The library does **not** manage:

- ego stopped checks
- state machines (`PASS` / `STOP`)
- time-based hysteresis
- ROS subscriptions, publishers, or TF transforms

Those responsibilities remain in the calling node or plugin.

## API

### Input (`Inputs`)

| Field                     | Type                                                              | Description                             |
| ------------------------- | ----------------------------------------------------------------- | --------------------------------------- |
| `ego_pose`                | `geometry_msgs::msg::Pose`                                        | Ego pose used for dynamic object checks |
| `pointcloud_in_base_link` | `pcl::PointCloud<pcl::PointXYZ>::ConstPtr`                        | Obstacle point cloud in `base_link`     |
| `objects`                 | `autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr` | Dynamic objects                         |

Callers must transform the point cloud to `base_link` before passing it to the library.

### Output (`CheckResult`)

| Field               | Type                               | Description                                                           |
| ------------------- | ---------------------------------- | --------------------------------------------------------------------- |
| `is_obstacle_found` | `bool`                             | `true` if the nearest obstacle is within `contact_distance_threshold` |
| `nearest_obstacle`  | `std::optional<ProximityObstacle>` | Nearest obstacle and its distance, if any                             |

`ProximityObstacle` contains:

| Field              | Type                                | Description                                            |
| ------------------ | ----------------------------------- | ------------------------------------------------------ |
| `is_point_cloud`   | `bool`                              | `true` if the nearest obstacle came from a point cloud |
| `nearest_distance` | `double`                            | Minimum distance to the ego footprint [m]              |
| `nearest_point`    | `geometry_msgs::msg::Point`         | Nearest point in map coordinates                       |
| `uuid`             | `unique_identifier_msgs::msg::UUID` | Object UUID, or empty for point clouds                 |

### Parameters (`Parameters`)

| Name                       | Type                                            | Description                                           |
| -------------------------- | ----------------------------------------------- | ----------------------------------------------------- |
| `pointcloud_enable_check`  | `bool`                                          | Enable point cloud proximity checks                   |
| `object_type_enable_check` | `unordered_map<string, bool>`                   | Enable checks per object label (`car`, `truck`, etc.) |
| `obstacle_types_map`       | `unordered_map<string, ObstacleTypeParameters>` | Per-type footprint margins                            |

`ObstacleTypeParameters` fields:

| Name                            | Type     | Description                                 | Typical value |
| ------------------------------- | -------- | ------------------------------------------- | ------------- |
| `surround_check_front_distance` | `double` | Front margin added to the ego footprint [m] | 0.5           |
| `surround_check_side_distance`  | `double` | Side margin added to the ego footprint [m]  | 0.0–1.0       |
| `surround_check_back_distance`  | `double` | Rear margin added to the ego footprint [m]  | 0.0–0.5       |

Supported object labels:

`unknown`, `car`, `truck`, `bus`, `trailer`, `motorcycle`, `bicycle`, `pedestrian`, `pointcloud`

## Assumptions / Known limits

- Point clouds must already be transformed to `base_link`.
- The library performs geometric proximity checks only. Stop/release decisions, velocity limits, and trajectory modification are handled by callers.
