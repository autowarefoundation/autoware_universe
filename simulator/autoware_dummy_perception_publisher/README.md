# dummy_perception_publisher

## Purpose

This node publishes the result of the dummy detection with the type of perception. The node uses a plugin architecture to support different object movement strategies.

## Inner-workings / Algorithms

### Plugin Architecture

The node uses a plugin-based architecture to handle different types of object movement. This allows for flexible and extensible movement behaviors without modifying the core node logic.

#### Available Plugins

1. **StraightLineObjectMovementPlugin**: Moves objects in straight lines based on their initial velocity and direction.
2. **PredictedObjectMovementPlugin**: Moves objects along predicted paths from perception predictions, providing more realistic movement patterns.

#### Plugin Base Class

All movement plugins inherit from `DummyObjectMovementBasePlugin` which provides:

- Object management (add, delete operations)
- Associated action type handling
- Common interface for object movement

#### Object Action Handling

- **ADD**: New objects are created and they move in a straight line, acceleration and deceleration parameters can be used.
- **MODIFY**: Handled directly by the node, bypassing plugin movement logic. Immediately replaces the object's position information across all plugins.
- **DELETE**: The specified object is removed from all plugins.
- **DELETEALL**: Clears all objects from all plugins.
- **PREDICT**: New objects are created, they move in a straight line for a set time and then the predictions extracted from the perception module are used to dictate where the objects will move to. NOTE: for ease of calculation, acceleration is not taken into account when calculating the object's position, only its initial speed.

## Inputs / Outputs

### Input

| Name                | Type                                              | Description                                               |
| ------------------- | ------------------------------------------------- | --------------------------------------------------------- |
| `/tf`               | `tf2_msgs/TFMessage`                              | TF (self-pose)                                            |
| `input/object`      | `tier4_simulation_msgs::msg::DummyObject`         | dummy detection objects                                   |
| `predicted_objects` | `autoware_perception_msgs::msg::PredictedObjects` | predicted objects (used by PredictedObjectMovementPlugin) |

### Output

| Name                                | Type                                                     | Description             |
| ----------------------------------- | -------------------------------------------------------- | ----------------------- |
| `output/dynamic_object`             | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | dummy detection objects |
| `output/points_raw`                 | `sensor_msgs::msg::PointCloud2`                          | point cloud of objects  |
| `output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects`          | ground truth objects    |

## Parameters

### Core Parameters

| Name                        | Type   | Default Value | Explanation                                        |
| --------------------------- | ------ | ------------- | -------------------------------------------------- |
| `visible_range`             | double | 100.0         | sensor visible range [m]                           |
| `detection_successful_rate` | double | 0.8           | sensor detection rate. (min) 0.0 - 1.0(max)        |
| `enable_ray_tracing`        | bool   | true          | if True, use ray tracking                          |
| `use_object_recognition`    | bool   | true          | if True, publish objects topic                     |
| `use_base_link_z`           | bool   | true          | if True, node uses z coordinate of ego base_link   |
| `publish_ground_truth`      | bool   | false         | if True, publish ground truth objects              |
| `use_fixed_random_seed`     | bool   | false         | if True, use fixed random seed                     |
| `random_seed`               | int    | 0             | random seed                                        |
| `object_centric_pointcloud` | bool   | false         | if True, generate object-centric point clouds      |
| `angle_increment`           | double | 0.004363323   | angle increment for ray tracing (0.25° in radians) |

### PredictedObjectMovementPlugin Parameters

| Name                               | Type   | Default Value | Explanation                                                             |
| ---------------------------------- | ------ | ------------- | ----------------------------------------------------------------------- |
| `min_predicted_path_keep_duration` | double | 3.0           | minimum time (seconds) to keep using same prediction                    |
| `switch_time_threshold`            | double | 2.0           | time threshold (seconds) to switch from straight-line to predicted path |

#### Common Remapping Parameters

The plugin uses `CommonParameters` for both vehicle and pedestrian object types. Each parameter is prefixed with either `vehicle.` or `pedestrian.`:

| Parameter Name               | Type   | Explanation                                               |
| ---------------------------- | ------ | --------------------------------------------------------- |
| `max_remapping_distance`     | double | maximum distance (meters) for remapping validation        |
| `max_speed_difference_ratio` | double | maximum speed difference ratio tolerance                  |
| `min_speed_ratio`            | double | minimum speed ratio relative to dummy object speed        |
| `max_speed_ratio`            | double | maximum speed ratio relative to dummy object speed        |
| `speed_check_threshold`      | double | speed threshold (m/s) above which speed checks apply      |
| `path_selection_strategy`    | string | path selection strategy: "highest_confidence" or "random" |
