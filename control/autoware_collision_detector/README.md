# Collision Detector

## Purpose

This module subscribes required data (ego-pose, obstacles, etc), and publishes diagnostics if an object is within a specific distance.

## Inner-workings / Algorithms

### Flow chart

### Algorithms

### Check data

Check that `collision_detector` receives no ground pointcloud, dynamic objects.

### Object Filtering

#### Recognition Assumptions

1. If the classification changes but it's considered the same object, the uuid does not change.
2. It's possible for the same uuid to be recognized after being lost for a few frames.
3. Once an object is determined to be excluded, it continues to be excluded for a certain period of time.

#### Filtering Process

1. Initial Recognition and Exclusion:

   - The system checks if a newly recognized object's classification is listed in `nearby_object_type_filters`.
   - If so, and the object is within the `nearby_filter_radius`, it is marked for exclusion.

2. New Object Determination:

   - An object is considered "new" based on its UUID.
   - If the UUID is not found in recent frame data, the object is treated as new.

3. Exclusion Mechanism:
   - Newly excluded objects are recorded by their UUID.
   - These objects continue to be excluded for a set period (`keep_ignoring_time`) as long as they maintain the classification specified in `nearby_object_type_filters` and remain within the `nearby_filter_radius`.

### Get distance to nearest object

Calculate distance between ego vehicle and the nearest object.
In this function, it calculates the minimum distance between the polygon of ego vehicle and all points in pointclouds and the polygons of dynamic objects.

## Inputs / Outputs

### Input

| Name                                           | Type                                              | Description                                                        |
| ---------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------ |
| `/perception/obstacle_segmentation/pointcloud` | `sensor_msgs::msg::PointCloud2`                   | Pointcloud of obstacles which the ego-vehicle should stop or avoid |
| `/perception/object_recognition/objects`       | `autoware_perception_msgs::msg::PredictedObjects` | Dynamic objects                                                    |
| `/tf`                                          | `tf2_msgs::msg::TFMessage`                        | TF                                                                 |
| `/tf_static`                                   | `tf2_msgs::msg::TFMessage`                        | TF static                                                          |

### Output

| Name           | Type                                    | Description |
| -------------- | --------------------------------------- | ----------- |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | Diagnostics |

## Parameters

| Name                         | Type                    | Description                                                                              | Default value                    |
| :--------------------------- | :---------------------- | :--------------------------------------------------------------------------------------- | :------------------------------- |
| `use_pointcloud`             | `bool`                  | Use pointcloud as obstacle check                                                         | `true`                           |
| `use_dynamic_object`         | `bool`                  | Use dynamic object as obstacle check                                                     | `true`                           |
| `collision_distance`         | `double`                | Distance threshold at which an object is considered a collision. [m]                     | 0.15                             |
| `nearby_filter_radius`       | `double`                | Distance range for filtering objects. Objects within this radius are considered. [m]     | 5.0                              |
| `keep_ignoring_time`         | `double`                | Time to keep filtering objects that first appeared in the vicinity [sec]                 | 10.0                             |
| `nearby_object_type_filters` | `object of bool values` | Specifies which object types to filter. Only objects with `true` value will be filtered. | `{unknown: true, others: false}` |

## Assumptions / Known limits

- This module is based on `surround_obstacle_checker`
