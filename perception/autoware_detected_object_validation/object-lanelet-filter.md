# object_lanelet_filter

## Purpose

The `object_lanelet_filter` is a node that filters detected object by using vector map.
The objects only inside of the vector map will be published.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name               | Type                                             | Description            |
| ------------------ | ------------------------------------------------ | ---------------------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`          | vector map             |
| `input/object`     | `autoware_perception_msgs::msg::DetectedObjects` | input detected objects |

### Output

| Name            | Type                                             | Description               |
| --------------- | ------------------------------------------------ | ------------------------- |
| `output/object` | `autoware_perception_msgs::msg::DetectedObjects` | filtered detected objects |

## Parameters

Description of the `filter_settings` in the parameters of the `object_lanelet_filter` node.

| Name                                              | Type     | Description                                                                                                                           |
| ------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `debug`                                           | `bool`   | If `true`, publishes additional debug information, including visualization markers on the `~/debug/marker` topic for tools like RViz. |
| `lanelet_extra_margin`                            | `double` | If `> 0`, expands the lanelet polygons used for overlap checks by this margin (in meters). If `<= 0`, polygon expansion is disabled.  |
| `lanelet_xy_overlap_filter.enabled`               | `bool`   | If `true`, enables filtering of objects based on their overlap with lanelet polygons.                                                 |
| `lanelet_direction_filter.enabled`                | `bool`   | If `true`, enables filtering of objects based on their velocity direction relative to the lanelet.                                    |
| `lanelet_direction_filter.velocity_yaw_threshold` | `double` | The yaw angle difference threshold (in radians) between the object’s velocity vector and the lanelet direction.                       |
| `lanelet_direction_filter.object_speed_threshold` | `double` | The minimum speed (in m/s) of an object required for the direction filter to be applied.                                              |
| `lanelet_object_elevation_filter.enabled`         | `bool`   | If `true`, enables filtering of objects based on their elevation relative to the nearest lanelet surface.                             |
| `max_elevation_threshold`                         | `double` | The maximum allowable elevation (in meters) of an object relative to the nearest lanelet surface.                                     |
| `min_elevation_threshold`                         | `double` | The minimum allowable elevation (in meters) of an object relative to the nearest lanelet surface.                                     |
| `lanelet_extra_margin`                            | `double` | The margin value that will be added to the lanelet boundaries.                                                                        |

### Core Parameters

| Name                             | Type | Default Value | Description                               |
| -------------------------------- | ---- | ------------- | ----------------------------------------- |
| `filter_target_label.UNKNOWN`    | bool | false         | If true, unknown objects are filtered.    |
| `filter_target_label.CAR`        | bool | false         | If true, car objects are filtered.        |
| `filter_target_label.TRUCK`      | bool | false         | If true, truck objects are filtered.      |
| `filter_target_label.BUS`        | bool | false         | If true, bus objects are filtered.        |
| `filter_target_label.TRAILER`    | bool | false         | If true, trailer objects are filtered.    |
| `filter_target_label.MOTORCYCLE` | bool | false         | If true, motorcycle objects are filtered. |
| `filter_target_label.BICYCLE`    | bool | false         | If true, bicycle objects are filtered.    |
| `filter_target_label.PEDESTRIAN` | bool | false         | If true, pedestrian objects are filtered. |

## Assumptions / Known limits

The lanelet filter is performed based on the shape polygon and bounding box of the objects.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
