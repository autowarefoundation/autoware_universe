# autoware_object_sorter

This package contains a object filter module for [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.idl) and [autoware_perception_msgs/msg/TrackedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/TrackedObject.idl).

This package can filter the objects base on velocity and range.

## Interface

### Input

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg` or `autoware_perception_msgs/msg/TrackedObjects.msg`)
  - 3D detected objects

### Output

- `~/output/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg` or `autoware_perception_msgs/msg/TrackedObjects.msg`)

### Parameters

{{ json_to_markdown("perception/autoware_object_sorter/schema/object_sorter.schema.json") }}
