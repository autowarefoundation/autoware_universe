# autoware_object_velocity_splitter

This package contains a object filter module for [autoware_perception_msgs/msg/DetectedObject](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_perception_msgs/msg/DetectedObject.idl).
This package can split DetectedObjects into two messages by object's speed.

## Interface

### Input

- `~/input/objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - 3D detected objects

### Output

- `~/output/low_speed_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - Objects with low speed
- `~/output/high_speed_objects` (`autoware_perception_msgs/msg/DetectedObjects.msg`)
  - Objects with high speed

### Parameters

{{ json_to_markdown("perception/autoware_object_velocity_splitter/schema/object_velocity_splitter.schema.json") }}
