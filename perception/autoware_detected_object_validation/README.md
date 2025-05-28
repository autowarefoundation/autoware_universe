# detected_object_validation

## Purpose

The purpose of this package is to eliminate obvious false positives of DetectedObjects.

## References/External links

- [Obstacle pointcloud based validator](obstacle-pointcloud-based-validator.md)
- [Occupancy grid based validator](occupancy-grid-based-validator.md)
- [Object lanelet filter](object-lanelet-filter.md)
- [Object position filter](object-position-filter.md)

### Node Parameters

#### object_lanelet_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_lanelet_filter.schema.json") }}

#### object_position_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_position_filter.schema.json") }}

#### obstacle_pointcloud_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/obstacle_pointcloud_based_validator.schema.json") }}

#### occupancy_grid_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/occupancy_grid_based_validator.schema.json") }}
