# tier4_perception_rviz_plugin

## Purpose

It is an rviz plugin for visualizing the result from tier4 perception module. This package is based on the implementation of the rviz plugin developed by Autoware.Auto.

See Autoware.Auto design documentation for the original design philosophy. [[1]](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/visualization/autoware_rviz_plugins)

<!-- Write the purpose of this package and briefly describe the features.

Example:
  {package_name} is a package for planning trajectories that can avoid obstacles.
  This feature consists of two steps: obstacle filtering and optimizing trajectory.
-->

## Input Types / Visualization Results

### DetectedObjectsWithFeature

#### Input Types

| Name | Type                                                     | Description            |
| ---- | -------------------------------------------------------- | ---------------------- |
|      | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | detection result array |

#### Visualization Result

![detected-object-with_feature-visualization-description](./images/detected-object-with-feature-visualization-description.jpg)

## References/External links

[1] <https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/tools/visualization/autoware_rviz_plugins>

## Future extensions / Unimplemented parts
