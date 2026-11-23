# autoware_geo_pose_projector

## Overview

This node is a simple node that subscribes to the geo-referenced pose topic and publishes the pose in the map frame.

## Subscribed Topics

| Name                      | Type                                                 | Description         |
| ------------------------- | ---------------------------------------------------- | ------------------- |
| `input_geo_pose`          | `geographic_msgs::msg::GeoPoseWithCovarianceStamped` | geo-referenced pose |
| `/map/map_projector_info` | `autoware_map_msgs::msg::MapProjectedObjectInfo`     | map projector info  |

## Published Topics

| Name          | Type                                            | Description                           |
| ------------- | ----------------------------------------------- | ------------------------------------- |
| `output_pose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | pose in map frame                     |
| `/tf`         | `tf2_msgs::msg::TFMessage`                      | tf from parent link to the child link |

## Parameters

{{ json_to_markdown("localization/autoware_geo_pose_projector/schema/geo_pose_projector.schema.json") }}

## Limitations

The covariance of the input topic is expressed in (Latitude, Longitude, Altitude, rotation about East, rotation about North, rotation about Up) order. The node reorders the full matrix into the map frame (x = East, y = North, z = Up), so cross-covariance terms move with their axes.
The rotation between true North and the grid North of the map projection (meridian convergence) is not applied. For projection types whose map axes are not aligned with East and North, the converted covariance can be off, especially when the latitude and longitude variances differ.
