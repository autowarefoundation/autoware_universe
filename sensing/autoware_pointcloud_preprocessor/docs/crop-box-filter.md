# crop_box_filter

## Purpose

The `crop_box_filter` is a node that removes points with in a given box region. This filter is used to remove the points that hit the vehicle itself.

## Inner-workings / Algorithms

`pcl::CropBox` is used, which filters all points inside a given box.

## Inputs / Outputs

This implementation inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/crop_box_filter_node.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
