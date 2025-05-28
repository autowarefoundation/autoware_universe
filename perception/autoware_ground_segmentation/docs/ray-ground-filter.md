# Ray Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

The points is separated radially (Ray), and the ground is classified for each Ray sequentially from the point close to ego-vehicle based on the geometric information such as the distance and angle between the points.

![ray-xy](./image/ground_filter-ray-xy.drawio.svg)

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

![ray-xz](./image/ground_filter-ray-xz.drawio.svg)

{{ json_to_markdown("perception/autoware_ground_segmentation/schema/ray_ground_filter.schema.json") }}

## Assumptions / Known limits

The input_frame is set as parameter but it must be fixed as base_link for the current algorithm.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
