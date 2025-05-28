# RANSAC Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

Apply the input points to the plane, and set the points at a certain distance from the plane as points other than the ground. Normally, whn using this method, the input points is filtered so that it is almost flat before use. Since the drivable area is often flat, there are methods such as filtering by lane.

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

{{ json_to_markdown("perception/autoware_ground_segmentation/schema/ransac_ground_filter.schema.json") }}

## Assumptions / Known limits

- This method can't handle slopes.
- The input points is filtered so that it is almost flat.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## References/External links

<https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html>

## (Optional) Future extensions / Unimplemented parts
