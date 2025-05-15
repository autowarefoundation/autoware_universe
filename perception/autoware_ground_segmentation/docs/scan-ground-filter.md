# Scan Ground Filter

## Purpose

The purpose of this node is that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

This algorithm works by following steps,

1. Divide whole pointclouds into groups by azimuth angle (so-called ray)
2. Sort points by radial distance (xy-distance), on each ray.
3. Divide pointcloud into grids, on each ray.
4. Classify the point
   1. Check radial distance to previous pointcloud, if the distance is large and previous pointcloud is "no ground" and the height level of current point greater than previous point, the current pointcloud is classified as no ground.
   2. Check vertical angle of the point compared with previous ground grid
   3. Check the height of the point compared with predicted ground level
   4. If vertical angle is greater than local_slope_max and related height to predicted ground level is greater than "non ground height threshold", the point is classified as "non ground"
   5. If the vertical angle is in range of [-local_slope_max, local_slope_max] or related height to predicted ground level is smaller than non_ground_height_threshold, the point is classified as "ground"
   6. If the vertical angle is lower than -local_slope_max or the related height to ground level is greater than detection_range_z_max, the point will be classified as out of range

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

#### Core Parameters

![scan_ground_parameter](./image/scan_ground_filter_parameters.drawio.svg)

{{ json_to_markdown("perception/autoware_ground_segmentation/schema/scan_ground_filter.schema.json") }}

## Assumptions / Known limits

The input_frame is set as parameter but it must be fixed as base_link for the current algorithm.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

<!-- cspell: ignore Shen Liang -->

The elevation grid idea is referred from "Shen Z, Liang H, Lin L, Wang Z, Huang W, Yu J. Fast Ground Segmentation for 3D LiDAR Point Cloud Based on Jump-Convolution-Process. Remote Sensing. 2021; 13(16):3239. <https://doi.org/10.3390/rs13163239>"

## (Optional) Future extensions / Unimplemented parts

- Horizontal check for classification is not implemented yet.
- Output ground visibility for diagnostic is not implemented yet.
