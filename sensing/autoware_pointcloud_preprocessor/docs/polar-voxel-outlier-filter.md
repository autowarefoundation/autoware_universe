# polar_voxel_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain using a polar coordinate voxel grid approach that is optimized for LiDAR sensor characteristics.

## Inner-workings / Algorithms

Removing point cloud noise based on the number of points existing within polar coordinate voxels (radius, azimuth, elevation). Unlike traditional Cartesian voxel grids, this filter uses polar coordinates which naturally align with LiDAR sensor data patterns, providing more accurate filtering for rotating LiDAR systems.

The filter supports two input formats:
- **Standard PointXYZ**: Computes polar coordinates from Cartesian coordinates
- **PointXYZIRCAEDT**: Uses pre-computed polar coordinates for optimal performance

Key features:
- Optional return type classification (can be disabled to process all returns)
- Separate handling of primary and secondary returns for improved accuracy
- Configurable polar voxel resolution (radius, azimuth, elevation)
- Radius-based filtering to focus on relevant distance ranges
- Configurable return type classification (primary vs secondary returns)

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_outlier_filter_node.schema.json") }}

## Assumptions / Known limits

- Designed primarily for LiDAR point clouds with polar coordinate characteristics
- Performance is optimal when using PointXYZIRCAEDT format with pre-computed polar coordinates
- Requires finite coordinate values (filters out NaN/Inf points automatically)

## (Optional) Error detection and handling

The filter includes robust error handling:
- Input validation for null point clouds
- Automatic filtering of invalid points (NaN, Inf values)
- Graceful handling of points outside configured radius ranges
- Support for both point cloud formats with automatic detection

## (Optional) Performance characterization

- **PointXYZIRCAEDT format**: Optimal performance using pre-computed polar coordinates
- **PointXYZ format**: Additional computational overhead for coordinate conversion
- Memory efficient with point index mapping
- Separate processing of primary/secondary returns for improved accuracy

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- Potential integration with adaptive voxel sizing based on distance
- Separate handling of primary and secondary returns for improved accuracy
- Dynamic parameter adjustment based on environmental conditions
