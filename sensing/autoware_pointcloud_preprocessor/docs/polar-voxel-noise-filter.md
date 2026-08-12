# Polar Voxel Noise Filter

## Overview

The Polar Voxel Noise Filter is a point cloud noise filtering algorithm that operates in polar
coordinate space for LiDAR data processing. This filter supports both simple intensity-based noise removal and return-type-aware filtering for more small objects like insects, and other
sparse noise.

**Key Features**:

- **Flexible filtering modes** with optional return type classification
- **Automatic format detection** between PointXYZIRC and PointXYZIRCAEDT
- **Polar voxel filtering** using radius, azimuth, and elevation bins
- **Intensity-aware noise judgement** using average voxel intensity
- **Optional secondary return suppression** in the final output
- **Optional debug support** with noise point cloud publishing for analysis

## Purpose

The purpose is to remove point cloud noise such as insects and voxels with high number of secondary returns using a polar coordinate voxel
grid approach optimized for LiDAR sensor characteristics. This filter provides configurable
filtering methods:

1. **Simple Mode**: Noise judgement using voxel occupancy and average intensity
2. **Return-Type-Aware Mode**: Extends noise judgement with secondary return counting

The return-type-aware mode is intended for sensors where secondary returns are more likely to be
associated with noise such as rain, or other weak reflections.

## Key Differences from Cartesian Voxel Grid Filter

### Coordinate System

- **Cartesian Voxel Grid**: Divides 3D space into regular cubic voxels using `(x, y, z)`
- **Polar Voxel Grid**: Divides 3D space into polar voxels using `(radius, azimuth, elevation)`

### Advantages of Polar Voxelization

1. **Natural LiDAR Representation**: LiDAR sensors scan in polar patterns, so the voxel structure
   matches the data more naturally
2. **Range-Aware Spatial Binning**: Angular bins remain consistent across the scan
3. **Format Flexibility**: Supports both Cartesian input and precomputed polar-coordinate input
4. **Return-Type-Aware Filtering**: Can use primary/secondary return information when available
5. **Debug Visibility**: Can publish the removed points as a separate cloud for tuning

## Point Cloud Format Support

This filter supports point clouds with intensity information and automatically detects between two
formats:

### PointXYZIRC Format

- **Usage**: Point format with `(x, y, z, intensity, return_type, channel)` fields
- **Processing**: Computes polar coordinates `(radius, azimuth, elevation)` from Cartesian values
- **Return Type**: Uses `return_type` for classification when enabled
- **Performance**: Good performance with coordinate conversion overhead

### PointXYZIRCAEDT Format

- **Usage**: Point clouds with pre-computed polar coordinate fields
- **Fields**: `(x, y, z, intensity, return_type, channel, azimuth, elevation, distance, time_stamp)`
- **Detection**: Automatically detects when polar coordinate fields are present
- **Processing**: Uses pre-computed polar coordinates directly
- **Performance**: Faster processing because it avoids trigonometric conversion

```yaml
# PointXYZIRC: Computes polar coordinates from Cartesian
# - x, y, z       (float32): Cartesian coordinates
# - intensity     (uint8/compatible field): Point intensity
# - return_type   (uint8):   Return type classification
# - channel       (uint16):  Channel information

# PointXYZIRCAEDT: Uses pre-computed polar coordinates
# - x, y, z       (float32): Cartesian coordinates
# - intensity     (uint8/compatible field): Point intensity
# - return_type   (uint8):   Return type classification
# - channel       (uint16):  Channel information
# - azimuth       (float32): Pre-computed azimuth angle
# - elevation     (float32): Pre-computed elevation angle
# - distance      (float32): Pre-computed radius
# - time_stamp    (uint32):  Point timestamp
```

**Note**: The filter always requires an `intensity` field. The `return_type` field is required
only when `use_return_type_classification=true`.

## Inner-workings / Algorithms

### Coordinate Conversion

**For PointXYZIRC format:**
Each point `(x, y, z)` is converted to polar coordinates:

- **Radius**: `r = sqrt(x² + y² + z²)`
- **Azimuth**: `θ = atan2(y, x)`
- **Elevation**: `φ = atan2(z, sqrt(x² + y²))`

**For PointXYZIRCAEDT format:**
Uses pre-computed polar coordinates directly from the point fields:

- **Radius**: `r = point.distance`
- **Azimuth**: `θ = point.azimuth`
- **Elevation**: `φ = point.elevation`

### Voxel Index Calculation

Each point is assigned to a voxel based on:

- **Radius Index**: `floor(radius / radial_resolution)`
- **Azimuth Index**: `floor(azimuth / azimuth_resolution)`
- **Elevation Index**: `floor(elevation / elevation_resolution)`

### Return Type Classification

When `use_return_type_classification=true`, points are classified using the `return_type` field:

- **Primary Returns**: Return types specified in `primary_return_types`
- **Secondary Returns**: All other return types not specified as primary
- **Classification Use**: Applied only in the return-type-aware noise judgement path

### Filtering Methodology

The filter uses different algorithms based on the `use_return_type_classification` parameter.

#### Simple Mode (`use_return_type_classification=false`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Coordinate Processing**:
   - PointXYZIRC: Computes polar coordinates from Cartesian
   - PointXYZIRCAEDT: Uses pre-computed polar coordinates
3. **Voxel Binning**: Points are grouped into polar voxels
4. **Voxel Statistics**: For each voxel, compute:
   - total point count
   - average intensity
5. **Noise Judgement**:
   - a voxel is treated as noise when
     `point_count <= voxel_points_threshold AND intensity_avg <= avg_intensity_threshold`
6. **Output**: Points in non-noise voxels are kept

#### Return-Type-Aware Mode (`use_return_type_classification=true`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Return Type Validation**: Ensures `return_type` field is present
3. **Coordinate Processing**:
   - PointXYZIRC: Computes polar coordinates from Cartesian
   - PointXYZIRCAEDT: Uses pre-computed polar coordinates
4. **Return Type Classification**: Points are classified as primary or secondary returns
5. **Voxel Statistics**: For each voxel, compute:
   - total point count
   - average intensity
   - secondary return count
6. **Noise Judgement**:
   - a voxel is treated as noise when matched the next conditions:
     - `point_count <= voxel_points_threshold AND intensity_avg <= avg_intensity_threshold`
     - `secondary_return_count >= secondary_noise_threshold AND intensity_avg <= avg_intensity_threshold`
7. **Optional Secondary Return Filtering**:
   - when `filter_secondary_returns=true`, only primary returns are published even from kept voxels
8. **Output**: Filtered point cloud plus optional debug noise cloud

### Noise Judgement

#### Simple Mode

- **Noise voxel**:
  `point_count <= voxel_points_threshold AND intensity_avg <= avg_intensity_threshold`

#### Return-Type-Aware Mode

- **Noise voxel**:
  `((point_count <= voxel_points_threshold) OR (secondary_return_count >= secondary_noise_threshold)) AND intensity_avg <= avg_intensity_threshold`

### Key Features

- **Flexible Architecture**: Configurable between simple and return-type-aware filtering
- **Format-Optimized Processing**: Automatic selection of optimal coordinate source
- **Intensity-Aware Filtering**: Uses average voxel intensity to avoid removing dense valid returns
- **Secondary Return Handling**: Can use or suppress secondary returns depending on configuration
- **Debug Support**: Optional noise cloud publishing for analysis and tuning

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer
[README](../README.md).

### Input Requirements

- **Supported Formats**: PointXYZIRC or PointXYZIRCAEDT
- **Intensity Field**: Always required
- **Return Type Field**: Required only when `use_return_type_classification=true`
- **Invalid Inputs**: Point clouds without `intensity` are rejected, and point clouds without
  `return_type` are rejected in return-type-aware mode

### Additional Debug Topics

| Name                                                | Type                            | Description                       |
| --------------------------------------------------- | ------------------------------- | --------------------------------- |
| `~/polar_voxel_noise_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2` | Filtered-out points for debugging |

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer
[README](../README.md).

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_noise_filter_node.schema.json") }}

### Parameter Interactions

- **use_return_type_classification**: Enables return-type-aware noise filtering
- **primary_return_types**: Only used when `use_return_type_classification=true`
- **secondary_noise_threshold**: Only used when `use_return_type_classification=true`
- **filter_secondary_returns**: When `true`, only primary returns remain in the output
- **avg_intensity_threshold**: Used in both modes as part of the noise decision
- **publish_noise_cloud**: When `true`, publishes removed points for debugging

## Assumptions / Known limits

- **Simple mode**: Uses point count and average intensity only
- **Return-type-aware mode**: Requires `return_type` field
- **Supported formats**: PointXYZIRC and PointXYZIRCAEDT only
- **Finite coordinates required**: NaN and Inf values are ignored
- **Radius range filtering**: Points outside `[min_radius, max_radius]` are excluded
- **Indices unsupported**: Input indices are ignored
- **Angular domain assumptions**:
  - **PointXYZIRC azimuth**: Computed from `atan2(y, x)`, so the domain is `[-π, π]`; this filter
    does not normalize it to `[0, 2π]` before voxelization
  - **PointXYZIRCAEDT azimuth/elevation**: Uses the polar coordinates provided by the input
    message as-is
  - **Elevation domain**: Expected to be `[-π/2, π/2]`

## Error detection and handling

The filter includes input and parameter validation:

- **Input validation**: Checks for missing required fields
- **Return type validation**: Enforced only in return-type-aware mode
- **Coordinate validation**: Ignores invalid or out-of-range points
- **Dynamic parameter validation**: Rejects invalid runtime updates such as negative radii or
  out-of-range return types

## Usage

### Launch the Filter

```xml
<node pkg="autoware_pointcloud_preprocessor"
      exec="polar_voxel_noise_filter_node"
      name="polar_voxel_noise_filter">
  <param from="$(find-pkg-share autoware_pointcloud_preprocessor)/config/polar_voxel_noise_filter_node.param.yaml"/>
</node>
```

### ROS 2 Topics

- **Input**: inherited from `autoware::pointcloud_preprocessor::Filter`
- **Output**: inherited from `autoware::pointcloud_preprocessor::Filter`
- **Debug Noise Cloud**: `~/polar_voxel_noise_filter/debug/pointcloud_noise`

## Performance characterization

### Computational Complexity

- **Time Complexity**: Approximately `O(n)` where `n` is the number of input points
- **Space Complexity**: Approximately `O(v)` where `v` is the number of occupied voxels

### Performance Impact by Mode

#### Simple Mode

- Lower overhead because no return type classification is used
- Best choice when `return_type` is unavailable or not needed

#### Return-Type-Aware Mode

- Adds return type lookup and secondary return counting
- Provides stronger suppression of sparse secondary-return-heavy noise

#### Point Format Impact

- **PointXYZIRCAEDT**: Faster due to pre-computed polar coordinates
- **PointXYZIRC**: Slightly slower due to per-point coordinate conversion

### Optimization Tips

1. Use `PointXYZIRCAEDT` input when available for better performance
2. Enable `publish_noise_cloud` only when debugging
3. Tune `avg_intensity_threshold` together with `voxel_points_threshold`
4. Use return-type-aware mode only when the sensor provides meaningful `return_type` values
