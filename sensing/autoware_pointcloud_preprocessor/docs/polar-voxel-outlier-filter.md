# Polar Voxel Outlier Filter

## Overview

The Polar Voxel Outlier Filter is a point cloud outlier filtering algorithm that operates in polar coordinate space for LiDAR data processing. This filter supports both simple occupancy-based filtering and advanced two-criteria filtering with return type classification for enhanced noise removal.

**Key Features**:

- **Flexible filtering modes** with configurable return type classification
- **Automatic format detection** between PointXYZIRC and PointXYZIRCAEDT
- **Two-criteria filtering** using primary and secondary return analysis (when enabled)
- **Range-aware visibility estimation** for improved diagnostic accuracy
- **Comprehensive diagnostics** with filter ratio and visibility metrics
- **Optional debug support** with noise point cloud publishing for analysis

## Purpose

The purpose is to remove point cloud noise such as insects and rain using a polar coordinate voxel grid approach optimized for LiDAR sensor characteristics. This filter provides configurable filtering methods:

1. **Simple Mode**: Basic occupancy filtering (any return type counts equally)
2. **Advanced Mode**: Two-criteria filtering with return type classification for enhanced accuracy

The advanced mode concept is that when two returns exist, the first return is more likely to represent a region of noise (rain, fog, smoke, and so on).

## Key Differences from Cartesian Voxel Grid Filter

### Coordinate System

- **Cartesian Voxel Grid**: Divides 3D space into regular cubic voxels using (x, y, z) coordinates
- **Polar Voxel Grid**: Divides 3D space into polar voxels using (radius, azimuth, elevation) coordinates

### Advantages of Polar Voxelization

1. **Natural LiDAR Representation**: LiDAR sensors naturally scan in polar patterns, making polar voxels more aligned with the data structure
2. **Adaptive Resolution**: Automatically provides higher angular resolution at closer distances and lower resolution at far distances
3. **Range-Aware Filtering**: Can apply different filtering strategies based on distance from sensor
4. **Range-Aware Visibility**: Configurable range limit for accurate visibility estimation
5. **Azimuthal Uniformity**: Maintains consistent azimuthal coverage regardless of distance
6. **Configurable Return Type Classification**: Optional return type analysis for enhanced filtering

## Point Cloud Format Support

This filter supports point clouds with return type information (required for advanced mode) and automatically detects between two formats:

### PointXYZIRC Format

- **Usage**: Point format with (x, y, z, intensity, return_type, channel) fields
- **Processing**: Computes polar coordinates (radius, azimuth, elevation) from Cartesian coordinates
- **Return Type**: Uses return_type field for classification (when enabled)
- **Performance**: Good performance with coordinate conversion overhead

### PointXYZIRCAEDT Format

- **Usage**: Point clouds with pre-computed polar coordinate fields and return type
- **Fields**: (x, y, z, intensity, return_type, channel, azimuth, elevation, distance, time_stamp)
- **Detection**: Automatically detects when polar coordinate fields are present
- **Processing**: Uses pre-computed polar coordinates directly, no conversion needed
- **Performance**: Faster processing as it avoids trigonometric calculations

```yaml
# PointXYZIRC: Computes polar coordinates from Cartesian
# - x, y, z       (float32): Cartesian coordinates
# - intensity     (float32): Point intensity
# - return_type   (uint8):   Return type classification
# - channel       (uint16):  Channel information

# PointXYZIRCAEDT: Uses pre-computed polar coordinates
# - x, y, z       (float32): Cartesian coordinates
# - intensity     (float32): Point intensity
# - return_type   (uint8):   Return type classification
# - channel       (uint16):  Channel information
# - azimuth       (float32): Pre-computed azimuth angle
# - elevation     (float32): Pre-computed elevation angle
# - distance      (float32): Pre-computed radius
# - time_stamp    (uint32):  Point timestamp
```

**Note**: The filter automatically detects the format and uses the appropriate processing path. Return type classification can be enabled or disabled based on requirements.

## Inner-workings / Algorithms

### Coordinate Conversion

**For PointXYZIRC format:**
Each point (x, y, z) is converted to polar coordinates:

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

- **Radius Index**: `floor(radius / radial_resolution_m)`
- **Azimuth Index**: `floor(azimuth / azimuth_resolution_rad)`
- **Elevation Index**: `floor(elevation / elevation_resolution_rad)`

### Return Type Classification

When `use_return_type_classification=true`, points are classified using the `return_type` field:

- **Primary Returns**: Return types specified in `primary_return_types` parameter (default: [1,6,10])
- **Secondary Returns**: All other return types not specified as primary
- **Classification**: Used for advanced two-criteria filtering

### Range-Aware Visibility Estimation

The visibility metric is calculated only for voxels within the configured range:

- **Range Filtering**: Only voxels with maximum radius ≤ `visibility_estimation_max_range_m` are considered
- **Visibility Estimation Tuning**: Reported visibility value is tuned using the `visibility_estimation_max_secondary_voxel_count` parameter
- **Reliability**: Excludes potentially unreliable distant measurements from visibility calculations
- **Configurable**: Allows adjustment based on sensor characteristics and requirements

### Filtering Methodology

The filter uses different algorithms based on the `use_return_type_classification` parameter:

#### Simple Mode (`use_return_type_classification=false`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Coordinate Processing**:
   - PointXYZIRC: Computes polar coordinates from Cartesian
   - PointXYZIRCAEDT: Uses pre-computed polar coordinates
3. **Voxel Binning**: Points are grouped into polar voxels
4. **Simple Thresholding**: Voxels with ≥ `voxel_points_threshold` points (any return type) are kept
5. **Output**: Filtered point cloud with basic noise removal

#### Advanced Mode (`use_return_type_classification=true`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Return Type Validation**: Ensures return_type field is present
3. **Coordinate Processing**:
   - PointXYZIRC: Computes polar coordinates from Cartesian
   - PointXYZIRCAEDT: Uses pre-computed polar coordinates
4. **Return Type Classification**: Points are classified as primary or secondary returns
5. **Two-Criteria Filtering**:
   - **Criterion 1**: Primary returns ≥ `voxel_points_threshold`
   - **Criterion 2**: Secondary returns ≤ `secondary_noise_threshold`
   - **Both criteria must be satisfied** for a voxel to be kept
6. **Range-Aware Visibility**: Visibility calculation limited to voxels within `visibility_estimation_max_range_m` and based on the number of voxels that fail criterion 2, bounded by `visibility_estimation_max_secondary_voxel_count`
7. **Secondary Return Filtering**: Optional exclusion of secondary returns from output
8. **Output**: Filtered point cloud with enhanced noise removal

### Advanced Two-Criteria Filtering

When enabled, for each voxel both criteria must be satisfied:

- **Primary Return Threshold**: `primary_count >= voxel_points_threshold`
- **Secondary Return Threshold**: `secondary_count <= secondary_noise_threshold`
- **Final Decision**: `valid_voxel = (primary_threshold_met AND secondary_threshold_met)`

### Key Features

- **Flexible Architecture**: Configurable between simple and advanced filtering
- **Format-Optimized Processing**: Automatic selection of optimal coordinate source
- **Range-Aware Diagnostics**: Visibility estimation limited to reliable sensor range
- **Secondary Voxel Limiting**: Configurable maximum on secondary voxels for visibility estimation
- **Comprehensive Diagnostics**: Mode-specific filter ratio and visibility metrics
- **Debug Support**: Optional noise cloud publishing for analysis and tuning

### Return Type Management (Advanced Mode Only)

- **Primary Returns**: Configurable list of return types (default: [1,6,10])
- **Secondary Returns**: All return types not specified as primary
- **Dynamic Classification**: Runtime configurable through parameter updates
- **Output Filtering**: Optional exclusion of secondary returns from final output

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Input Requirements

- **Supported Formats**: PointXYZIRC or PointXYZIRCAEDT
- **Return Type Field**: Required only when `use_return_type_classification=true`
- **Invalid Inputs**: Point clouds without return_type field will be rejected in advanced mode

### Additional Debug Topics

| Name                                                  | Type                                                | Description                                                                                 |
| ----------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `~/polar_voxel_outlier_filter/debug/filter_ratio`     | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of output to input points                                                             |
| `~/polar_voxel_outlier_filter/debug/visibility`       | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of voxels passing secondary return threshold test (advanced mode only, range-limited) |
| `~/polar_voxel_outlier_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2`                     | Filtered-out points for debugging (when enabled)                                            |

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Filtering Parameters

| Parameter                                         | Type   | Description                                             | Default        |
| ------------------------------------------------- | ------ | ------------------------------------------------------- | -------------- |
| `radial_resolution_m`                             | double | Resolution in radial direction (meters)                 | 0.5            |
| `azimuth_resolution_rad`                          | double | Resolution in azimuth direction (radians)               | 0.0175 (~1.0°) |
| `elevation_resolution_rad`                        | double | Resolution in elevation direction (radians)             | 0.0175 (~1.0°) |
| `voxel_points_threshold`                          | int    | Minimum points required per voxel                       | 2              |
| `min_radius_m`                                    | double | Minimum radius to consider (meters)                     | 0.5            |
| `max_radius_m`                                    | double | Maximum radius to consider (meters)                     | 300.0          |
| `visibility_estimation_max_range_m`               | double | Maximum range for visibility estimation (meters)        | 20.0           |
| `visibility_estimation_max_secondary_voxel_count` | int    | Maximum secondary voxel count for visibility estimation | 500            |

### Return Type Classification Parameters

| Parameter                        | Type  | Description                                 | Default    |
| -------------------------------- | ----- | ------------------------------------------- | ---------- |
| `use_return_type_classification` | bool  | Enable advanced two-criteria filtering      | true       |
| `filter_secondary_returns`       | bool  | Keep only primary returns in output         | false      |
| `secondary_noise_threshold`      | int   | Maximum secondary returns allowed per voxel | 4          |
| `primary_return_types`           | int[] | Return types considered as primary returns  | [1,6,8,10] |

### Performance and Debug Control

| Parameter             | Type | Description                                                         | Default |
| --------------------- | ---- | ------------------------------------------------------------------- | ------- |
| `publish_noise_cloud` | bool | Generate and publish noise cloud for debugging (performance impact) | true    |

### Diagnostics Parameters

| Parameter                      | Type   | Description                             | Default |
| ------------------------------ | ------ | --------------------------------------- | ------- |
| `filter_ratio_error_threshold` | double | Error threshold for filter ratio        | 0.5     |
| `filter_ratio_warn_threshold`  | double | Warning threshold for filter ratio      | 0.7     |
| `visibility_error_threshold`   | double | Error threshold for visibility metric   | 0.8     |
| `visibility_warn_threshold`    | double | Warning threshold for visibility metric | 0.9     |

### Parameter Interactions

- **use_return_type_classification**: Must be `true` to enable advanced two-criteria filtering
- **filter_secondary_returns**: When `true`, only primary returns appear in output (advanced mode only)
- **secondary_noise_threshold**: Only used when `use_return_type_classification=true`
- **visibility_estimation_max_secondary_voxel_count**: Only used when `use_return_type_classification=true`, limits secondary voxel counting in visibility calculations
- **primary_return_types**: Only used when `use_return_type_classification=true`
- **visibility_estimation_max_range_m**: Limits visibility calculation to reliable sensor range (advanced mode only)
- **publish_noise_cloud**: When `false`, improves performance by skipping noise cloud generation
- **Diagnostics**: Visibility is only published when return type classification is enabled

## Configuration Examples

### Simple Mode Configuration

```yaml
# Basic occupancy filtering - any return type counts equally
use_return_type_classification: false
voxel_points_threshold: 2 # Total points threshold
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0349 # ~2 degrees
elevation_resolution_rad: 0.0349 # ~2 degrees
publish_noise_cloud: false # Performance optimization
```

### Advanced Mode Configuration

```yaml
# Two-criteria filtering with return type classification
use_return_type_classification: true
voxel_points_threshold: 2 # Primary return threshold
secondary_noise_threshold: 4 # Secondary return threshold
visibility_estimation_max_secondary_voxel_count: 0 # Max secondary voxels for visibility
primary_return_types: [1, 6, 10] # Primary return types
filter_secondary_returns: false # Include secondary returns in output
radial_resolution_m: 0.2
azimuth_resolution_rad: 0.025 # ~1.4 degrees
elevation_resolution_rad: 0.05 # ~2.9 degrees
visibility_estimation_max_range_m: 100.0 # Range limit for visibility calculation
publish_noise_cloud: false
```

### Debug Configuration

```yaml
# Enable debugging and monitoring
use_return_type_classification: true
visibility_estimation_max_range_m: 80.0 # Shorter range for urban environments
visibility_estimation_max_secondary_voxel_count: 10 # Allow some secondary voxels in visibility calculation
publish_noise_cloud: true # Enable noise cloud for analysis
filter_ratio_error_threshold: 0.3
filter_ratio_warn_threshold: 0.5
visibility_error_threshold: 0.4
visibility_warn_threshold: 0.6
```

### Sensor-Specific Configuration Examples

```yaml
# Long-range highway LiDAR
visibility_estimation_max_range_m: 200.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
voxel_points_threshold: 2

# Urban short-range LiDAR
visibility_estimation_max_range_m: 20.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
voxel_points_threshold: 2

# High-resolution near-field processing
visibility_estimation_max_range_m: 20.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0175 # ~1 degree
elevation_resolution_rad: 0.0175 # ~1 degree
```

## Assumptions / Known limits

- **Simple mode**: Works with any point cloud format, basic occupancy filtering only
- **Advanced mode**: Requires return_type field for enhanced filtering
- **Supported formats**: PointXYZIRC and PointXYZIRCAEDT only
- **Finite coordinates required**: Automatically filters out NaN/Inf points
- **Return type dependency**: Advanced filtering effectiveness depends on accurate return type classification
- **Visibility range dependency**: Visibility accuracy depends on appropriate `visibility_estimation_max_range_m` setting
- **Secondary voxel limiting**: Visibility estimation can be tuned via `visibility_estimation_max_secondary_voxel_count`

## Error detection and handling

The filter includes robust error handling:

- **Mode-specific validation**: Checks return_type field presence in advanced mode
- **Input validation**: Checks for null point clouds
- **Coordinate validation**: Filters invalid points (NaN, Inf values) automatically
- **Range validation**: Points outside configured radius ranges are excluded
- **Parameter validation**: Ensures `visibility_estimation_max_range_m` > 0 and `visibility_estimation_max_secondary_voxel_count` ≥ 0
- **Dynamic parameter validation**: Runtime parameter updates with validation

## Usage

### Launch the Filter

```bash
# Example launch file integration for simple mode:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="use_return_type_classification" value="false"/>
#   <param name="radial_resolution_m" value="1.0"/>
#   <param name="azimuth_resolution_rad" value="0.0349"/>
#   <param name="voxel_points_threshold" value="3"/>
# </node>

# Example launch file integration for advanced mode:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="use_return_type_classification" value="true"/>
#   <param name="radial_resolution_m" value="0.5"/>
#   <param name="azimuth_resolution_rad" value="0.025"/>
#   <param name="voxel_points_threshold" value="2"/>
#   <param name="secondary_noise_threshold" value="4"/>
#   <param name="visibility_estimation_max_secondary_voxel_count" value="0"/>
#   <param name="primary_return_types" value="[1,6,10]"/>
#   <param name="visibility_estimation_max_range_m" value="100.0"/>
# </node>
```

### ROS 2 Topics

#### Input/Output

- **Input**: `/input` (sensor_msgs/PointCloud2) - Must have return_type field for advanced mode
- **Output**: `/output` (sensor_msgs/PointCloud2)

#### Debug Topics

- **Filter Ratio**: `~/polar_voxel_outlier_filter/debug/filter_ratio` (autoware_internal_debug_msgs/Float32Stamped)
- **Visibility**: `~/polar_voxel_outlier_filter/debug/visibility` (autoware_internal_debug_msgs/Float32Stamped) - Advanced mode only, range-limited
- **Noise Cloud**: `~/polar_voxel_outlier_filter/debug/pointcloud_noise` (sensor_msgs/PointCloud2)

### Programmatic Usage

```cpp
#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

// Create node
auto node = std::make_shared<autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent>(options);

// The filter automatically detects point cloud format and applies filtering based on configuration:
// - Simple mode (use_return_type_classification=false): Basic occupancy filtering
// - Advanced mode (use_return_type_classification=true): Two-criteria filtering with return type analysis
// - Both modes support PointXYZIRC and PointXYZIRCAEDT formats
// - Advanced mode uses range-limited visibility estimation with configurable secondary voxel limiting
```

## Performance characterization

### Computational Complexity

- **Time Complexity**: O(n) where n is the number of input points
- **Space Complexity**: O(v) where v is the number of occupied voxels

### Performance Impact by Mode and Format

#### **Simple Mode**

- **PointXYZIRCAEDT**: Fast processing with pre-computed coordinates
- **PointXYZIRC**: Good performance with coordinate conversion overhead
- **No return type analysis**: Reduced computational overhead

#### **Advanced Mode**

- **PointXYZIRCAEDT**: Optimal performance with pre-computed coordinates and return type analysis
- **PointXYZIRC**: Good performance with coordinate conversion and return type analysis
- **Enhanced filtering**: Additional return type classification and range-aware visibility processing

### Memory Usage

- **Hash-based voxel storage**: Efficiently handles sparse voxel occupancy
- **Single-pass processing**: Minimal memory overhead
- **Mode-specific diagnostics**: Efficient performance monitoring
- **Range filtering**: Additional hash map for visibility calculation (advanced mode only)

### Optimization Tips

1. **Choose appropriate mode** based on requirements:
   - Simple mode for basic filtering needs
   - Advanced mode for enhanced noise removal
2. **Use PointXYZIRCAEDT format** when available for optimal performance
3. **Tune voxel resolutions** based on your use case
4. **Configure return type mappings** to match your sensor (advanced mode)
5. **Set appropriate visibility range** (`visibility_estimation_max_range_m`) for your sensor and environment
6. **Tune secondary voxel limiting** (`visibility_estimation_max_secondary_voxel_count`) for visibility estimation accuracy
7. **Monitor diagnostics** for real-time performance assessment
8. **Disable noise cloud publishing** in production for better performance

## Diagnostics and Monitoring

### Filter Ratio Diagnostics

- **Published for both modes**: Overall filtering effectiveness (output/input ratio)
- **Configurable thresholds**: Error/warning levels for automated monitoring
- **Real-time feedback**: Immediate filtering performance assessment

### Visibility Diagnostics

- **Advanced mode only**: Uses return type classification data
- **Range-limited metric**: Only considers voxels within `visibility_estimation_max_range_m`
- **Secondary voxel limiting**: Controlled by `visibility_estimation_max_secondary_voxel_count` parameter
- **Voxel-based metric**: Percentage of range-limited voxels passing secondary threshold test
- **Environmental indicator**: Useful for detecting sensor conditions within reliable range
- **Diagnostic context**: Status messages include the configured visibility estimation range and secondary voxel limits

### Debug Features

- **Noise point cloud**: All filtered-out points for analysis (when enabled)
- **Runtime parameter updates**: Dynamic threshold and range adjustment
- **Mode-specific logging**: Debug messages tailored to filtering mode
- **Range-aware diagnostics**: Visibility calculations clearly indicate the estimation range

## Use Cases and Configuration Guidelines

### Simple Mode Use Cases

1. **Legacy System Integration**: Basic filtering without return type requirements
2. **Performance-Critical Applications**: When computational resources are limited
3. **Unknown Return Type Reliability**: When sensor return type information is questionable
4. **Basic Noise Removal**: Simple occupancy-based filtering requirements

### Advanced Mode Use Cases

1. **Modern LiDAR Processing**: Enhanced filtering with reliable return type information
2. **Environmental Monitoring**: Range-aware visibility estimation and weather condition detection
3. **High-Quality Filtering**: Two-criteria approach for superior noise removal
4. **Autonomous Vehicle Applications**: Safety-critical filtering with comprehensive diagnostics
5. **Range-Specific Analysis**: Different visibility requirements for near vs. far field

### Parameter Tuning Guidelines

#### Simple Mode Configuration

- **For dense environments**: Smaller voxel resolutions, higher point thresholds
- **For sparse data**: Larger voxel resolutions, lower point thresholds
- **For performance**: Larger resolutions, disable noise cloud publishing

#### Advanced Mode Configuration

- **For aggressive noise removal**: Lower secondary noise threshold (0-2)
- **For conservative filtering**: Higher secondary noise threshold (3-5)
- **For strict visibility estimation**: Set `visibility_estimation_max_secondary_voxel_count` to a lower number
- **For lenient visibility estimation**: Allow higher secondary voxel counts (several hundred)
- **For primary-only output**: Enable `filter_secondary_returns`
- **For sensor-specific optimization**: Adjust `primary_return_types` based on sensor characteristics
- **For range-specific visibility**: Set `visibility_estimation_max_range_m` based on sensor effective range and application requirements

#### Visibility Range Guidelines

- **Urban environments**: 30-80m (shorter ranges for reliable near-field analysis)
- **Highway applications**: 100-200m (longer ranges for high-speed scenarios)
- **Parking/loading**: 10-30m (very short ranges for precise near-field monitoring)
- **Sensor specifications**: Match to sensor's reliable detection range

## Comparison Table

| Aspect                   | Simple Mode     | Advanced Mode                   |
| ------------------------ | --------------- | ------------------------------- |
| **Filtering Method**     | Basic occupancy | Two-criteria with return type   |
| **Return Type Required** | No              | Yes                             |
| **Computational Cost**   | Low             | Moderate                        |
| **Filtering Quality**    | Good            | Excellent                       |
| **Visibility Metrics**   | None            | Range-aware with voxel limiting |
| **Configuration**        | Simple          | Advanced                        |
| **Use Case**             | Basic filtering | Enhanced noise removal          |
| **Environmental Adapt**  | Limited         | Comprehensive                   |
| **Range Awareness**      | Basic           | Configurable                    |

## Migration Guide

### Enabling Advanced Mode

To enable advanced filtering on existing systems:

1. **Ensure return type field**: Verify input point clouds have return_type field
2. **Set parameter**: `use_return_type_classification: true`
3. **Configure return types**: Set `primary_return_types` for your sensor
4. **Set visibility range**: Configure `visibility_estimation_max_range_m` for your application
5. **Configure secondary voxel limiting**: Set `visibility_estimation_max_secondary_voxel_count` based on requirements
6. **Tune thresholds**: Adjust `secondary_noise_threshold` based on requirements
7. **Monitor diagnostics**: Use range-aware visibility metrics for performance assessment

### Disabling Advanced Mode

To use simple mode for basic filtering:

1. **Set parameter**: `use_return_type_classification: false`
2. **Configure threshold**: Set `voxel_points_threshold` for total point count
3. **Remove advanced parameters**: Return type and visibility range parameters will be ignored
4. **Simplified monitoring**: Only filter ratio diagnostics available
