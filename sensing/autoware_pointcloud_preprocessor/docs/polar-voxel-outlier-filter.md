# Polar Voxel Outlier Filter

## Overview

The Polar Voxel Outlier Filter is a point cloud outlier filtering algorithm that operates in polar coordinate space instead of Cartesian coordinate space. This filter is designed for LiDAR data processing, offering two-criteria filtering with return type classification for enhanced noise removal compared to simple occupancy-based methods.

**Key Features**:

- **Automatic format detection** and handling of both `PointXYZ` and `PointXYZIRCAEDT` point cloud formats
- **Two-criteria filtering** for PointXYZIRCAEDT with return type classification
- **Diagnostics** with filter ratio and visibility metrics
- **Debug support** with optional noise point cloud publishing for analysis

## Purpose

The purpose is to remove point cloud noise such as insects and rain using a polar coordinate voxel grid approach that is optimized for LiDAR sensor characteristics. This filter provides a two-criteria filtering method for PointXYZIRCAEDT format with return type classification, which gives additional configuration flexibility when using dual return modes and also enables visibility estimation. The concept is that when two returns exist, the first return is more likely to represent a region of noise (rain, fog, smoke, and so on).

## Key Differences from Cartesian Voxel Grid Filter

### Coordinate System

- **Cartesian Voxel Grid**: Divides 3D space into regular cubic voxels using (x, y, z) coordinates
- **Polar Voxel Grid**: Divides 3D space into polar voxels using (radius, azimuth, elevation) coordinates

### Advantages of Polar Voxelization

1. **Natural LiDAR Representation**: LiDAR sensors naturally scan in polar patterns, making polar voxels more aligned with the data structure
2. **Adaptive Resolution**: Automatically provides higher angular resolution at closer distances and lower resolution at far distances
3. **Range-Aware Filtering**: Can apply different filtering strategies based on distance from sensor
4. **Azimuthal Uniformity**: Maintains consistent azimuthal coverage regardless of distance
5. **Automatic Format Detection**: Handles both PointXYZ and PointXYZIRCAEDT formats

## Point Cloud Format Support

### PointXYZ Format

- **Usage**: Standard PCL point format with (x, y, z) coordinates
- **Processing**: Computes polar coordinates (radius, azimuth, elevation) from Cartesian coordinates
- **Filtering**: Simple occupancy threshold - voxels with ≥ `voxel_points_threshold` points are kept
- **Performance**: Good performance with coordinate conversion overhead

### PointXYZIRCAEDT Format

- **Usage**: Point clouds with pre-computed polar coordinate fields (`distance`, `azimuth`, `elevation`, `return_type`)
- **Detection**: Automatically detects when these fields are present in the point cloud
- **Processing**: Uses pre-computed polar coordinates directly, no conversion needed
- **Advanced Filtering**: Two-criteria filtering with return type classification when enabled:
  1. **Primary Return Threshold**: `primary_returns.size() >= voxel_points_threshold`
  2. **Secondary Noise Threshold**: `secondary_returns.size() <= secondary_noise_threshold`
  3. **Both criteria must be satisfied** for a voxel to be kept
- **Performance**: Faster processing as it avoids trigonometric calculations
- **Compatibility**: Works with any point cloud format that includes the required fields

```yaml
# Example fields that enable optimized processing and advanced filtering:
# - distance      (float32): Pre-computed radius
# - azimuth       (float32): Pre-computed azimuth angle
# - elevation     (float32): Pre-computed elevation angle
# - return_type   (uint8):   Return type classification (primary=[1], secondary=all others)
```

**Note**: The filter automatically detects the presence of polar coordinate fields and return type information, choosing the appropriate processing path. Advanced two-criteria filtering is enabled when `use_return_type_classification=true`.

## Inner-workings / Algorithms

### Coordinate Conversion

**For PointXYZ format:**
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

### Filtering Methodology

The filter uses different algorithms depending on the input point cloud format:

#### For PointXYZ Format

1. **Format Detection**: Automatically detects standard PointXYZ format
2. **Coordinate Conversion**: Computes polar coordinates from Cartesian coordinates
3. **Binning**: Points are grouped into polar voxels
4. **Simple Thresholding**: Voxels with ≥ `voxel_points_threshold` points are kept
5. **Output**: Filtered point cloud with noise points removed

#### For PointXYZIRCAEDT Format

1. **Format Detection**: Automatically detects enhanced format with return type fields
2. **Return Type Classification**: Points are classified as primary or secondary returns
3. **Advanced Two-Criteria Filtering** (when `use_return_type_classification=true`):
   - **Criterion 1**: Primary returns ≥ `voxel_points_threshold`
   - **Criterion 2**: Secondary returns ≤ `secondary_noise_threshold`
   - **Both criteria must be satisfied** for a voxel to be kept
4. **Optional Secondary Filtering**: Can exclude secondary returns from output when `filter_secondary_returns=true`
5. **Output**: Filtered point cloud with enhanced noise removal

### Key Features

- **Polar Coordinate System**: Uses (radius, azimuth, elevation) which naturally align with LiDAR sensor patterns
- **Return Type Classification**: Configurable primary/secondary return handling
- **Secondary Return Filtering**: Optional filtering to keep only primary returns in output
- **Dynamic Threshold Evaluation**: Ensures primary returns dominate over secondary returns
- **Diagnostics**: Filter ratio and visibility metrics with configurable thresholds
- **Debug Support**: Publishes filtered-out points for analysis and tuning when enabled

### Return Type Management

- **Primary Returns**: Typically stronger, more reliable returns (default: [1])
- **Secondary Returns**: All return types not specified as primary (automatically includes [2,3,4,5,etc.])
- **Classification Control**: Can be disabled to process all returns equally
- **Filtering Control**: Can optionally exclude secondary returns from output

Note that primary and secondary returns vary between sensors and return modes, so parametrization should be performed after studying real sensor data.

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Additional Debug Topics

| Name                                                  | Type                                                | Description                                                                    |
| ----------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------ |
| `~/polar_voxel_outlier_filter/debug/filter_ratio`     | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of output to input points                                                |
| `~/polar_voxel_outlier_filter/debug/visibility`       | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of voxels passing secondary return threshold test (PointXYZIRCAEDT only) |
| `~/polar_voxel_outlier_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2`                     | Filtered-out points for debugging (when enabled)                               |

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Filtering Parameters

| Parameter                  | Type   | Description                                 | Default       |
| -------------------------- | ------ | ------------------------------------------- | ------------- |
| `radial_resolution_m`      | double | Resolution in radial direction (meters)     | 0.2           |
| `azimuth_resolution_rad`   | double | Resolution in azimuth direction (radians)   | 0.025 (~1.4°) |
| `elevation_resolution_rad` | double | Resolution in elevation direction (radians) | 0.05 (~2.9°)  |
| `voxel_points_threshold`   | int    | Minimum primary points required per voxel   | 2             |
| `min_radius_m`             | double | Minimum radius to consider (meters)         | 0.5           |
| `max_radius_m`             | double | Maximum radius to consider (meters)         | 300.0         |

### Advanced Filtering Parameters

| Parameter                        | Type  | Description                                 | Default  |
| -------------------------------- | ----- | ------------------------------------------- | -------- |
| `use_return_type_classification` | bool  | Enable advanced two-criteria filtering      | true     |
| `filter_secondary_returns`       | bool  | Keep only primary returns in output         | false    |
| `secondary_noise_threshold`      | int   | Maximum secondary returns allowed per voxel | 4        |
| `primary_return_types`           | int[] | Return types considered as primary returns  | [1,6,10] |

### Performance and Debug Control

| Parameter             | Type | Description                                                         | Default |
| --------------------- | ---- | ------------------------------------------------------------------- | ------- |
| `publish_noise_cloud` | bool | Generate and publish noise cloud for debugging (performance impact) | false   |

### Diagnostics Parameters

| Parameter                      | Type   | Description                             | Default |
| ------------------------------ | ------ | --------------------------------------- | ------- |
| `filter_ratio_error_threshold` | double | Error threshold for filter ratio        | 0.7     |
| `filter_ratio_warn_threshold`  | double | Warning threshold for filter ratio      | 0.3     |
| `visibility_error_threshold`   | double | Error threshold for visibility metric   | 0.5     |
| `visibility_warn_threshold`    | double | Warning threshold for visibility metric | 0.7     |

### Core Parameters (Schema-based)

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_outlier_filter_node.schema.json") }}

### Parameter Interactions

- **use_return_type_classification**: Must be `true` to enable advanced two-criteria filtering
- **filter_secondary_returns**: When `true`, only primary returns appear in output (regardless of filtering criteria)
- **secondary_noise_threshold**: Higher values require stronger primary return dominance
- **primary_return_types**: Only these return types are considered primary; all others are automatically secondary
- **publish_noise_cloud**: When `false`, improves performance by skipping noise cloud generation
- **Diagnostics**: Visibility is only published when return type classification is enabled

## Assumptions / Known limits

- Designed for LiDAR point clouds with polar coordinate characteristics
- Advanced filtering requires PointXYZIRCAEDT format with `use_return_type_classification=true`
- PointXYZ format uses simple occupancy filtering only
- Requires finite coordinate values (filters out NaN/Inf points automatically)
- Two-criteria filtering: Both primary return threshold AND secondary threshold evaluation must be satisfied
- Edge case: Voxels with zero secondary returns pass threshold criterion if they have primary returns

## Error detection and handling

The filter includes robust error handling:

- Input validation for null point clouds
- Automatic filtering of invalid points (NaN, Inf values)
- Graceful handling of points outside configured radius ranges
- Support for both point cloud formats with automatic detection
- Zero division protection in threshold calculations
- Dynamic parameter validation and updates

## Usage

### Launch the Filter

```bash
# Basic launch using the filter directly in a launch file
# (Note: This filter is typically used as part of a larger pointcloud preprocessing pipeline)

# Example launch file integration:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="radial_resolution_m" value="1.0"/>
#   <param name="azimuth_resolution_rad" value="0.0349"/>
#   <param name="voxel_points_threshold" value="3"/>
#   <param name="use_return_type_classification" value="true"/>
#   <param name="secondary_noise_threshold" value="2"/>
#   <param name="primary_return_types" value="[1]"/>
# </node>
```

### ROS 2 Topics

#### Input/Output

- **Input**: `/input` (sensor_msgs/PointCloud2)
- **Output**: `/output` (sensor_msgs/PointCloud2)

#### Debug Topics

- **Filter Ratio**: `~/polar_voxel_outlier_filter/debug/filter_ratio` (autoware_internal_debug_msgs/Float32Stamped)
- **Visibility**: `~/polar_voxel_outlier_filter/debug/visibility` (autoware_internal_debug_msgs/Float32Stamped)
- **Noise Cloud**: `~/polar_voxel_outlier_filter/debug/pointcloud_noise` (sensor_msgs/PointCloud2)

### Programmatic Usage

```cpp
#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

// Create node
auto node = std::make_shared<autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent>(options);

// The filter automatically detects point cloud format and applies appropriate processing:
// - PointXYZ: Simple occupancy filtering with coordinate conversion
// - PointXYZIRCAEDT: Advanced two-criteria filtering with return type classification
//   when use_return_type_classification=true
```

## Performance characterization

### Computational Complexity

- **Time Complexity**: O(n) where n is the number of input points
- **Space Complexity**: O(v) where v is the number of occupied voxels

### Performance Impact by Format

- **PointXYZIRCAEDT with Return Type Classification**:
  - Uses pre-computed polar coordinates and return type information
  - Advanced two-criteria filtering for enhanced noise removal
  - Diagnostics including visibility metrics
  - Reduced computational overhead compared to PointXYZ due to no coordinate conversion
- **PointXYZ**: Standard performance with basic occupancy filtering
  - Computes polar coordinates on-the-fly
  - Additional computational cost for `sqrt`, `atan2` operations
  - Simple threshold-based filtering only

### Memory Usage

- Uses hash maps for voxel storage, automatically handling sparse voxel occupancy
- Memory scales with the number of occupied voxels, not the total possible voxel space
- Single-pass processing: Visibility statistics collected during main filtering loop
- Efficient diagnostics: Real-time performance monitoring with minimal overhead

### Optimization Tips

1. **Use PointXYZIRCAEDT format** with return type classification for better performance and filtering quality
2. **Enable return type classification** (`use_return_type_classification=true`) for advanced filtering
3. **Tune voxel resolutions** based on your use case:
   - Larger resolutions = faster processing, less precise filtering
   - Smaller resolutions = slower processing, more precise filtering
4. **Adjust secondary noise threshold** to control filtering aggressiveness
5. **Configure return type mappings** to match your sensor's characteristics
6. **Monitor diagnostics** for real-time performance assessment
7. **Disable noise cloud publishing** (`publish_noise_cloud=false`) in production for better performance

## Diagnostics and Monitoring

### Filter Ratio Diagnostics

- Published for both input formats
- Represents overall filtering effectiveness (output/input ratio)
- Configurable error/warning thresholds

### Visibility Diagnostics

- Only published for PointXYZIRCAEDT with return type classification enabled
- Represents the percentage of voxels that pass the primary-to-secondary threshold test
- Statistics collected during main filtering loop for efficiency
- Useful for detecting sensor blockage, environmental conditions, or filtering effectiveness

### Debug Features

- **Noise point cloud**: Contains all filtered-out points for analysis (when enabled)
- **Runtime parameter updates**: All thresholds can be adjusted dynamically
- **Detailed logging**: Debug messages show voxel statistics and filtering results

## Implementation Files

### Core Implementation

- `include/autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp`
- `src/outlier_filter/polar_voxel_outlier_filter_node.cpp`

### Configuration

- `config/polar_voxel_outlier_filter_node.param.yaml`
- `schema/polar_voxel_outlier_filter_node.schema.json`

### Documentation

- `docs/polar-voxel-outlier-filter.md` (this document)

## Use Cases and Configuration Guidelines

### Typical Applications

1. **LiDAR Outlier Removal**: Noise removal using return type analysis
2. **Multi-Return Processing**: Separate handling of primary and secondary returns
3. **Dynamic Object Filtering**: Enhanced filtering of isolated points from moving objects
4. **Range-Based Processing**: Different filtering strategies for near vs far points
5. **Sensor Quality Assessment**: Real-time monitoring of sensor performance and environmental conditions
6. **Noise Characterization**: Debug analysis of filtered points for sensor tuning

### Parameter Tuning Guidelines

#### Basic Configuration

- **For dense urban environments**: Use smaller `radial_resolution_m` (0.2-0.5m) and `azimuth_resolution_rad` (0.5-1°)
- **For highway scenarios**: Use larger `radial_resolution_m` (0.5-1.0m) and moderate `azimuth_resolution_rad` (1-2°)
- **For noisy sensors**: Increase `voxel_points_threshold` (3-5 points)
- **For sparse data**: Decrease `voxel_points_threshold` (1-2 points)

#### Advanced Configuration

- **For aggressive noise removal**: Lower `secondary_noise_threshold` (0-1)
- **For conservative filtering**: Higher `secondary_noise_threshold` (2-4)
- **For primary-only output**: Enable `filter_secondary_returns`
- **For custom return classification**: Adjust `primary_return_types` list (all others will be considered secondary returns)
- **For diagnostics**: Tune error/warning thresholds based on expected performance

## Comparison with Cartesian Voxel Filter

| Aspect                    | Cartesian Voxel  | Polar Voxel (PointXYZ) | Polar Voxel (PointXYZIRCAEDT) |
| ------------------------- | ---------------- | ---------------------- | ----------------------------- |
| **Coordinate System**     | (x, y, z)        | (r, θ, φ) computed     | (r, θ, φ) pre-computed        |
| **Voxel Shape**           | Cubic            | Wedge-shaped           | Wedge-shaped                  |
| **Resolution**            | Uniform          | Adaptive               | Adaptive                      |
| **LiDAR Alignment**       | Limited          | Good                   | Good                          |
| **Filtering Method**      | Simple threshold | Simple threshold       | Advanced two-criteria         |
| **Return Type Support**   | None             | None                   | Full classification           |
| **Range Handling**        | Equal treatment  | Range-aware            | Range-aware                   |
| **Computational Cost**    | Low              | Moderate               | Low                           |
| **Coordinate Conversion** | None             | Required               | Not Required                  |
| **Diagnostics**           | Basic            | Basic                  | Comprehensive                 |
| **Debug Support**         | Limited          | Limited                | Full noise analysis           |

## Future extensions

### Potential Enhancements

1. **Cylindrical Mode**: Option to use (r, θ, z) instead of (r, θ, φ) for ground-based applications
2. **Adaptive Thresholding**: Range-dependent point thresholds based on distance from sensor
3. **Multi-Resolution**: Different resolutions for different range bands
4. **GPU Acceleration**: CUDA implementation for high-throughput processing
5. **Machine Learning Integration**: AI-based return type classification and adaptive filtering
6. **Multi-Sensor Fusion**: Support for combining multiple LiDAR sensors
7. **Real-time Parameter Optimization**: Automatic parameter tuning based on environmental conditions

### Recently Implemented

- Separate handling of primary and secondary returns for improved accuracy ✅
- Performance optimization with conditional noise cloud publishing ✅
- Comprehensive parameter validation and runtime configuration ✅
