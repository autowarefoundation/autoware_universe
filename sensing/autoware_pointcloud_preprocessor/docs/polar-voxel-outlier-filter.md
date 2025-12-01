# Polar Voxel Outlier Filter

## Overview

The Polar Voxel Outlier Filter is a point cloud outlier filtering algorithm that operates in polar coordinate space for LiDAR data processing. This filter supports both simple occupancy-based filtering and advanced two-criteria filtering with return type classification for enhanced noise removal.

**Key Features**:

- **Flexible filtering modes** with configurable return type classification
- **Automatic format detection** between PointXYZIRC and PointXYZIRCAEDT
- **Two-criteria filtering** using primary and secondary return analysis (when enabled)
- **Range-aware visibility estimation** for improved diagnostic accuracy
- **Comprehensive diagnostics** with filter ratio and visibility metrics
- **Visibility estimation only mode** for diagnostic-only operation without point cloud output
- **Optional debug support** with noise point cloud publishing for analysis

## Purpose

The purpose is to remove point cloud noise such as insects and rain using a polar coordinate voxel grid approach optimized for LiDAR sensor characteristics. This filter provides configurable filtering methods:

1. **Simple Mode**: Basic occupancy filtering (any return type counts equally)
2. **Advanced Mode**: Two-criteria filtering with return type classification for enhanced accuracy
3. **Visibility Estimation Only Mode**: Diagnostic-only operation for monitoring without data processing overhead

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
7. **Diagnostic-Only Operation**: Visibility estimation without point cloud processing overhead

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

- **Primary Returns**: Return types specified in `primary_return_types` parameter (default: [1,6,8,10])
- **Secondary Returns**: All other return types not specified as primary
- **Classification**: Used for advanced two-criteria filtering

### Range-Aware Visibility Estimation

The visibility metric is calculated only for voxels within the configured range:

- **Range Filtering**: Only voxels with maximum radius ≤ `visibility_estimation_max_range_m`, `visibility_estimation_min_azimuth_rad` ≤ azimuth ≤ `visibility_estimation_max_azimuth_rad`, and `visibility_estimation_min_elevation_rad` ≤ elevation ≤ `visibility_estimation_max_elevation_rad` are considered
- **Visibility Estimation Tuning**: Reported visibility value is tuned using the `visibility_estimation_max_secondary_voxel_count` parameter
- **Reliability**: Excludes potentially unreliable distant measurements from visibility calculations
- **Configurable**: Allows adjustment based on sensor characteristics and requirements

### Filtering Methodology

The filter uses different algorithms based on the `use_return_type_classification` parameter and can operate in two output modes:

#### Normal Mode (`visibility_estimation_only=false`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Coordinate Processing**: Uses appropriate coordinate source
3. **Voxel Processing**: Groups points into polar voxels
4. **Filtering Logic**: Applies simple or advanced filtering
5. **Output Generation**: Creates filtered point cloud
6. **Diagnostics**: Publishes filter ratio and visibility metrics
7. **Optional Noise Cloud**: Publishes filtered-out points if enabled

#### Visibility Estimation Only Mode (`visibility_estimation_only=true`)

1. **Format Detection**: Same as normal mode
2. **Coordinate Processing**: Same as normal mode
3. **Voxel Processing**: Same as normal mode
4. **Filtering Logic**: Same as normal mode (for accurate diagnostics)
5. **Output Generation**: **SKIPPED** - creates empty output for interface compatibility
6. **Diagnostics**: **ALWAYS PUBLISHED** - full visibility and filter ratio metrics
7. **Noise Cloud**: **SKIPPED** - no noise cloud generation regardless of `publish_noise_cloud` setting

**Use Cases for Visibility Estimation Only Mode:**

- **Environmental monitoring**: Track visibility conditions without processing overhead
- **Sensor health monitoring**: Monitor LiDAR performance without data pipeline impact
- **Algorithm validation**: Test filtering parameters without output processing
- **Diagnostic-only applications**: Pure monitoring without data transformation

#### Simple Mode (`use_return_type_classification=false`)

1. **Format Detection**: Automatically detects PointXYZIRC vs PointXYZIRCAEDT
2. **Coordinate Processing**:
   - PointXYZIRC: Computes polar coordinates from Cartesian
   - PointXYZIRCAEDT: Uses pre-computed polar coordinates
3. **Voxel Binning**: Points are grouped into polar voxels
4. **Simple Thresholding**: Voxels with ≥ `voxel_points_threshold` points (any return type) are kept
5. **Output**: Filtered point cloud with basic noise removal (unless visibility-only mode)

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
6. **Range-Aware Visibility**: Visibility calculation limited to voxels within `visibility_estimation_max_range_m`, `visibility_estimation_(min|max)_azimuth_rad`, and `visibility_estimation_(min|max)_elevation_rad`, and secondary voxel count limited by `visibility_estimation_max_secondary_voxel_count`
7. **Secondary Return Filtering**: Optional exclusion of secondary returns from output
8. **Output**: Filtered point cloud with enhanced noise removal (unless visibility-only mode)

### Advanced Two-Criteria Filtering

When enabled, for each voxel both criteria must be satisfied:

- **Primary Return Threshold**: `primary_count >= voxel_points_threshold`
- **Secondary Return Threshold**: `secondary_count <= secondary_noise_threshold`
- **Final Decision**: `valid_voxel = (primary_threshold_met AND secondary_threshold_met)`

### Key Features

- **Flexible Architecture**: Configurable between simple and advanced filtering
- **Format-Optimized Processing**: Automatic selection of optimal coordinate source
- **Range-Aware Diagnostics**: Visibility estimation limited to reliable sensor range
- **Secondary Voxel Limiting**: Configurable limit on secondary voxels for visibility estimation
- **Visibility-Only Mode**: Diagnostic operation without point cloud output
- **Comprehensive Diagnostics**: Mode-specific filter ratio and visibility metrics
- **Debug Support**: Optional noise cloud publishing for analysis and tuning

### Return Type Management (Advanced Mode Only)

- **Primary Returns**: Configurable list of return types (default: [1,6,8,10])
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
| `~/polar_voxel_outlier_filter/debug/filter_ratio`     | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of output to input points (always published)                                          |
| `~/polar_voxel_outlier_filter/debug/visibility`       | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of voxels passing secondary return threshold test (advanced mode only, range-limited) |
| `~/polar_voxel_outlier_filter/debug/pointcloud_noise` | `sensor_msgs::msg::PointCloud2`                     | Filtered-out points for debugging (when enabled and not in visibility-only mode)            |

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_outlier_filter_node.schema.json") }}

### Parameter Interactions

- **use_return_type_classification**: Must be `true` to enable advanced two-criteria filtering
- **filter_secondary_returns**: When `true`, only primary returns appear in output (advanced mode only)
- **secondary_noise_threshold**: Only used when `use_return_type_classification=true`
- **visibility_estimation_max_secondary_voxel_count**: Only used when `use_return_type_classification=true`, limits secondary voxel counting in visibility calculations
- **primary_return_types**: Only used when `use_return_type_classification=true`
- **visibility_estimation_max_range_m**: Limits visibility calculation to reliable sensor range (advanced mode only)
- **visibility_estimation_only**: When `true`, skips point cloud output generation but still calculates and publishes diagnostics
- **publish_noise_cloud**: When `false`, improves performance by skipping noise cloud generation (ignored when `visibility_estimation_only=true`)
- **Diagnostics**: Visibility is only published when return type classification is enabled

## Configuration Examples

### Visibility-Only Configuration

```yaml
# Run filter for diagnostics only - no point cloud processing
visibility_estimation_only: true
use_return_type_classification: true
voxel_points_threshold: 2
secondary_noise_threshold: 4
visibility_estimation_max_secondary_voxel_count: 500
visibility_estimation_max_range_m: 20.0
primary_return_types: [1, 6, 8, 10]
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0175
elevation_resolution_rad: 0.0175
# publish_noise_cloud is ignored in visibility-only mode
```

### Simple Mode Configuration

```yaml
# Basic occupancy filtering - any return type counts equally
use_return_type_classification: false
voxel_points_threshold: 2 # Total points threshold
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0175 # ~1 degree
elevation_resolution_rad: 0.0175 # ~1 degree
visibility_estimation_only: false # Normal filtering mode
publish_noise_cloud: true # Enable for debugging
```

### Advanced Mode Configuration

```yaml
# Two-criteria filtering with return type classification
use_return_type_classification: true
voxel_points_threshold: 2 # Primary return threshold
secondary_noise_threshold: 4 # Secondary return threshold
visibility_estimation_max_secondary_voxel_count: 500 # Max secondary voxels for visibility
primary_return_types: [1, 6, 8, 10] # Primary return types
filter_secondary_returns: false # Include secondary returns in output
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0175 # ~1 degree
elevation_resolution_rad: 0.0175 # ~1 degree
visibility_estimation_max_range_m: 20.0 # Range limit for visibility calculation
visibility_estimation_min_azimuth_rad: 0.78
visibility_estimation_max_azimuth_rad: 2.35
visibility_estimation_min_elevation_rad: -0.26
visibility_estimation_max_elevation_rad: 1.04
visibility_estimation_only: false # Normal filtering mode
publish_noise_cloud: true
```

### Debug Configuration

```yaml
# Enable debugging and monitoring
use_return_type_classification: true
visibility_estimation_max_range_m: 20.0 # Configured range for urban environments
visibility_estimation_min_azimuth_rad: 0.78
visibility_estimation_max_azimuth_rad: 2.35
visibility_estimation_min_elevation_rad: -0.26
visibility_estimation_max_elevation_rad: 1.04
visibility_estimation_max_secondary_voxel_count: 500 # Allow secondary voxels in visibility calculation
visibility_estimation_only: false # Normal filtering with debug output
publish_noise_cloud: true # Enable noise cloud for analysis
filter_ratio_error_threshold: 0.5
filter_ratio_warn_threshold: 0.7
visibility_error_threshold: 0.8
visibility_warn_threshold: 0.9
```

### Sensor-Specific Configuration Examples

```yaml
# Long-range highway LiDAR
visibility_estimation_max_range_m: 200.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
voxel_points_threshold: 2
visibility_estimation_only: false

# Urban short-range LiDAR
visibility_estimation_max_range_m: 20.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
voxel_points_threshold: 2
visibility_estimation_only: false

# High-resolution near-field processing
visibility_estimation_max_range_m: 20.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 0.5
azimuth_resolution_rad: 0.0175 # ~1 degree
elevation_resolution_rad: 0.0175 # ~1 degree
visibility_estimation_only: false

# Environmental monitoring only
visibility_estimation_max_range_m: 50.0
visibility_estimation_max_secondary_voxel_count: 500
radial_resolution_m: 1.0 # Coarser resolution for performance
visibility_estimation_only: true # Diagnostics only
```

## Assumptions / Known limits

- **Simple mode**: Works with any point cloud format, basic occupancy filtering only
- **Advanced mode**: Requires return_type field for enhanced filtering
- **Visibility-only mode**: Runs full filtering algorithm but produces no point cloud output
- **Supported formats**: PointXYZIRC and PointXYZIRCAEDT only
- **Finite coordinates required**: Automatically filters out NaN/Inf points
- **Return type dependency**: Advanced filtering effectiveness depends on accurate return type classification
- **Visibility range dependency**: Visibility accuracy depends on appropriate `visibility_estimation_max_range_m`, `visibility_estimation_(min|max)_azimuth_rad`, and `visibility_estimation_(min|max)_elevation_rad` settings. Besides, for azimuth and elevation range, the following definitions are assumed
  - azimuth: starts with the y-axis, increasing in counter-corkscrew rule around the z-axis. The range domain is $`[0, 2\pi]`$
  - elevation: starts with the x-axis, increasing in counter-corkscrew rule around the y-axis. The range domain is $`[-\frac{\pi}{2}, \frac{pi}{2}]`$
- **Secondary voxel limiting**: Visibility estimation can be tuned via `visibility_estimation_max_secondary_voxel_count`

## Error detection and handling

The filter includes robust error handling:

- **Mode-specific validation**: Checks return_type field presence in advanced mode
- **Input validation**: Checks for null point clouds
- **Coordinate validation**: Filters invalid points (NaN, Inf values) automatically
- **Range validation**: Points outside configured radius ranges are excluded
- **Parameter validation**: Ensures `visibility_estimation_max_range_m` > 0 and `visibility_estimation_max_secondary_voxel_count` ≥ 0
- **Dynamic parameter validation**: Runtime parameter updates with validation
- **Mode compatibility**: Validates parameter combinations for different operating modes

## Usage

### Launch the Filter

```bash
# Example launch file integration for simple mode:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="use_return_type_classification" value="false"/>
#   <param name="radial_resolution_m" value="1.0"/>
#   <param name="azimuth_resolution_rad" value="0.0349"/>
#   <param name="voxel_points_threshold" value="3"/>
#   <param name="visibility_estimation_only" value="false"/>
# </node>

# Example launch file integration for advanced mode:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="use_return_type_classification" value="true"/>
#   <param name="radial_resolution_m" value="0.5"/>
#   <param name="azimuth_resolution_rad" value="0.0175"/>
#   <param name="voxel_points_threshold" value="2"/>
#   <param name="secondary_noise_threshold" value="4"/>
#   <param name="visibility_estimation_max_secondary_voxel_count" value="500"/>
#   <param name="primary_return_types" value="[1,6,8,10]"/>
#   <param name="visibility_estimation_max_range_m" value="20.0"/>
#   <param name="visibility_estimation_min_azimuth_rad" value="0.78"/>
#   <param name="visibility_estimation_max_azimuth_rad" value="2.35"/>
#   <param name="visibility_estimation_min_elevation_rad" value="-0.26"/>
#   <param name="visibility_estimation_max_elevation_rad" value="1.04"/>
#   <param name="visibility_estimation_only" value="false"/>
# </node>

# Example launch file integration for visibility-only mode:
# <node pkg="autoware_pointcloud_preprocessor" exec="polar_voxel_outlier_filter_node" name="polar_voxel_filter">
#   <param name="use_return_type_classification" value="true"/>
#   <param name="visibility_estimation_only" value="true"/>
#   <param name="visibility_estimation_max_range_m" value="20.0"/>
#   <param name="visibility_estimation_min_azimuth_rad" value="0.78"/>
#   <param name="visibility_estimation_max_azimuth_rad" value="2.35"/>
#   <param name="visibility_estimation_min_elevation_rad" value="-0.26"/>
#   <param name="visibility_estimation_max_elevation_rad" value="1.04"/>
#   <param name="visibility_estimation_max_secondary_voxel_count" value="500"/>
# </node>
```

### ROS 2 Topics

#### Input/Output

- **Input**: `/input` (sensor_msgs/PointCloud2) - Must have return_type field for advanced mode
- **Output**: `/output` (sensor_msgs/PointCloud2) - Empty in visibility-only mode

#### Debug Topics

- **Filter Ratio**: `~/polar_voxel_outlier_filter/debug/filter_ratio` (autoware_internal_debug_msgs/Float32Stamped) - Always published
- **Visibility**: `~/polar_voxel_outlier_filter/debug/visibility` (autoware_internal_debug_msgs/Float32Stamped) - Advanced mode only, range-limited
- **Noise Cloud**: `~/polar_voxel_outlier_filter/debug/pointcloud_noise` (sensor_msgs/PointCloud2) - Not published in visibility-only mode

### Programmatic Usage

```cpp
#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

// Create node
auto node = std::make_shared<autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent>(options);

// The filter automatically detects point cloud format and applies filtering based on configuration:
// - Simple mode (use_return_type_classification=false): Basic occupancy filtering
// - Advanced mode (use_return_type_classification=true): Two-criteria filtering with return type analysis
// - Visibility-only mode (visibility_estimation_only=true): Diagnostics without point cloud output
// - Both modes support PointXYZIRC and PointXYZIRCAEDT formats
// - Advanced mode uses range-limited visibility estimation with configurable secondary voxel limiting
```

## Performance characterization

### Computational Complexity

- **Time Complexity**: O(n) where n is the number of input points
- **Space Complexity**: O(v) where v is the number of occupied voxels

### Performance Impact by Mode

#### **Visibility Estimation Only Mode**

- **Computational**: Runs full filtering algorithm for accurate diagnostics
- **Memory**: Minimal memory usage - no output point cloud allocation
- **I/O**: Publishes diagnostics only, no point cloud output
- **Use Case**: Optimal for monitoring applications without data processing needs

#### **Normal Mode**

- **PointXYZIRCAEDT**: Optimal performance with pre-computed coordinates
- **PointXYZIRC**: Good performance with coordinate conversion overhead
- **Output Processing**: Full point cloud generation and optional noise cloud

#### **Simple Mode**

- **PointXYZIRCAEDT**: Fast processing with pre-computed coordinates
- **PointXYZIRC**: Good performance with coordinate conversion overhead
- **No return type analysis**: Reduced computational overhead

#### **Advanced Mode**

- **PointXYZIRCAEDT**: Optimal performance with pre-computed coordinates and return type analysis
- **PointXYZIRC**: Good performance with coordinate conversion and return type analysis
- **Enhanced filtering**: Additional return type classification and range-aware visibility processing

### Memory Usage

- **Visibility-only mode**: Significantly reduced memory footprint
- **Hash-based voxel storage**: Efficiently handles sparse voxel occupancy
- **Single-pass processing**: Minimal memory overhead regardless of mode
- **Mode-specific outputs**: Memory allocation optimized per mode
- **Range filtering**: Additional hash map for visibility calculation (advanced mode only)

### Optimization Tips

1. **Use visibility estimation only mode** for pure monitoring applications
2. **Choose appropriate mode** based on requirements:
   - Normal mode for data processing needs
   - Visibility-only mode for environmental/sensor monitoring
3. **Use PointXYZIRCAEDT format** when available for optimal performance
4. **Combine modes dynamically** - switch at runtime based on operational needs
5. **Tune voxel resolutions** based on your use case
6. **Configure return type mappings** to match your sensor (advanced mode)
7. **Set appropriate visibility range** (`visibility_estimation_max_range_m`) for your sensor and environment
8. **Tune secondary voxel limiting** (`visibility_estimation_max_secondary_voxel_count`) for visibility estimation accuracy
9. **Monitor diagnostics** for real-time performance assessment in both modes

## Diagnostics and Monitoring

### Filter Ratio Diagnostics

- **Published for all modes**: Overall filtering effectiveness (output/input ratio)
- **Configurable thresholds**: Error/warning levels for automated monitoring
- **Real-time feedback**: Immediate filtering performance assessment
- **Visibility-only mode**: Shows theoretical filter effectiveness without actual filtering

### Visibility Diagnostics

- **Advanced mode only**: Uses return type classification data
- **Range-limited metric**: Only considers voxels within `visibility_estimation_max_range_m`, `visibility_estimation_(min|max)_azimuth_rad` and `visibility_estimation_(min|max)_elevation_rad`
- **Secondary voxel limiting**: Controlled by `visibility_estimation_max_secondary_voxel_count` parameter
- **Voxel-based metric**: Percentage of range-limited voxels passing secondary threshold test
- **Environmental indicator**: Useful for detecting sensor conditions within reliable range
- **Diagnostic context**: Status messages include the configured visibility estimation range and secondary voxel limits
- **Available in all modes**: Published even in visibility-only mode for monitoring

### Debug Features

- **Noise point cloud**: All filtered-out points for analysis (when enabled and not in visibility-only mode)
- **Runtime parameter updates**: Dynamic threshold and range adjustment
- **Mode-specific logging**: Debug messages tailored to filtering mode
- **Range-aware diagnostics**: Visibility calculations clearly indicate the estimation range

## Use Cases and Configuration Guidelines

### Visibility Estimation Only Mode Use Cases

1. **Environmental Monitoring**: Track atmospheric conditions without processing overhead
2. **Sensor Health Monitoring**: Monitor LiDAR performance and visibility conditions
3. **Algorithm Validation**: Test and tune filtering parameters without output generation
4. **Pure Diagnostics**: Applications that only need visibility and filter ratio metrics
5. **Resource-Constrained Systems**: Minimize computational load while maintaining monitoring
6. **Weather Station Integration**: Automated visibility reporting for meteorological systems

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

#### Visibility Estimation Only Mode Configuration

- **For environmental monitoring**: Enable advanced mode with appropriate visibility range
- **For sensor monitoring**: Use return type classification for detailed analysis
- **For performance**: Larger voxel resolutions to reduce computation
- **For accuracy**: Smaller voxel resolutions for precise visibility estimation

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
- **For range-specific visibility**: Set `visibility_estimation_max_range_m`, `visibility_estimation_(min|max)_azimuth_rad`, and `visibility_estimation_(min|max)_elevation_rad` based on sensor effective range and application requirements

#### Mode Selection Guidelines

- **Choose visibility-only mode when**:
  - Only diagnostic information is needed
  - Computational resources are limited
  - Running parallel monitoring alongside main processing
  - Testing filter parameters without affecting downstream systems

- **Choose normal mode when**:
  - Filtered point cloud output is required
  - Integration into data processing pipelines
  - Real-time filtering for perception systems
  - Full functionality including noise cloud debugging

#### Visibility Range Guidelines

- **Urban environments**: 30-80m (shorter ranges for reliable near-field analysis)
- **Highway applications**: 100-200m (longer ranges for high-speed scenarios)
- **Parking/loading**: 10-30m (very short ranges for precise near-field monitoring)
- **Sensor specifications**: Match to sensor's reliable detection range

## Comparison Table

| Aspect                   | Simple Mode     | Advanced Mode                   | Visibility-Only Mode            |
| ------------------------ | --------------- | ------------------------------- | ------------------------------- |
| **Filtering Method**     | Basic occupancy | Two-criteria with return type   | Same as selected mode           |
| **Return Type Required** | No              | Yes                             | Depends on mode selected        |
| **Computational Cost**   | Low             | Moderate                        | Moderate (no output)            |
| **Filtering Quality**    | Good            | Excellent                       | N/A (no output)                 |
| **Visibility Metrics**   | None            | Range-aware with voxel limiting | Range-aware with voxel limiting |
| **Configuration**        | Simple          | Advanced                        | Advanced (diagnostics-focused)  |
| **Use Case**             | Basic filtering | Enhanced noise removal          | Monitoring/diagnostics          |
| **Environmental Adapt**  | Limited         | Comprehensive                   | Comprehensive                   |
| **Range Awareness**      | Basic           | Configurable                    | Configurable                    |
| **Point Cloud Output**   | Yes             | Yes                             | No (empty)                      |

## Migration Guide

### Enabling Visibility Estimation Only Mode

To enable diagnostic-only operation:

1. **Set parameter**: `visibility_estimation_only: true`
2. **Configure diagnostics**: Ensure proper visibility and filter ratio thresholds
3. **Verify mode**: Check logs for "visibility estimation only" confirmation
4. **Monitor diagnostics**: Use published metrics for monitoring
5. **No output expectation**: Downstream nodes should handle empty point clouds

### Dynamic Mode Switching

The mode can be changed at runtime:

```bash
# Switch to visibility-only mode
ros2 param set /polar_voxel_filter visibility_estimation_only true

# Switch back to normal mode
ros2 param set /polar_voxel_filter visibility_estimation_only false
```

### Enabling Advanced Mode

To enable advanced filtering on existing systems:

1. **Ensure return type field**: Verify input point clouds have return_type field
2. **Set parameter**: `use_return_type_classification: true`
3. **Configure return types**: Set `primary_return_types` for your sensor
4. **Set visibility range**: Configure `visibility_estimation_max_range_m`, `visibility_estimation_(min|max)_azimuth_rad`, and `visibility_estimation_(min|max)_elevation_rad` for your application
5. **Configure secondary voxel limiting**: Set `visibility_estimation_max_secondary_voxel_count` based on requirements
6. **Tune thresholds**: Adjust `secondary_noise_threshold` based on requirements
7. **Monitor diagnostics**: Use range-aware visibility metrics for performance assessment

### Disabling Advanced Mode

To use simple mode for basic filtering:

1. **Set parameter**: `use_return_type_classification: false`
2. **Configure threshold**: Set `voxel_points_threshold` for total point count
3. **Remove advanced parameters**: Return type and visibility range parameters will be ignored
4. **Simplified monitoring**: Only filter ratio diagnostics available

### Compatibility Considerations

- **Output interface**: Empty point clouds maintain topic compatibility in visibility-only mode
- **Diagnostic topics**: Same diagnostic information regardless of mode
- **Parameter compatibility**: All filtering parameters work in both normal and visibility-only modes
- **Performance impact**: Mode changes take effect on next filter call

### Updating Existing Configurations

For systems already using advanced mode, add the new parameters:

```yaml
# Add to existing configuration:
visibility_estimation_only: false # Default for normal operation
visibility_estimation_max_secondary_voxel_count: 500 # Updated default
primary_return_types: [1, 6, 8, 10] # Updated to include return type 8
```

This approach provides **maximum flexibility** with **range-aware visibility estimation**, **configurable secondary voxel limiting**, and **diagnostic-only operation** while maintaining optimal performance for all use cases! 🎯✨
