# polar_voxel_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain using a polar coordinate voxel grid approach that is optimized for LiDAR sensor characteristics. This filter provides a two-criteria filtering method for PointXYZIRCAEDT format with return type classification, which gives more flexibility in configuration when using dual return modes and also enables visibility estimation.

## Inner-workings / Algorithms

### Filtering Methodology

The filter uses different algorithms depending on the input point cloud format:

#### For PointXYZ format:
- **Simple Occupancy Filtering**: Voxels with ≥ `voxel_points_threshold` points are kept
- Computes polar coordinates from Cartesian coordinates
- No return type classification

#### For PointXYZIRCAEDT format (with return type classification enabled):
- **Two-Criteria Filtering**: Both criteria must be satisfied:
1. **Primary Return Threshold**: The number of primary return points in the voxel must be greater than or equal to `voxel_points_threshold`.
2. **Secondary Noise Threshold**: If there are any secondary returns in the voxel, the number of primary return points must exceed the number of secondary return points by at least `secondary_noise_threshold`. If there are no secondary returns, only the primary return threshold is checked.
- Uses pre-computed polar coordinates for optimal performance
- **Optimized Visibility Tracking**: Statistics collected inline during filtering (single-pass)
- Supports sophisticated return type management

### Key Features

- **Polar Coordinate System**: Uses (radius, azimuth, elevation) which naturally align with LiDAR sensor patterns
- **Return Type Classification**: Configurable primary/secondary return handling
- **Secondary Return Filtering**: Optional filtering to keep only primary returns in output
- **Dynamic Threshold Evaluation**: Ensures primary returns dominate over secondary returns
- **Comprehensive Diagnostics**: Filter ratio and visibility metrics with configurable thresholds
- **Debug Support**: Publishes filtered-out points for analysis and tuning

### Return Type Management

- **Primary Returns**: Typically stronger, more reliable returns (default: [1])
- **Secondary Returns**: Weaker returns, usually from partially transparent objects (default: [2,3,4,5])
- **Classification Control**: Can be disabled to process all returns equally
- **Filtering Control**: Can optionally exclude secondary returns from output
Note that primary and second returns vary between sensors and return modes, so parametrization should be performed after studying real sensor data.

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Additional Debug Topics

| Name                                                          | Type                            | Description                               |
| ------------------------------------------------------------- | ------------------------------- | ----------------------------------------- |
| `~/polar_voxel_outlier_filter/debug/filter_ratio`           | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of output to input points |
| `~/polar_voxel_outlier_filter/debug/visibility`             | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of voxels passing secondary return threshold test (PointXYZIRCAEDT only) |
| `~/polar_voxel_outlier_filter/debug/pointcloud_noise`       | `sensor_msgs::msg::PointCloud2` | Filtered-out points for debugging       |

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_outlier_filter_node.schema.json") }}

### Parameter Interactions

- **use_return_type_classification**: Must be `true` to enable advanced two-criteria filtering
- **filter_secondary_returns**: When `true`, only primary returns appear in output (regardless of filtering criteria)
- **secondary_noise_threshold**: Higher values require stronger primary return dominance
- **Diagnostics**: Visibility is only published when return type classification is enabled

## Assumptions / Known limits

- Designed primarily for LiDAR point clouds with polar coordinate characteristics
- **Advanced filtering requires PointXYZIRCAEDT format** with `use_return_type_classification=true`
- PointXYZ format uses simple occupancy filtering only
- Requires finite coordinate values (filters out NaN/Inf points automatically)
- **Two-criteria filtering**: Both primary return threshold AND threshold evaluation must be satisfied
- Edge case: Voxels with zero secondary returns pass threshold criterion if they have primary returns

## (Optional) Error detection and handling

The filter includes robust error handling:
- Input validation for null point clouds
- Automatic filtering of invalid points (NaN, Inf values)
- Graceful handling of points outside configured radius ranges
- Support for both point cloud formats with automatic detection
- **Zero division protection** in threshold calculations
- Dynamic parameter validation and updates

## (Optional) Performance characterization

- **PointXYZIRCAEDT format**: Optimal performance using pre-computed polar coordinates
- **PointXYZ format**: Additional computational overhead for coordinate conversion
- **Two-criteria filtering**: Slightly more computation but significantly better filtering quality
- **Single-pass processing**: Visibility statistics collected during main filtering loop for efficiency
- Memory efficient with point index mapping and separate return type vectors
- **Debug overhead**: Noise point cloud generation adds minimal processing time
- **Optimized visibility calculation**: No redundant voxel traversal, computed inline during filtering

## Diagnostics and Monitoring

### Filter Ratio Diagnostics
- **Always published** for both input formats
- Represents overall filtering effectiveness (output/input ratio)
- Configurable error/warning thresholds

### Visibility Diagnostics  
- **Only published for PointXYZIRCAEDT** with return type classification enabled
- Represents the percentage of voxels that pass the primary-to-secondary threshold test
- **Efficiently computed**: Statistics collected during main filtering loop, no separate traversal
- Useful for detecting sensor blockage, environmental conditions, or filtering effectiveness
- **Real-time tracking**: Counts both passing and failing voxels for comprehensive analysis

### Debug Features
- **Noise point cloud**: Contains all filtered-out points for analysis
- **Runtime parameter updates**: All thresholds can be adjusted dynamically
- **Detailed logging**: Debug messages show voxel statistics and filtering results

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- Potential integration with adaptive voxel sizing based on distance
- ~~Separate handling of primary and secondary returns for improved accuracy~~ ✅ **Implemented**
- Dynamic parameter adjustment based on environmental conditions
- **Multi-criteria weighting**: Beyond simple AND logic for criterion combination
- **Adaptive thresholds**: Environmental condition-based parameter adjustment
