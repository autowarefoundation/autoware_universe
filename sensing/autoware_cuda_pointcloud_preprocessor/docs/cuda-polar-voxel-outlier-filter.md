# cuda_polar_voxel_outlier_filter

## Purpose

This node is a CUDA accelerated version of the `PolarVoxelOutlierFilter` available in [autoware_cuda_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor).

## Inner-workings / Algorithms

This node is an alternative implementation to `autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent`, which filters outliers based on voxels in polar coordinate space instead of Cartesian coordinate space.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                               |
| ------------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud's topic.                 |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud's type negotiation topic |

### Output

| Name                       | Type                                             | Description                              |
| -------------------------- | ------------------------------------------------ | ---------------------------------------- |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Processed pointcloud's topic             |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Processed pointcloud's negotiation topic |

#### Additional Debug Topics

| Name                            | Type                                                | Description                                                                    |
| ------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------ |
| `~/debug/filter_ratio`          | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of output to input points                                                |
| `~/debug/visibility`            | `autoware_internal_debug_msgs::msg::Float32Stamped` | Ratio of voxels passing secondary return threshold test (PointXYZIRCAEDT only) |
| `~/debug/pointcloud_noise`      | `sensor_msgs::msg::PointCloud2`                     | Processed pointcloud's topic which is categorized as outlier                   |
| `~/debug/pointcloud_noise/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo`    | Negotiation topic                                                              |

## Parameters

See [the original implementation in autoware_cuda_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor/docs/polar-voxel-outlier-filter.md) for the detail.

### Core Parameters (Schema-based)

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/polar_voxel_outlier_filter_node.schema.json") }}

## Assumptions / Known limits

Due to differences in floating-point arithmetic between CPUs and GPUs, the outputs of `autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent` and this filter may not be identical.

Adding compiler options, such as the following, can reduce numerical discrepancies, though a slight performance impact can also be introduced, and it is still difficult to acquire complete identical results.

```CMake
list(APPEND CUDA_NVCC_FLAGS "--fmad=false")
```

To prioritize performance, these compiler options are not enabled.
