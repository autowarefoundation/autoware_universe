# cuda_voxel_grid_downsample_filter

## Purpose

This node is a CUDA accelerated version of the `FasterVoxelGridDownsampleFilter` available in [autoware_cuda_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor).

## Inner-workings / Algorithms

This node reimplements of the function of `autoware::pointcloud_preprocessor::FasterVoxelGridDownsampleFilter`, which calculates centroids as representative points in each voxels.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                               |
| ------------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud's topic.                 |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud's type negotiation topic |

### Output

| Name                       | Type                                             | Description                                             |
| -------------------------- | ------------------------------------------------ | ------------------------------------------------------- |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Processed pointcloud's topic (in `PointXYZIRC` fashion) |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Processed pointcloud's negotiation topic                |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_cuda_pointcloud_preprocessor/schema/cuda_voxel_grid_downsample_filter.schema.schema.json") }}

## Assumptions / Known limits

- This node expects that the input pointcloud is compatible with `autoware::point_types::PointXYZI` ([ref](https://github.com/autowarefoundation/autoware_core/tree/main/common/autoware_point_types)). Here, "compatible" means the pointcloud fields start with XYZI in this order. Multiple data types, including `uint8` and `float`, are supported for the datatype of the `intensity` field.
