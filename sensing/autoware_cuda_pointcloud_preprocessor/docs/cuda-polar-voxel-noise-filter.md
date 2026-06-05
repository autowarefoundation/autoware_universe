# cuda_polar_voxel_noise_filter

## Purpose

This node is a CUDA-accelerated version of the `PolarVoxelNoiseFilter` available in [autoware_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor).

## Inner-workings / Algorithms

This node is an alternative implementation to `autoware::pointcloud_preprocessor::PolarVoxelNoiseFilterComponent`, which removes low-density polar voxels and optionally suppresses secondary returns using return-type classification.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                              |
| ------------------------- | ------------------------------------------------ | ---------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud topic.                  |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud type negotiation topic. |

### Output

| Name                       | Type                                             | Description                                 |
| -------------------------- | ------------------------------------------------ | ------------------------------------------- |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Filtered pointcloud topic.                  |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Filtered pointcloud type negotiation topic. |

#### Additional Debug Topics

| Name                            | Type                                             | Description                         |
| ------------------------------- | ------------------------------------------------ | ----------------------------------- |
| `~/debug/pointcloud_noise`      | `sensor_msgs::msg::PointCloud2`                  | Points classified as noise.         |
| `~/debug/pointcloud_noise/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Noise pointcloud negotiation topic. |

## Parameters

See [the original implementation in autoware_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor/docs/polar-voxel-noise-filter.md) for the detailed parameter definitions.

## Assumptions / Known limits

- Input pointclouds must contain an `intensity` field.
- If `use_return_type_classification` is enabled, the input pointcloud must also contain a `return_type` field.
- The CUDA implementation supports `PointXYZIRC` input and `PointXYZIRCAEDT` input with precomputed polar coordinates.
- Due to floating-point differences between CPU and GPU execution, results may not be bitwise identical to the CPU implementation.
