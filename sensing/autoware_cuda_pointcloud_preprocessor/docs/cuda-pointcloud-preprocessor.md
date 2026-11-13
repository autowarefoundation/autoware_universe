# cuda_pointcloud_preprocessor

## Purpose

This node implements all standard pointcloud preprocessing algorithms applied to a single LiDAR's pointcloud in CUDA.
In particular, this node implements:

- box cropping (ego-vehicle and ego-vehicle's mirrors)
- distortion correction
- ring-based outlier filtering

## Inner-workings / Algorithms

As this node reimplements the functionalities of the CPU-version algorithms, please have a look at the documentations of [crop-box](../../autoware_pointcloud_preprocessor/docs/crop-box-filter.md), [distortion correction](../../autoware_pointcloud_preprocessor/docs/distortion-corrector.md), and [ring-based outlier filter](../../autoware_pointcloud_preprocessor/docs/ring-outlier-filter.md) for more information about these algorithms.

In addition to the individual algorithms previously mentioned, this node uses the `cuda_blackboard`, a cuda transport layer that enables a zero-copy mechanism between GPU and GPU memory for both input and output.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                               |
| ------------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud's topic.                 |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud's type negotiation topic |
| `~/input/twist`           | `geometry_msgs::msg::TwistWithCovarianceStamped` | Topic of the twist information.           |
| `~/input/imu`             | `sensor_msgs::msg::Imu`                          | Topic of the IMU data.                    |

### Output

| Name                       | Type                                             | Description                              |
| -------------------------- | ------------------------------------------------ | ---------------------------------------- |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Processed pointcloud's topic             |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Processed pointcloud's negotiation topic |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_cuda_pointcloud_preprocessor/schema/cuda_pointcloud_preprocessor.schema.json") }}

## Assumptions / Known limits

- The CUDA implementations, while following the original CPU ones, will not offer the same numerical results, and small approximations were needed to maximize GPU usage.
- This node expects that the input pointcloud follows the `autoware::point_types::PointXYZIRCAEDT` layout. The output pointcloud uses the `autoware::point_types::PointXYZIRC` layout, or `autoware::point_types::PointXYZIRCT` when `output_point_type` is `XYZIRCT`; both are defined in the `autoware_point_types` package. The `time_stamp` of `PointXYZIRCT` is copied from the input, i.e. relative to the cloud's header stamp.
- Downstream nodes must accept 20-byte `PointXYZIRCT` points before `XYZIRCT` is enabled: the concatenation nodes only do so from [autowarefoundation/autoware_universe#13324](https://github.com/autowarefoundation/autoware_universe/pull/13324) on.
