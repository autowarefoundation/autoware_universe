# autoware_ptv3

## Purpose

The `autoware_ptv3` package is used for 3D lidar segmentation.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for Point Transformers V3 (PTv3) [1].
The sparse convolution backend corresponds to [spconv](https://github.com/traveller59/spconv).
Autoware installs it automatically in its setup script. If needed, the user can also build it and install it following the [following instructions](https://github.com/autowarefoundation/spconv_cpp).

## Inputs / Outputs

### Input

| Name                 | Type                            | Description             |
| -------------------- | ------------------------------- | ----------------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | Input pointcloud topic. |

### Output

| Name                                   | Type                                                | Description                                             |
| -------------------------------------- | --------------------------------------------------- | ------------------------------------------------------- |
| `~/output/pointcloud/segmentation`     | `sensor_msgs::msg::PointCloud2`                     | XYZ cloud with class ID and probability fields.         |
| `~/output/pointcloud/visualization`    | `sensor_msgs::msg::PointCloud2`                     | XYZ cloud with RGB field.                               |
| `~/output/pointcloud/filtered`         | `sensor_msgs::msg::PointCloud2`                     | Filtered cloud in the requested `filter.output_format`. |
| `debug/cyclic_time_ms`                 | `autoware_internal_debug_msgs::msg::Float64Stamped` | Cyclic time (ms).                                       |
| `debug/pipeline_latency_ms`            | `autoware_internal_debug_msgs::msg::Float64Stamped` | Pipeline latency time (ms).                             |
| `debug/processing_time/preprocess_ms`  | `autoware_internal_debug_msgs::msg::Float64Stamped` | Preprocess (ms).                                        |
| `debug/processing_time/inference_ms`   | `autoware_internal_debug_msgs::msg::Float64Stamped` | Inference time (ms).                                    |
| `debug/processing_time/postprocess_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Postprocess time (ms).                                  |
| `debug/processing_time/total_ms`       | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total processing time (ms).                             |

## Parameters

### PTv3Node node

{{ json_to_markdown("perception/autoware_ptv3/schema/ptv3.schema.json") }}

### PTv3Node model

{{ json_to_markdown("perception/autoware_ptv3/schema/ml_package_ptv3.schema.json") }}

`filter.*` parameters are configured in `config/ptv3.param.yaml`, while class metadata and the
visualization `palette` are configured in `config/ml_package_ptv3.param.yaml`.

### The `build_only` option

The `autoware_ptv3` node has a `build_only` option to build the TensorRT engine file from the specified ONNX file, after which the program exits.

```bash
ros2 launch autoware_ptv3 ptv3.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_ptv3` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_ptv3 ptv3.launch.xml log_level:=debug
```

## Assumptions / Known limits

This node detects the input pointcloud format automatically on the first received message and
supports:

- `XYZIRCAEDT` (10 fields)
- `XYZIRADRT` (9 fields)
- `XYZIRC` (6 fields)
- `XYZI` (4 fields)

The filtered output cloud format is controlled by `filter.output_format`. When it is set to an
empty string, the filtered output preserves the same format as the input cloud.

## Trained Models

The model was trained on the T4Dataset using approximately 4,000 frames and is available in the Autoware artifacts.

### Changelog

## References/External links

[1] Xiaoyang Wu, Li Jiang, Peng-Shuai Wang, Zhijian Liu, Xihui Liu, Yu Qiao, Wanli Ouyang, Tong He, and Hengshuang Zhao. "Point Transformer V3: Simpler, Faster, Stronger." 2024 Conference on Computer Vision and Pattern Recognition. <!-- cspell:disable-line -->
