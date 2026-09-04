# cuda_crop_box_filter

## Purpose

This node is a CUDA accelerated version of the `CropBoxFilter` available in [autoware_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor/README.md).

It exists so that a preprocessing chain can stay GPU-resident. The cropping this package already performs lives inside `CudaPointcloudPreprocessorNode`, fused with distortion correction and ring outlier filtering and requiring a per-point time field, so it cannot be used on its own. Without a standalone crop box, a chain that crops before downsampling has to return to the host and come back.

## Inner-workings / Algorithms

Each point is tested against an axis-aligned box in a single kernel, producing a 0/1 mask. An exclusive scan over that mask gives every kept point its output slot, and a second kernel copies the selected points.

Two properties follow from that shape and are worth stating:

- **The point layout passes through untouched.** Selection copies whole points of `point_step` bytes, so intensity, ring, timestamp and any vendor field survive without this node needing to know they exist. Only the `x`, `y` and `z` offsets are read, and they are taken from the input's `fields` rather than assumed.
- **Bounds are inclusive**, matching the CPU filter: a point exactly on a face is inside.

Non-finite points are dropped in **both** polarities. A NaN coordinate compares false against every bound, so it is never "inside"; a naive `negative` would therefore keep it and hand NaN downstream, which is not what the CPU filter does.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                               |
| ------------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud's topic.                 |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud's type negotiation topic |

### Output

| Name                       | Type                                             | Description                                |
| -------------------------- | ------------------------------------------------ | ------------------------------------------ |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Cropped pointcloud's topic                 |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Cropped pointcloud's negotiation topic     |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_cuda_pointcloud_preprocessor/schema/cuda_crop_box_filter.schema.json") }}

## Assumptions / Known limits

- **This node does not transform between frames.** The CPU `CropBoxFilter` can crop in a frame other than the cloud's own and looks up tf to do it. Here the box is required to be in the cloud's frame. `input_frame`, when set, is an assertion: a cloud arriving in a different frame is dropped with an error rather than cropped against the wrong region, because cropping the right box in the wrong frame removes the wrong points and nothing downstream would report it.
- The input must carry `x`, `y` and `z` as `FLOAT32` fields lying wholly within `point_step`. A cloud that does not is dropped with an error; no conversion is attempted.
