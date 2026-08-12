# roi_cluster_fusion

## Purpose

The `roi_cluster_fusion` is a package for filtering clusters that are less likely to be objects and overwriting labels of clusters with that of Region Of Interests (ROIs) by a 2D object detector.

## Inner-workings / Algorithms

The clusters are projected onto image planes, and then if the ROIs of clusters and ROIs by a detector are overlapped, the labels of clusters are overwritten with that of ROIs by detector. Intersection over Union (IoU) is used to determine if there are overlaps between them.

![roi_cluster_fusion_image](./images/roi_cluster_fusion.png)

## Inputs / Outputs

### Input

| Name                     | Type                                                     | Description                                               |
| ------------------------ | -------------------------------------------------------- | --------------------------------------------------------- |
| `input`                  | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | clustered pointcloud                                      |
| `input/camera_info[0-7]` | `sensor_msgs::msg::CameraInfo`                           | camera information to project 3d points onto image planes |
| `input/rois[0-7]`        | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ROIs from each image                                      |
| `input/image_raw[0-7]`   | `sensor_msgs::msg::Image`                                | images for visualization                                  |

### Output

| Name                   | Type                                                     | Description                |
| ---------------------- | -------------------------------------------------------- | -------------------------- |
| `output`               | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | labeled cluster pointcloud |
| `debug/image_raw[0-7]` | `sensor_msgs::msg::Image`                                | images for visualization   |

## Parameters

The following figure is an inner pipeline overview of RoI cluster fusion node.
Please refer to it for your parameter settings.

![roi_cluster_fusion_pipeline](./images/roi_cluster_fusion_pipeline.svg)

### Core Parameters

{{ json_to_markdown("perception/autoware_image_projection_based_fusion/schema/roi_cluster_fusion.schema.json") }}

### Example Configuration

The `iou_threshold` parameter should be configured as an object with thresholds for each object class:

```yaml
iou_threshold:
  UNKNOWN: 0.1
  CAR: 0.65
  TRUCK: 0.65
  BUS: 0.65
  TRAILER: 0.65
  MOTORCYCLE: 0.65
  BICYCLE: 0.65
  PEDESTRIAN: 0.65
```

The threshold values determine the minimum IoU score required to overwrite a cluster's label with the corresponding ROI label. Different thresholds can be set for each object class to account for varying detection characteristics.

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:

  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

<!-- Write future extensions of this package.

Example:
  Currently, this package can't handle the chattering obstacles well. We plan to add some probabilistic filters in the perception layer to improve it.
  Also, there are some parameters that should be global(e.g. vehicle size, max steering, etc.). These will be refactored and defined as global parameters so that we can share the same parameters between different nodes.
-->
