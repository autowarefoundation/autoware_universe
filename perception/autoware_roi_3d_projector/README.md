# autoware_roi_3d_projector

## Purpose

The `autoware_roi_3d_projector` package is designed to use objects detected from 2D images to generate 3D object detections for more robust perception.

## Inner Workings / Algorithms

This package produces 3D object detections through the following steps:

1. Back-project the bottom-center pixel of a ROI onto the ground plane (`z=0` in the target coordinate system). This point is used as the object's bottom center.
2. Back-project the top-center pixel of the ROI to a point near the one computed in step 1 for height estimation.
3. Back-project the bottom-left and bottom-right pixels of the ROI to estimate the object’s width (diameter).
4. Use these four points to determine the object’s position and size.

## Parameters

{{ json_to_markdown("perception/autoware_roi_3d_projector/schema/roi_3d_projector.schema.json") }}

## Input

| Name                          | Type                                                   | Description        |
| ----------------------------- | ------------------------------------------------------ | ------------------ |
| `input/rois<id>`              | tier4_perception_msgs::msg::DetectedObjectsWithFeature | <id>'s input ROI   |
| `camera/rois<id>/camera_info` | sensor_msgs::msg::CameraInfo                           | <id>'s camera info |

## Output

| Name                      | Type                                           | Description                             |
| ------------------------- | ---------------------------------------------- | --------------------------------------- |
| `output/rois<id>/objects` | autoware_perception_msgs::msg::DetectedObjects | The object generated from <id>'s 2D ROI |
