# bbox_object_locator

## Inner Workings / Algorithms

This node produces 3D object detection from a 2D image bounding box (bbox) through the following steps:

1. Back-project the bottom-center pixel of a bbox onto the ground plane (`z=0` in the target coordinate system). This point is used as the object's bottom center.
2. Back-project the top-center pixel of the bbox to a point near the one computed in step 1 for height estimation.
3. Back-project the bottom-left and bottom-right pixels of the bbox to estimate the object’s width (diameter).
4. Use these four points to determine the object’s position and size.

## Parameters

{{ json_to_markdown("perception/autoware_image_object_locator/schema/bbox_object_locator.schema.json") }}

## Input

| Name                    | Type                                                   | Description        |
| ----------------------- | ------------------------------------------------------ | ------------------ |
| `input/rois<id>`        | tier4_perception_msgs::msg::DetectedObjectsWithFeature | <id>'s input ROI   |
| `input/camera_info<id>` | sensor_msgs::msg::CameraInfo                           | <id>'s camera info |

## Output

| Name                      | Type                                           | Description                             |
| ------------------------- | ---------------------------------------------- | --------------------------------------- |
| `output/rois<id>/objects` | autoware_perception_msgs::msg::DetectedObjects | The object generated from <id>'s 2D ROI |
