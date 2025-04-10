# autoware_traffic_light_map_based_detector

## Overview

`autoware_traffic_light_map_based_detector` calculates where the traffic lights will appear in the image based on the HD map.

Calibration and vibration errors can be entered as parameters, and the size of the detected RegionOfInterest will change according to the error.

![traffic_light_map_based_detector_result](./docs/traffic_light_map_based_detector_result.svg)

If the node receives route information, it only looks at traffic lights on that route.
If the node receives no route information, it looks at them within a radius of `max_detection_range` and the angle between the traffic light and the camera is less than `car_traffic_light_max_angle_range` or `pedestrian_traffic_light_max_angle_range`.

## Input topics

| Name                  | Type                                      | Description             |
| --------------------- | ----------------------------------------- | ----------------------- |
| `~/input/vector_map`  | autoware_map_msgs::msg::LaneletMapBin     | vector map              |
| `~/input/camera_info` | sensor_msgs::msg::CameraInfo              | target camera parameter |
| `~/input/route`       | autoware_planning_msgs::msg::LaneletRoute | optional: route         |

## Output topics

| Name              | Type                                             | Description                                                               |
| ----------------- | ------------------------------------------------ | ------------------------------------------------------------------------- |
| `~/output/rois`   | tier4_perception_msgs::msg::TrafficLightRoiArray | location of traffic lights in image corresponding to the camera info      |
| `~/expect/rois`   | tier4_perception_msgs::msg::TrafficLightRoiArray | location of traffic lights in image without any offset                    |
| `~/debug/markers` | visualization_msgs::msg::MarkerArray             | markers which show a line that combines from camera to each traffic light |

## Node parameters

{{ json_to_markdown("perception/autoware_traffic_light_map_based_detector/schema/traffic_light_map_based_detector.schema.json") }}
