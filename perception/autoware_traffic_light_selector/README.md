autoware_traffic_light_selector

## Overview

`autoware_traffic_light_selector` selects the interest traffic light from the list of accurately detected traffic lights by something (e.g. deep learning neural network) based on the expect ROIs and rough ROIs information and then assign traffic_light_id for them.

## Input topics

| Name                  | Type                                                   | Description                                                          |
| --------------------- | ------------------------------------------------------ | -------------------------------------------------------------------- |
| `input/detected_rois` | tier4_perception_msgs::msg::DetectedObjectsWithFeature | accurately detected traffic light                                    |
| `input/rough_rois`    | tier4_perception_msgs::msg::TrafficLightRoiArray       | location of traffic lights in image corresponding to the camera info |
| `input/expect_rois`   | tier4_perception_msgs::msg::TrafficLightRoiArray       | location of traffic lights in image without any offset               |

## Output topics

| Name                  | Type                                        | Description                                |
| --------------------- | ------------------------------------------- | ------------------------------------------ |
| `output/traffic_rois` | tier4_perception_msgs::TrafficLightRoiArray | detected traffic light of interest with id |

## Node parameters

None
