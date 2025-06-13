`autoware_traffic_light_category_merger`

## Overview

`autoware_traffic_light_category_merger` receives the Traffic Light (TL) classification result from Car/Pedestrian classifiers to merge into single classification result. The expect ROIs TL without classification result will be filled as Unknown.

## Input topics

| Name                       | Type                                          | Description                   |
| -------------------------- | --------------------------------------------- | ----------------------------- |
| `input/car_signals`        | tier4_perception_msgs::msg::TrafficLightArray | Car TLs classification        |
| `input/pedestrian_signals` | tier4_perception_msgs::msg::TrafficLightArray | Pedestrian TLs classification |

## Output topics

| Name                     | Type                                          | Description                           |
| ------------------------ | --------------------------------------------- | ------------------------------------- |
| `output/traffic_signals` | tier4_perception_msgs::msg::TrafficLightArray | Car and Pedestrian TLs classification |

## Node parameters

N/A
