# Autoware Traffic Light RViz Plugin

This package provides a RViz2 plugin for visualizing traffic light recognition results in Autoware.

![image](visualization.png)

## Property Description

### Topic Settings

- **Lanelet Map Topic**: Topic to receive Lanelet2 map data
  - Type: `autoware_map_msgs/msg/LaneletMapBin`
- **Traffic Light Topic**: Topic to receive traffic light recognition results
  - Type: `autoware_perception_msgs/msg/TrafficLightGroupArray`

### Display Settings

- **Timeout**: Time in seconds before clearing traffic light state display
- **Show Text**: Toggle text display of traffic light states
- **Show Bulb**: Toggle visual representation of traffic lights
- **Text Prefix**: Prefix for traffic light state text
- **Font Size**: Size of the traffic light state text
- **Text Color**: Color of the traffic light state text

### Text Position Adjustment

- **Text X Offset**: Offset for text display along X-axis
- **Text Y Offset**: Offset for text display along Y-axis
- **Text Z Offset**: Offset for text display along Z-axis
