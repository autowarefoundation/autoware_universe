# autoware_radar_objects_adapter

## Purpose

This package converts `autoware_sensing_msgs::msg::RadarObjects` into `autoware_perception_msgs::msg::DetectedObjects`, acting as a simple integration of radars into the perception pipeline.

## RadarObjectsAdapter

A node that converts radar objects from the sensing definition into a perception friendly format with no filtering involved.

### Inputs / Outputs

#### Input

| Name               | Type                                     | Description                                |
| ------------------ | ---------------------------------------- | ------------------------------------------ |
| ~/input/objects    | autoware_sensing_msgs::msg::RadarObjects | Input radar objects as defined in sensing. |
| ~/input/radar_info | autoware_sensing_msgs::msg::RadarInfo    | Input radar info.                          |

#### Output

| Name             | Type                                           | Description                                    |
| ---------------- | ---------------------------------------------- | ---------------------------------------------- |
| ~/output/objects | autoware_perception_msgs::msg::DetectedObjects | Output radar objects in the perception format. |

## Parameters

{{ json_to_markdown("sensing/autoware_radar_objects_adapter/schema/radar_objects_adapter.schema.json") }}
