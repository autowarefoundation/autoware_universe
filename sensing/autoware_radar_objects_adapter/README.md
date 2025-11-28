# autoware_radar_objects_adapter

## Purpose

This package converts `autoware_sensing_msgs::msg::RadarObjects` into `autoware_perception_msgs::msg::DetectedObjects`, acting as a simple integration of radars into the perception pipeline.

## RadarObjectsAdapter

A node that converts radar objects from the sensing definition into a perception friendly format with no filtering involved.

### Parameter: classification_remap

This parameter allows remapping of classification labels from `autoware_sensing_msgs::msg::RadarClassification` to `autoware_perception_msgs::msg::ObjectClassification`. It should be provided as a flat list of strings, where each pair of strings represents an input label and the corresponding output label.

For example, the current default configuration remaps `MOTORCYCLE` and `BICYCLE` from radar classification to `CAR` in the perception classification, while keeping other labels unchanged.

**Note**: If multiple radar labels are remapped to the same perception label, multiple probabilities may appear for that label.
This does not violate any logic in Autoware but may be worth monitoring.

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
