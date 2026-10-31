# autoware_radar_objects_adapter

## Purpose

This package converts `autoware_sensing_msgs::msg::RadarObjects` into `autoware_perception_msgs::msg::DetectedObjects` and `autoware_perception_msgs::msg::TrackedObjects`, acting as a simple integration of radars into the perception pipeline.

## RadarObjectsAdapter

A node that converts radar objects from the sensing definition into a perception friendly format with no filtering involved.

### What the input is expected to be

The node converts the message format; it does not transform coordinates, compensate the ego motion, or synchronize anything. It therefore expects the radar driver to publish `RadarObjects` in the form the perception pipeline uses:

- Positions, orientations, sizes and their covariances are expressed in the frame named in `header.frame_id`, which is the frame the output objects are published in (typically `base_link`).
- Velocities and accelerations are absolute (ground-relative) and expressed in that same frame, as the driver reports them; the node rotates them by the object's orientation so that the output twist and acceleration are expressed in the object's own frame, as Autoware's object messages require. The `absolute_dynamics` flag of `RadarInfo` is not checked.
- The `RadarInfo` message declares the object fields the radar provides. The following fields are required, and objects are dropped until a `RadarInfo` that declares all of them has arrived: `existence_probability`, `position_x`, `position_y`, `velocity_x`, `velocity_y`, `acceleration_x`, `acceleration_y`, `orientation`.
- Fields the `RadarInfo` does not declare are filled from the `default_*` parameters: `position_z`, `velocity_z`, `acceleration_z`, `size_x`, `size_y`, `size_z`. The yaw variances of the pose and the twist are set only when `orientation_std` and `orientation_rate_std` are declared.

### Parameter: classification_remap

This parameter allows remapping of classification labels from `autoware_sensing_msgs::msg::RadarClassification` to `autoware_perception_msgs::msg::ObjectClassification`. It should be provided as a flat list of strings, where each pair of strings represents an input label and the corresponding output label.

For example, the current default configuration remaps `MOTORCYCLE` and `BICYCLE` from radar classification to `CAR` in the perception classification, while keeping other labels unchanged.

The `probability` of each radar classification is treated as an independent confidence for that label, not as a share of a distribution, and is copied as it is. Consequently, if multiple radar labels are remapped to the same perception label, multiple entries with that label and their own probabilities appear in the output; the probabilities are not merged. This does not violate any logic in Autoware but may be worth monitoring.

### Inputs / Outputs

#### Input

| Name               | Type                                     | Description                                |
| ------------------ | ---------------------------------------- | ------------------------------------------ |
| ~/input/objects    | autoware_sensing_msgs::msg::RadarObjects | Input radar objects as defined in sensing. |
| ~/input/radar_info | autoware_sensing_msgs::msg::RadarInfo    | Input radar info.                          |

#### Output

| Name                | Type                                           | Description                                                                                            |
| ------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| ~/output/detections | autoware_perception_msgs::msg::DetectedObjects | Output radar objects as detections.                                                                    |
| ~/output/tracks     | autoware_perception_msgs::msg::TrackedObjects  | Output radar objects as tracks, keeping the radar's object ids (made unique per input topic as UUIDs). |

## Parameters

{{ json_to_markdown("sensing/autoware_radar_objects_adapter/schema/radar_objects_adapter.schema.json") }}
