# `autoware_radar_object_tracker`

## Purpose

This package provides a radar object tracking node that processes sequences of detected objects to assign consistent identities to them and estimate their velocities.

## Inner-workings / Algorithms

This radar object tracker is a combination of data association and tracking algorithms.

<!-- In the future, you can add an overview image here -->
<!-- ![radar_object_tracker_overview](image/radar_object_tracker_overview.svg) -->

### Data Association

The data association algorithm matches detected objects to existing tracks.

### Tracker Models

The tracker models used in this package vary based on the class of the detected object.
See more details in the [models.md](models.md).

<!-- In the future, you can add flowcharts, state transitions, and other details about how this package works. -->

## Inputs / Outputs

### Input

| Name          | Type                                             | Description      |
| ------------- | ------------------------------------------------ | ---------------- |
| `~/input`     | `autoware_perception_msgs::msg::DetectedObjects` | Detected objects |
| `/vector/map` | `autoware_map_msgs::msg::LaneletMapBin`          | Map data         |

### Output

| Name       | Type                                            | Description     |
| ---------- | ----------------------------------------------- | --------------- |
| `~/output` | `autoware_perception_msgs::msg::TrackedObjects` | Tracked objects |

## Parameters

{{ json_to_markdown("perception/autoware_radar_object_tracker/schema/data-association_matrix.schema.json") }}

{{ json_to_markdown("perception/autoware_radar_object_tracker/schema/default_tracker.schema.json") }}

{{ json_to_markdown("perception/autoware_radar_object_tracker/schema/radar_object_tracker.schema.json") }}

{{ json_to_markdown("perception/autoware_radar_object_tracker/schema/simulation_tracker.schema.json") }}

## Assumptions / Known limits

<!-- In the future, you can add assumptions and known limitations of this package. -->

## (Optional) Error detection and handling

<!-- In the future, you can add details about how this package detects and handles errors. -->

## (Optional) Performance characterization

<!-- In the future, you can add details about the performance of this package. -->

## (Optional) References/External links

<!-- In the future, you can add references and links to external code used in this package. -->

## (Optional) Future extensions / Unimplemented parts

<!-- In the future, you can add details about planned extensions or unimplemented parts of this package. -->
