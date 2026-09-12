# autoware_traffic_light_pipeline

Single-node composition of the traffic light recognition pipeline.

## Node (`traffic_light_recognition`)

### Input / Output

| Direction  | Topic                      | Type                                             |
| ---------- | -------------------------- | ------------------------------------------------ |
| Subscribed | `~/input/image`            | `sensor_msgs/msg/Image`                          |
| Subscribed | `~/input/camera_info`      | `sensor_msgs/msg/CameraInfo`                     |
| Subscribed | `~/input/vector_map`       | `autoware_map_msgs/msg/LaneletMapBin`            |
| Subscribed | `~/input/route`            | `autoware_planning_msgs/msg/LaneletRoute`        |
| Published  | `~/output/traffic_signals` | `tier4_perception_msgs/msg/TrafficLightArray`    |
| Published  | `~/output/rois`            | `tier4_perception_msgs/msg/TrafficLightRoiArray` |

### Node parameters

{{ json_to_markdown("perception/autoware_traffic_light_pipeline/schema/traffic_light_recognition.schema.json") }}

## Prerequisites

The ML models used by this pipeline (traffic light detector / classifier) must be downloaded in advance to `~/autoware_data`. See [Manual downloading of artifacts](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#manual-downloading-of-artifacts) for how to download them.

The default `data_path` launch argument (see below) points to `~/autoware_data`, so no additional configuration is needed once the models are placed there.

## How to launch

```bash
ros2 launch autoware_traffic_light_pipeline traffic_light_recognition.launch.xml
```

Useful launch arguments:

| Argument      | Default                     | Description                                                        |
| ------------- | --------------------------- | ------------------------------------------------------------------ |
| `data_path`   | `$(env HOME)/autoware_data` | Directory containing the downloaded ML artifacts                   |
| `camera_name` | `camera6`                   | Which camera namespace this instance subscribes to / publishes for |
| `build_only`  | `false`                     | Exit after the TensorRT engine is built                            |

Example, running against a second camera:

```bash
ros2 launch autoware_traffic_light_pipeline traffic_light_recognition.launch.xml camera_name:=camera7
```

## How to test

```bash
PACKAGE_NAME=autoware_traffic_light_pipeline
colcon build --packages-select $PACKAGE_NAME
colcon test --packages-select $PACKAGE_NAME --event-handlers console_cohesion+
```

The unit/integration tests also require the ML models under `~/autoware_data`, since `test_traffic_light_recognition_node` brings up the node with the same `ml_model_path` default as the launch file above.
