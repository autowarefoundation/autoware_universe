# autoware_dummy_traffic_light_publisher

## Purpose

Publish dummy traffic light signals for simulation environments where a traffic light recognition module is not available.

## Modes

| Mode         | Description                                                                                    |
| ------------ | ---------------------------------------------------------------------------------------------- |
| `standalone` | Cycles through Green -> Yellow -> Red for all traffic lights found in the vector map.          |
| `empty`      | Publishes an empty `TrafficLightGroupArray` (no traffic light groups).                         |
| `fixed`      | Publishes a single fixed color (`fixed_color`) for all traffic lights found in the vector map. |

To assign different signals per traffic light ID, use pass-through instead (see below); `fixed` mode intentionally applies one color to every light.

## Pass-through

When a message is received on the input topic (`~/input/traffic_signals`), the node relays it as-is instead of generating its own signals. If no new input arrives within `passthrough_timeout` seconds, the node falls back to its configured mode.

## Interface

### Subscriptions

| Topic                     | Type                                                  | Description                                                   |
| ------------------------- | ----------------------------------------------------- | ------------------------------------------------------------- |
| `~/input/vector_map`      | `autoware_map_msgs/msg/LaneletMapBin`                 | Lanelet2 map to extract traffic light regulatory element IDs. |
| `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | External traffic light signals for pass-through.              |

### Publications

| Topic                      | Type                                                  | Description                                 |
| -------------------------- | ----------------------------------------------------- | ------------------------------------------- |
| `~/output/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | Generated or relayed traffic light signals. |

## Parameters

| Parameter             | Type   | Default   | Description                                                           |
| --------------------- | ------ | --------- | --------------------------------------------------------------------- |
| `mode`                | string | `"empty"` | Operating mode: `"standalone"`, `"empty"` or `"fixed"`.               |
| `publish_rate`        | double | `10.0`    | Publishing frequency [Hz].                                            |
| `green_duration`      | double | `30.0`    | Duration of the green phase [s] (`standalone` mode).                  |
| `yellow_duration`     | double | `3.0`     | Duration of the yellow phase [s] (`standalone` mode).                 |
| `red_duration`        | double | `30.0`    | Duration of the red phase [s] (`standalone` mode).                    |
| `passthrough_timeout` | double | `1.0`     | Time after last input before falling back to the configured mode [s]. |
| `fixed_color`         | string | `"red"`   | Color published in `fixed` mode: `"green"`, `"yellow"` or `"red"`.    |

## Usage

### Launch

```bash
ros2 launch autoware_dummy_traffic_light_publisher dummy_traffic_light_publisher.launch.xml
```

To override the mode:

```bash
ros2 launch autoware_dummy_traffic_light_publisher dummy_traffic_light_publisher.launch.xml mode:=standalone
```

To publish a fixed color for all traffic lights:

```bash
ros2 launch autoware_dummy_traffic_light_publisher dummy_traffic_light_publisher.launch.xml mode:=fixed fixed_color:=green
```

## Design and extension tactics

This package separates concerns into three layers:

| Layer             | Class                            | Role                                                                                       |
| ----------------- | -------------------------------- | ------------------------------------------------------------------------------------------ |
| ROS I/O           | `DummyTrafficLightPublisherNode` | Subscriptions (`take()`), timer, publisher, vector map parsing.                            |
| Message assembly  | `DummyTrafficLight`              | Pass-through judgment, traffic light ID management, `TrafficLightGroupArray` construction. |
| Signal generation | `TrafficLightCycle`              | Phase computation (Green/Yellow/Red) from elapsed time and `TrafficLightElement` output.   |

When extending, modify only the layer that owns the responsibility:

- **Adding arrow shapes, flashing patterns, or new signal types** — change `TrafficLightCycle`. It owns `TrafficLightElement` construction. Node and `DummyTrafficLight` are unaffected.
- **Per-intersection or per-ID signal control** — change `DummyTrafficLight`. It maps IDs to elements. `TrafficLightCycle` and Node are unaffected.
- **Adding new input sources or output topics** — change `DummyTrafficLightPublisherNode`. Logic layers are unaffected.

### Run node directly

The parameters have no in-code defaults, so a parameter file must be provided (e.g. the packaged `config/dummy_traffic_light_publisher.param.yaml`):

```bash
ros2 run autoware_dummy_traffic_light_publisher autoware_dummy_traffic_light_publisher_node --ros-args \
  --params-file $(ros2 pkg prefix --share autoware_dummy_traffic_light_publisher)/config/dummy_traffic_light_publisher.param.yaml \
  -p mode:=standalone \
  -r ~/input/vector_map:=/map/vector_map \
  -r ~/input/traffic_signals:=/simulator/input/traffic_signals \
  -r ~/output/traffic_signals:=/perception/traffic_light_recognition/traffic_signals
```
