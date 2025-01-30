# autoware_simpl

## Purpose

The `autoware_simpl` is used for 3D object motion prediction based on ML-based model called SIMPL.

## Inner-workings / Algorithms

The implementation bases on SIMPL [1] work. It uses TensorRT library for data process and network interface.

## Inputs / Outputs

### Inputs

| Name                 | Type                                            | Description              |
| -------------------- | ----------------------------------------------- | ------------------------ |
| `~/input/objects`    | `autoware_perception_msgs::msg::TrackedObjects` | Input agent state.       |
| `~/input/vector_map` | `autoware_map_msgs::msg::LeneletMapBin`         | Input vector map.        |
| `~/input/ego`        | `sensor_msgs::msg::Odometry`                    | Input ego vehicle state. |

### Outputs

| Name               | Type                                              | Description                |
| ------------------ | ------------------------------------------------- | -------------------------- |
| `~/output/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Predicted objects' motion. |

## Parameters

TBD
