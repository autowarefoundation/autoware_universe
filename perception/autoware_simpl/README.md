# autoware_simpl

## Purpose

The `autoware_simpl` is used for 3D object motion prediction based on ML-based model called SIMPL.

## Inner-workings / Algorithms

The implementation bases on SIMPL [1] work. It uses TensorRT library for data process and network interface.

### Inputs Representation

- $X_A\in R^{N\times T_{past}\times D_{agent}}$: Agent histories input.
- $X_M\in R^{K\times (P-1)\times D_{map}}$: Map points input.
- $X_{RPE}\in R^{(N+K)\times (N+K)\times D_{rpe}}$: Relative pose encoding input.

### Outputs Representation

- $P_{score}\in R^{N\times M}$: Predicted scores for each agent and mode.
- $P_{trajectory}\in R^{N\times M\times T_{future}\times D_{trajectory}}$: Predicted trajectories for each agent and mode.

## Inputs / Outputs

### Inputs

| Name                 | Type                                            | Description              |
| -------------------- | ----------------------------------------------- | ------------------------ |
| `~/input/objects`    | `autoware_perception_msgs::msg::TrackedObjects` | Input tracked agents.    |
| `~/input/vector_map` | `autoware_map_msgs::msg::LeneletMapBin`         | Input vector map.        |
| `~/input/ego`        | `sensor_msgs::msg::Odometry`                    | Input ego vehicle state. |

### Outputs

| Name               | Type                                              | Description               |
| ------------------ | ------------------------------------------------- | ------------------------- |
| `~/output/objects` | `autoware_perception_msgs::msg::PredictedObjects` | Predicted agents' motion. |

## Parameters

{{ json_to_markdown("perception/autoware_simpl/schema/simpl.scheme.json") }}

## Assumptions / Known limits

### Number of predicted agents

We have not supported the dynamic shape inference yet. Therefore, the number of predicted agents must be fixed as `processing.max_num_agent` ($N$).
This value is determined when exporting ONNX.

Note that the following parameters are also determined when exporting ONNX:

- `processing.num_past`: $T_{past}$
- `processing.max_num_polyline`: $K$
- `processing.max_num_point`: $P$
- `processing.num_mode`: $M$
- `processing.num_future`: $T_{future}$

## References / External Links

[1] Lu Zhang, Peiliang Li, Sikang Liu, and Shaojie Shen, "SIMPL: A Simple and Efficient Multi-agent Motion Prediction Baseline for Autonomous Driving", arXiv preprint arXiv:2402.02519 (2024). <!-- cspell:disable-line -->

[2] <https://github.com/HKUST-Aerial-Robotics/SIMPL>
