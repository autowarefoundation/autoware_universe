# autoware_tensorrt_oneplanner

## Purpose

`autoware_tensorrt_oneplanner` runs **OnePlanner**, a one-stage end-to-end driving model that
combines a BEVFusion LiDAR encoder with a diffusion-planner head. Instead of the conventional
two-stage pipeline (3D detection → tracked objects → learned planner), the planner head is
conditioned directly on the LiDAR BEV feature map through a cross-attention bridge, so no
decoded boxes or tracked-object input is needed.

Training lives in [tier4/OnePlanner](https://github.com/tier4/OnePlanner). Only the
**LiDAR-only** variant is supported (no cameras).

## Architecture

The model is deployed as two TensorRT engines chained on-device (the BEV feature map never
leaves the GPU):

```text
pointcloud ─► densification ─► voxelization ─► [BEV encoder engine] ─► bev_feature_map [1,512,180,180]
                                                                            │ (device-to-device)
odometry / acceleration / route / vector map /                              ▼
turn indicators / traffic lights ─► planner input tensors ─► [planner engine] ─► prediction [1,1,80,4]
                                                                                  turn_indicator_logit [1,5]
```

- **BEV encoder engine**: the BEVFusion lidar branch (VFE → sparse encoder → SECOND →
  SECOND-FPN). Same inputs as the `autoware_bevfusion` lidar-only model
  (`voxels` / `num_points_per_voxel` / `coors`), but outputs the dense BEV feature map
  instead of detections. Preprocessing (densification + voxelization CUDA kernels) is reused
  from `autoware_bevfusion`. Requires the spconv plugins from `autoware_tensorrt_plugins`.
- **Planner engine**: the diffusion-planner head plus the BEV cross-attention bridge. Consumes
  the same conditioning tensors as `autoware_diffusion_planner` (lanes, route, goal pose,
  ego state/history, turn indicators, …) plus `bev_feature_map`. The `neighbor_agents_past`
  tensor is fed as zeros — the neighbor encoder was removed at training time and the input is
  kept only for graph compatibility. Preprocessing and postprocessing are reused from
  `autoware_diffusion_planner`.

Inference is driven by the pointcloud callback (LiDAR rate, typically 10 Hz).

The prediction output is ego-only (`P=1`); unlike the diffusion planner there is no
neighbor-prediction head, so this node does not publish predicted objects.

## Inputs / Outputs

### Input

| Name                      | Type                                                    | Description                     |
| ------------------------- | ------------------------------------------------------- | ------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs/msg/PointCloud2` (cuda_blackboard-capable) | Concatenated LiDAR pointcloud   |
| `~/input/odometry`        | `nav_msgs/msg/Odometry`                                 | Ego kinematic state             |
| `~/input/acceleration`    | `geometry_msgs/msg/AccelWithCovarianceStamped`          | Ego acceleration                |
| `~/input/route`           | `autoware_planning_msgs/msg/LaneletRoute`               | Route                           |
| `~/input/vector_map`      | `autoware_map_msgs/msg/LaneletMapBin`                   | Vector map                      |
| `~/input/turn_indicators` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport`        | Turn indicator state            |
| `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray`   | Traffic light states (optional) |

### Output

| Name                       | Type                                                        | Description                  |
| -------------------------- | ----------------------------------------------------------- | ---------------------------- |
| `~/output/trajectory`      | `autoware_planning_msgs/msg/Trajectory`                     | Planned ego trajectory (8 s) |
| `~/output/trajectories`    | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Candidate trajectories       |
| `~/output/turn_indicators` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`           | Turn indicator command       |

## Model artifacts

Place the following under `$(env HOME)/autoware_data/ml_models/oneplanner/`:

| File                          | Description                                              |
| ----------------------------- | -------------------------------------------------------- |
| `oneplanner_bev_encoder.onnx` | BEVFusion lidar branch → BEV feature map                 |
| `oneplanner_planner.onnx`     | Planner head (with the diffusion sampling loop unrolled) |
| `oneplanner.param.json`       | Normalization statistics + `major_version`               |

Both ONNX files are produced by `scripts/export_onnx.py` in the
[tier4/OnePlanner](https://github.com/tier4/OnePlanner) repository from a trained E2E
checkpoint (run with `uv run` inside that repository):

```bash
# Planner head + normalization json (self-contained)
uv run --with onnx --with onnxscript --with onnxsim --with onnxruntime \
  python scripts/export_onnx.py planner \
  --ckpt <e2e_ckpt> --output-dir ~/autoware_data/ml_models/oneplanner --device cpu

# BEV encoder (spconv ONNX symbolics come from an AWML checkout)
uv run --extra gpu --with onnx --with onnxscript --with onnxsim --with onnxruntime \
  python scripts/export_onnx.py bev-encoder \
  --ckpt <e2e_ckpt> --perception-config configs/perception/model/<matching_config>.yaml \
  --awml-path <AWML checkout> --point-dim 5 \
  --output-dir ~/autoware_data/ml_models/oneplanner --device cuda
```

`use_intensity` (and `--point-dim`) must match the checkpoint's point dimension —
the jpntaxi checkpoint uses 5-dim points (x, y, z, intensity, time_lag).

TensorRT engines are built on the first launch (or with `build_only:=true`)
and cached next to the ONNX files.

```bash
ros2 launch autoware_tensorrt_oneplanner oneplanner.launch.xml build_only:=true
```

## Parameters

See `config/oneplanner.param.yaml`. The LiDAR preprocessing parameters
(`point_cloud_range`, `voxel_size`, `voxels_num`, …) must match the training configuration
(±122.4 m, 0.17 m voxels, 4-dim points without intensity — identical to the
`autoware_bevfusion` lidar-only deployment).

## Limitations

- `batch_size` is fixed to 1 (the BEV feature map is produced once per frame).
- LiDAR-only; the camera branch is not deployed.
- Requires CUDA, TensorRT, and spconv (the package is skipped at build time otherwise).
