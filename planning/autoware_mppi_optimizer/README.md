# Autoware MPPI Optimizer

The `autoware_mppi_optimizer` package optimizes trajectories using Model Predictive Path Integral (MPPI) control.

## Overview

This package depends on the external [mppi_generic_vendor](https://github.com/autowarefoundation/mppi_generic_vendor) package (`MPPI-Generic`) and ships first-order Dubins path-tracking extensions under `include/mppi/`. A CUDA library interface (`FirstOrderDubinsMppiInterface`) wraps the two-lane double-park MPPI example for use from C++ / ROS nodes.

### Layout

```text
autoware_mppi_optimizer/
├── include/mppi/         # First-order Dubins dynamics, cost, path utilities
├── include/autoware/mppi_optimizer/
│   └── first_order_dubins_mppi_interface.hpp
└── src/first_order_dubins/
    └── first_order_dubins_mppi_interface.cu
```



### Requirements

- CUDA Toolkit (curand, cufft)
- Eigen3



## Inputs / Outputs


| Topic                 | Type                                    | Description          |
| --------------------- | --------------------------------------- | -------------------- |
| `~/input/trajectory`  | `autoware_planning_msgs/msg/Trajectory` | Reference trajectory |
| `~/input/odometry`    | `nav_msgs/msg/Odometry`                 | Current ego state    |
| `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Optimized trajectory |




## Launch

```bash
ros2 launch autoware_mppi_optimizer mppi_optimizer.launch.xml
```



## Offline debug logging + retune

Enable CSV logging from the diffusion planner / MPPI params:

```yaml
enable_debug_trajectory_log: true
# Empty -> $XDG_CACHE_HOME/autoware/mppi_debug_log or $HOME/.cache/autoware/mppi_debug_log
debug_trajectory_log_directory: ""
```

Each cycle writes:

```text
$HOME/.cache/autoware/mppi_debug_log/
  index.csv
  cost_params.csv
  vehicle_params.csv
  000000_reference.csv
  000000_optimized.csv
  000000_ego.csv
  ...
```

`*_ego.csv` stores the odometry / accel / steer initial condition used online.
Offline retune loads ego + cost/vehicle params from the log so a no-op retune can match
the logged MPPI (obstacles are still not replayed).

Various features can be disabled by changing the following parameters set in `mppi_optimizer.param.yaml`:

```yaml
ignore_obstacles: true
ignore_drivable_area: true
force_cold_start_each_step: true
```

Then rebuild / restart the diffusion planner and compare live MPPI to offline retune.

Notes:

- `ignore_obstacles` drops tracked objects before MPPI (matches offline's empty objects).
- `ignore_drivable_area` is retained as an ablation flag; on this stack boundary crash is already
  disabled in the cost (`isEgoOutsideDrivableArea` always false).
- `force_cold_start_each_step` only resets tracking counters / arc-length (control is already
re-seeded from the reference via `updateImportanceSampler(u_nom)` each cycle).



### Replay only

```bash
ros2 run autoware_diffusion_planner mppi_debug_visualizer.py -- \
  --log-dir "$HOME/.cache/autoware/mppi_debug_log"
```



### Batch retune (CLI)

```bash
ros2 run autoware_mppi_optimizer mppi_offline_retune -- \
  --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
  --out-dir "$HOME/.cache/autoware/mppi_retune" \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml \
  --set track_coeff=2000 --set steer_rate_coeff=5000 \
  --copy-reference
```



### Interactive compare + retune

Same plots as `mppi_debug_visualizer.py` (XY, heading, velocity, accel, steer, steer-rate),
with diffusion reference (cyan), logged MPPI (red), and retuned MPPI (green):

```bash
# Option A — visualizer with retune panel
ros2 run autoware_diffusion_planner mppi_debug_visualizer.py -- \
  --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
  --enable-retune \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml

# Option B — wrapper alias
ros2 run autoware_mppi_optimizer mppi_offline_tuner.py -- \
  --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml
```

Adjust sliders, press **Retune** (or `r`). Overlay updates in place; metrics show max position/velocity error vs the reference.