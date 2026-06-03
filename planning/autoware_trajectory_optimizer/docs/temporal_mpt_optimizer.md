# Temporal MPT Optimizer

## Overview

The **TrajectoryTemporalMPTOptimizer** plugin refines the incoming trajectory using a fixed-horizon **acados** MPC for a temporal kinematic bicycle model. Unlike the spatial [MPT optimizer](mpt_optimizer.md) (QP with corridor bounds from `autoware_path_optimizer`), this plugin tracks a **time-ordered** reference sequence: stage `k` uses trajectory point `start_idx + k` without arc-length resampling.

States are \((x, y, \psi, v)\); controls are longitudinal acceleration and steering command \((a, \delta)\). The solver is generated at build time from `src/acados_mpt/generators/path_tracking_mpc_temporal.py`.

## Horizon (codegen)

| Quantity               | Value               |
| ---------------------- | ------------------- |
| Horizon length \(T_f\) | 8.0 s               |
| Stages \(N\)           | 80                  |
| Time step \(\Delta t\) | 0.1 s (\(T_f / N\)) |

These values are fixed in the Python OCP definition; changing them requires regenerating the acados solver and rebuilding.

## Enabling the plugin

Two steps are required:

1. Add the plugin class to `plugin_names` in `config/trajectory_optimizer.param.yaml` (position sets when it runs relative to other plugins).
2. Set `use_temporal_mpt_optimizer: true` (runtime gate inside `optimize_trajectory()`).

To disable completely, remove the class from `plugin_names`. Setting only `use_temporal_mpt_optimizer: false` skips optimization but still loads and initializes acados if the plugin remains in the list.

Example:

```yaml
plugin_names:
  - "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother"
  - "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryTemporalMPTOptimizer"

use_temporal_mpt_optimizer: true
```

Plugin-specific parameters live in `config/plugins/trajectory_temporal_mpt_optimizer.param.yaml` (loaded from `launch/trajectory_optimizer.launch.xml`).

## Build requirements

- [acados](https://github.com/acados/acados) installed and `ACADOS_SOURCE_DIR` set (see Autoware ansible `setup_acados` role).
- At configure/build time, Python codegen runs via `src/acados_mpt/CMakeLists.txt` and produces `c_generated_code/` in the build tree.

## Parameters

| Parameter                       | Default                               | Description                                                                                                          |
| ------------------------------- | ------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `min_points_for_optimization`   | `2`                                   | Minimum trajectory points required before running MPC                                                                |
| `enable_debug_info`             | `false`                               | Extra `RCLCPP_INFO` / `WARN` on solve success or failure                                                             |
| `publish_debug_topics`          | `false`                               | Publish reference, odometry, output trajectory, solve status, and control debug topics under `~/debug/temporal_mpt/` |
| `write_replay_fixture`          | `false`                               | Write a text fixture for offline replay (e.g. Python generators)                                                     |
| `replay_fixture_directory`      | `~/.ros/autoware_temporal_mpt_replay` | Output directory for replay fixtures                                                                                 |
| `log_replay_fixture_to_console` | `false`                               | Log fixture path and contents to the console                                                                         |

Bicycle CG distances `lf` / `lr` are fixed at **1.0 m** in code to match the generated OCP (`ocp.parameter_values`).

## Debug visualization

With `publish_debug_topics: true`, run:

```bash
ros2 run autoware_trajectory_optimizer temporal_mpt_debug_visualizer.py
```

See `scripts/temporal_mpt_debug_visualizer.py` for topic names and options.

## Known limitations

- Only the first \(N+1\) trajectory points are overwritten with the MPC state trajectory; the rest of the message is unchanged.
- On solver failure (`status != 0`), the plugin returns without modifying `traj_points` (debug topics may still publish the last iterate if enabled).
- Reference yaw uses a \(2\pi\) bias so heading cost aligns with the initial state when planner yaw branches differ.
- Requires a successful acados build; the plugin creates the solver at initialization when loaded.
