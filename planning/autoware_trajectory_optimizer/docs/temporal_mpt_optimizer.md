# Temporal MPT Optimizer

## Overview

The **TrajectoryTemporalMPTOptimizer** plugin refines the incoming trajectory using a fixed-horizon **acados** MPC for a temporal kinematic bicycle model. Unlike the spatial [MPT optimizer](mpt_optimizer.md) (QP with corridor bounds from `autoware_path_optimizer`), this plugin tracks a **time-ordered** reference sequence: stage `k` uses trajectory point `start_idx + k` without arc-length resampling.

States are \((x, y, \psi, v)\); controls are longitudinal acceleration and steering command \((a, \delta)\). The solver is generated at build time from `src/acados_mpt/generators/path_tracking_mpc_temporal.py`.

## Role in the plugin pipeline

The default trajectory optimizer pipeline runs many plugins in sequence after basic cleanup:

```yaml
plugin_names:
  - "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother"
  - "autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother"
  - "autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryExtender"
```

**TrajectoryTemporalMPTOptimizer** is intended to replace that entire post-`TrajectoryPointFixer` chain with a single joint optimization step. When enabled, a minimal pipeline is:

```yaml
plugin_names:
  - "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_optimizer::plugin::TrajectoryTemporalMPTOptimizer"

use_temporal_mpt_optimizer: true
```

Turn off the plugins it supersedes (`use_qp_smoother`, `use_velocity_optimizer`, `use_eb_smoother`, `use_akima_spline_interpolation`, `use_kinematic_feasibility_enforcer`, `use_mpt_optimizer`, `use_trajectory_extender`) so they are not loaded in parallel.

### What each replaced plugin contributes

| Replaced plugin                            | Classical role                                                           | Absorbed by temporal MPC                                                                                          |
| ------------------------------------------ | ------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------- |
| **TrajectoryKinematicFeasibilityEnforcer** | Clamps yaw change per segment to Ackermann and yaw-rate limits           | Bicycle dynamics with bounded steering \(\delta\) and lateral-acceleration constraint \(a\_\mathrm{lat} = v^2 K\) |
| **TrajectoryQPSmoother**                   | Smooths \((x, y)\) via QP curvature penalty while tracking the reference | Running cost on state tracking \((x, y, \psi, v)\) plus control penalty on \((a, \delta)\)                        |
| **TrajectoryVelocityOptimizer**            | Jerk-filtered longitudinal velocity profile                              | Speed is a state; longitudinal acceleration is a control with box constraints                                     |
| **TrajectoryEBSmootherOptimizer**          | Elastic-band geometric path smoothing                                    | Implicit in jointly optimized \((x, y)\) trajectory over the MPC horizon                                          |
| **TrajectorySplineSmoother**               | Akima spline interpolation of path geometry                              | Not needed when MPC directly outputs a smooth state trajectory                                                    |
| **TrajectoryMPTOptimizer**                 | Spatial QP-MPT with adaptive corridor bounds                             | Replaced by this temporal acados solver (different formulation, no corridor bounds)                               |
| **TrajectoryExtender**                     | Backward extension from past ego odometry                                | **Not** performed by temporal MPT; keep only if backward extension is still required                              |

The incoming message from upstream planning is treated as the MPC reference. The plugin overwrites at most the first \(N+1\) points (8.0 s at 0.1 s spacing) with the optimized state and control trajectory.

## Horizon (codegen)

| Quantity               | Value               |
| ---------------------- | ------------------- |
| Horizon length \(T_f\) | 8.0 s               |
| Stages \(N\)           | 80                  |
| Time step \(\Delta t\) | 0.1 s (\(T_f / N\)) |

These values are fixed in the Python OCP definition; changing them requires regenerating the acados solver and rebuilding.

## MPC formulation

### Vehicle model

Kinematic bicycle in the world frame (`generators/bicycle_model_temporal.py`):

- States: \(x, y, \psi, v\)
- Controls: longitudinal acceleration \(a\), front steering \(\delta\) [rad]
- Parameters: \(l_f, l_r\) from `vehicle_info` via `cg_distance_from_rear_axle_ratio`

### Cost

LINEAR*LS tracking of the reference against \([x, y, \psi, v, a*\mathrm{ref}, \delta*\mathrm{ref}]\) with weights from `path_tracking_mpc_temporal.py` (\(Q\), \(R\), terminal \(Q_e\)). Control references \(a*\mathrm{ref}\) and \(\delta\_\mathrm{ref}\) are set to zero at runtime.

### Constraints

- Longitudinal acceleration and steering command box limits
- Nonlinear lateral-acceleration bound: \(|a\_\mathrm{lat}| \le 1.2\,\mathrm{m/s^2}\)
- Fixed initial state at stage 0

## Reference construction

Horizon references are built in `trajectory_temporal_mpt_optimizer_utils` before each solve:

1. **Initial state** \(x_0\) from the first trajectory point \((x, y, \psi, v)\).
2. **Closest-point start** `start_idx`: index whose XY position is nearest \(x_0\) (typically 0 when the ego is at `traj_points[0]`).
3. **Horizon sampling**: stage \(k\) uses point `min(start_idx + k, n_pts - 1)` — a time-ordered discrete sequence, not arc-length resampling.
4. **Yaw bias**: reference yaw is shifted by an integer multiple of \(2\pi\) so heading cost aligns with \(x_0.\psi\) when planner yaw branches differ.
5. **Ego-centered coordinates**: all position references and the state passed to the solver are translated by \(-x_0[:2]\); the solution is shifted back before writing to the output message.

Unit tests for this logic live in `tests/test_trajectory_temporal_mpt_optimizer_utils.cpp`.

## Enabling the plugin

Two steps are required:

1. Add the plugin class to `plugin_names` in `config/trajectory_optimizer.param.yaml` (position sets when it runs relative to other plugins).
2. Set `use_temporal_mpt_optimizer: true` (runtime gate inside `optimize_trajectory()`).

To disable completely, remove the class from `plugin_names`. Setting only `use_temporal_mpt_optimizer: false` skips optimization but still loads and initializes acados if the plugin remains in the list.

Plugin-specific parameters live in `config/plugins/trajectory_temporal_mpt_optimizer.param.yaml` (loaded from `launch/trajectory_optimizer.launch.xml`).

## Build requirements

- [acados](https://github.com/acados/acados) installed and `ACADOS_SOURCE_DIR` set (see Autoware ansible `setup_acados` role).
- At configure/build time, Python codegen runs via `src/acados_mpt/CMakeLists.txt` and produces `c_generated_code/` in the build tree.

## Parameters

| Parameter                          | Default                               | Description                                                                                                                                                                     |
| ---------------------------------- | ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cg_distance_from_rear_axle_ratio` | `0.8`                                 | CG distance from rear axle as a fraction of `wheel_base` (`vehicle_info`). `lr = ratio × wheel_base`, `lf = wheel_base − lr`. Applied at node startup only (restart to change). |
| `min_points_for_optimization`      | `2`                                   | Minimum trajectory points required before running MPC                                                                                                                           |
| `enable_debug_info`                | `false`                               | Extra `RCLCPP_INFO` / `WARN` on solve success or failure                                                                                                                        |
| `publish_debug_topics`             | `false`                               | Publish reference, odometry, output trajectory, solve status, and control debug topics under `~/debug/temporal_mpt/`                                                            |
| `write_replay_fixture`             | `false`                               | Write a text fixture for offline replay (e.g. Python generators)                                                                                                                |
| `replay_fixture_directory`         | `~/.ros/autoware_temporal_mpt_replay` | Output directory for replay fixtures                                                                                                                                            |
| `log_replay_fixture_to_console`    | `false`                               | Log fixture path and contents to the console                                                                                                                                    |

`wheel_base` is read from the vehicle description (`vehicle_info.param.yaml`, loaded with the stack via `common_param_path` in `trajectory_optimizer.launch.xml`), not from this plugin YAML.

## Debug visualization

With `publish_debug_topics: true`, run:

```bash
ros2 run autoware_trajectory_optimizer temporal_mpt_debug_visualizer.py
```

See `scripts/temporal_mpt_debug_visualizer.py` for topic names and options.

## Comparison with spatial MPT

| Aspect                     | Spatial `TrajectoryMPTOptimizer` | Temporal `TrajectoryTemporalMPTOptimizer` |
| -------------------------- | -------------------------------- | ----------------------------------------- |
| Solver                     | QP (`autoware_path_optimizer`)   | acados SQP                                |
| Reference parameterization | Spatial / corridor-based         | Time-ordered discrete points              |
| Bounds                     | Adaptive lateral corridor        | Lateral-acceleration + steering limits    |
| Velocity handling          | Recalculated post-optimization   | Optimized as a state                      |
| Typical pipeline position  | After multiple smoothers         | Replaces smoothers + spatial MPT          |

## Known limitations

- Only the first \(N+1\) trajectory points are overwritten with the MPC state trajectory; the rest of the message is unchanged.
- On solver failure (`status != 0`), the plugin returns without modifying `traj_points` (debug topics may still publish the last iterate if enabled).
- Reference yaw uses a \(2\pi\) bias so heading cost aligns with the initial state when planner yaw branches differ.
- Requires a successful acados build; the plugin creates the solver at initialization when loaded.
- Does not extend the trajectory backward; use `TrajectoryExtender` separately if that behavior is still needed.
