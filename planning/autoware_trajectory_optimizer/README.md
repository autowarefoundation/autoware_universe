# Autoware Trajectory Optimizer

The `autoware_trajectory_optimizer` package generates smooth and feasible trajectories for autonomous vehicles using a plugin-based optimization pipeline. It takes candidate trajectories as input and applies a sequence of optimization plugins to produce smooth, drivable trajectories with proper velocity and acceleration profiles.

## Features

- **Plugin-based architecture** - Modular optimization pipeline where each step is a separate plugin
- **Multiple smoothing methods**:
  - Elastic Band (EB) smoother for path optimization
  - Akima spline interpolation for smooth path interpolation
  - QP-based smoother with quadratic programming for path smoothing with jerk constraints
- **Velocity optimization** - Jerk-filtered velocity smoothing from `autoware_velocity_smoother`
- **Trajectory validation** - Removes invalid points and fixes trajectory orientation
- **Backward trajectory extension** - Extends trajectory using past ego states
- **Dynamic parameter reconfiguration** - Runtime parameter updates supported

## Architecture

The package uses a plugin architecture where each optimization step inherits from `TrajectoryOptimizerPluginBase`. The current plugin execution order is:

1. **TrajectoryPointFixer** - Removes invalid/repeated points and fixes trajectory direction
2. **TrajectoryQPSmoother** - QP-based path smoothing with jerk constraints (optional)
3. **TrajectoryEBSmootherOptimizer** - Elastic Band path smoothing (optional)
4. **TrajectorySplineSmoother** - Akima spline interpolation (optional)
5. **TrajectoryVelocityOptimizer** - Velocity profile optimization
6. **TrajectoryExtender** - Extends trajectory backward using past ego states (optional, disabled by default)
7. **TrajectoryPointFixer** - Final cleanup of trajectory points

Each plugin can be enabled/disabled via parameters and manages its own configuration independently.

### ⚠️ Important: Plugin Ordering Constraints

**The order of plugin execution is critical and must be carefully maintained:**

- **QP Smoother must run before EB/Akima smoothers**: The QP solver relies on constant time intervals (Δt) between trajectory points (default: 0.1s). Both Elastic Band and Akima spline smoothers resample trajectories without preserving the time domain structure, which breaks the QP solver's assumptions. Therefore, when using multiple smoothers together, the QP smoother must execute first.

- **Trajectory Extender positioning**: The trajectory extender has known discontinuity issues when placed early in the pipeline. It negatively affects the QP solver results and introduces artifacts. For this reason, it has been moved to near the end of the pipeline and is **disabled by default** (`extend_trajectory_backward: false`). Fixing the extender's discontinuity issues is future work.

## QP Smoother

The QP smoother uses quadratic programming (OSQP solver) to optimize trajectory paths with advanced features:

- **Objective**: Minimizes path curvature while maintaining fidelity to the original trajectory
- **Decision variables**: Path positions (x, y) for each trajectory point
- **Constraints**: Fixed initial position (optionally fixed last position)
- **Velocity-based fidelity**: Automatically reduces fidelity weight at low speeds for aggressive smoothing of noise
- **Post-processing**: Recalculates velocities, accelerations, and orientations from smoothed positions

**For detailed documentation**, see [docs/qp_smoother.md](docs/qp_smoother.md) which covers:

- Mathematical formulation
- Velocity-based fidelity weighting (sigmoid function)
- Parameter tuning guidelines
- Usage examples
- Performance characteristics

## Dependencies

- `autoware_motion_utils` - Trajectory manipulation utilities
- `autoware_osqp_interface` - QP solver interface for QP smoother
- `autoware_path_smoother` - Elastic Band smoother
- `autoware_velocity_smoother` - Velocity smoothing algorithms
- `autoware_utils` - Common utilities (geometry, ROS helpers)
- `autoware_vehicle_info_utils` - Vehicle information

## Parameters

{{ json_to_markdown("planning/autoware_trajectory_optimizer/schema/trajectory_optimizer.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Main node parameters control plugin activation (e.g., `use_qp_smoother`, `use_akima_spline_interpolation`), while plugin-specific parameters use namespaced names (e.g., `trajectory_qp_smoother.weight_smoothness`).
