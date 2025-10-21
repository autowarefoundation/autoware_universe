# QP-Based Trajectory Smoother

## Overview

The QP (Quadratic Programming) smoother is a plugin in the trajectory optimizer that provides optimization-based trajectory smoothing using the OSQP solver. It smooths the geometric path (x,y positions) while maintaining path fidelity to the original trajectory from the planner.

## Mathematical Formulation

### Decision Variables

The optimization operates on 2D path positions:

```text
x = [x_0, y_0, x_1, y_1, ..., x_{N-1}, y_{N-1}]
```

where N is the number of trajectory points.

### Objective Function

The optimizer minimizes a weighted combination of two terms:

```text
minimize: J = J_smoothness + J_fidelity
```

#### Smoothness Term

Minimizes path curvature using second-order finite differences:

```text
J_smoothness = w_smoothness * Σ ||(p_{i+1} - 2*p_i + p_{i-1})||²
```

This term is scaled by `1/dt²` for velocity-aware smoothing.

#### Fidelity Term

Keeps the smoothed path close to the original trajectory:

```text
J_fidelity = Σ w_i * ||p_i - p_i^orig||²
```

where `w_i` is the per-point fidelity weight (see Velocity-Based Fidelity section).

### Constraints

- **First point**: Always fixed to original position (hard constraint)
- **Last point**: Optionally fixed (controlled by `constrain_last_point` parameter)

## Velocity-Based Fidelity Weighting

### Motivation

At low speeds, trajectory noise from the planner can cause jittery behavior. The velocity-based fidelity feature addresses this by:

- **Low speeds**: Lower fidelity weight → more aggressive smoothing.
- **High speeds**: Higher fidelity weight → preserve planner intent.

### Sigmoid Weight Function

Per-point fidelity weights are computed using a sigmoid function:

```text
w_i = w_min + (w_max - w_min) * σ(k * (|v_i| - v_th))

where σ(x) = 1 / (1 + exp(-x))
```

**Parameters:**

- `v_th`: Velocity threshold at sigmoid midpoint (default: 0.3 m/s)
- `k`: Sigmoid sharpness (default: 50.0, higher = sharper transition)
- `w_min`: Minimum weight at very low speeds (default: 0.01)
- `w_max`: Maximum weight at high speeds (default: 1.0)

**Key Properties:**

- Uses absolute velocity magnitude to handle reverse driving
- Smooth continuous transition (no discontinuities)
- Velocity threshold defines where weight is halfway between min/max
- Sharpness controls how rapidly weight changes near threshold

### Weight Behavior

```text
weight
1.0 |           _______  (high speed, preserve path)
    |          /
    |         /
0.01|_______/            (low speed, allow smoothing)
      0    v_th      v_max
           (0.3 m/s)
             speed
```

## Post-Processing

After QP optimization solves for smoothed positions, the following are recalculated:

1. **Orientation**: Computed from path tangent between consecutive points
2. **Velocity**: Derived from position differences and time step
3. **Velocity smoothing**: 3-point moving average to reduce numerical noise
4. **Acceleration**: Derived from velocity differences

## Parameters

### Core Optimization Weights

- `weight_smoothness` (default: 10.0)
  - Controls smoothness penalty
  - Higher values → smoother but more deviation from original path

- `weight_fidelity` (default: 1.0)
  - Baseline fidelity weight (used when velocity-based feature disabled)
  - Higher values → closer to original path

### Velocity-Based Fidelity

- `use_velocity_based_fidelity` (default: true)
  - Master switch for velocity-dependent weighting
  - Set to `true` to enable feature

- `velocity_threshold_mps` (default: 0.3)
  - Speed at which sigmoid transitions (midpoint)
  - Lower values → earlier transition to high fidelity

- `sigmoid_sharpness` (default: 50.0)
  - Controls transition steepness
  - Range: [1, 100], higher = sharper step-like transition

- `min_fidelity_weight` (default: 0.01)
  - Weight applied at very low speeds
  - Lower values → more aggressive smoothing when stopped/slow

- `max_fidelity_weight` (default: 1.0)
  - Weight applied at high speeds
  - Higher values → stronger preservation of planner path

### Endpoint Constraints

- `constrain_last_point` (default: false)
  - `true`: Both first and last points fixed as hard constraints
  - `false`: Only first point fixed, allows last point to move for additional smoothness

### Solver Settings

- `time_step_s` (default: 0.1)
  - Time discretization for velocity/acceleration calculations

- `osqp_eps_abs` / `osqp_eps_rel` (default: 1e-4)
  - OSQP solver convergence tolerances

- `osqp_max_iter` (default: 100)
  - Maximum solver iterations

- `osqp_verbose` (default: false)
  - Enable OSQP debug output

### Orientation Correction

- `fix_orientation` (default: true)
  - Apply orientation correction post-solve

- `orientation_correction_threshold_deg` (default: 5.0)
  - Yaw threshold for correcting QP output orientation

## Usage Examples

### Example 1: Enable Velocity-Based Fidelity

To enable velocity-dependent smoothing for stop-and-go scenarios:

```bash
ros2 param set /planning/trajectory_optimizer trajectory_qp_smoother.use_velocity_based_fidelity true
```

### Example 2: Tune Velocity Threshold

To make the transition occur at higher speeds (0.5 m/s instead of 0.2 m/s):

```bash
ros2 param set /planning/trajectory_optimizer trajectory_qp_smoother.velocity_threshold_mps 0.5
```

### Example 3: More Aggressive Low-Speed Smoothing

To allow even more smoothing at low speeds:

```bash
ros2 param set /planning/trajectory_optimizer trajectory_qp_smoother.min_fidelity_weight 0.005
```

### Example 4: Sharper Transition

To make the velocity-based transition more step-like:

```bash
ros2 param set /planning/trajectory_optimizer trajectory_qp_smoother.sigmoid_sharpness 80.0
```

### Example 5: Allow Last Point to Move

For maximum smoothness at trajectory endpoints:

```bash
ros2 param set /planning/trajectory_optimizer trajectory_qp_smoother.constrain_last_point false
```

## Debugging

### View Debug Logs

Enable debug logging to see weight statistics and solver diagnostics:

```bash
ros2 run autoware_trajectory_optimizer autoware_trajectory_optimizer_node --ros-args --log-level trajectory_optimizer:=debug
```

### Check Parameter Values

```bash
# Check if velocity-based fidelity is enabled
ros2 param get /planning/trajectory_optimizer trajectory_qp_smoother.use_velocity_based_fidelity

# Check weight configuration
ros2 param get /planning/trajectory_optimizer trajectory_qp_smoother.min_fidelity_weight
ros2 param get /planning/trajectory_optimizer trajectory_qp_smoother.max_fidelity_weight
```

### Monitor Performance

```bash
# View processing time metrics
ros2 topic echo /planning/trajectory_optimizer/debug/processing_time_detail_ms
```

## Limitations

1. **Path deviation**: Heavy smoothing can cause deviation from original path
2. **Velocity discontinuities**: Velocity is recalculated, may differ from planner intent
3. **Computational cost**: QP solve adds overhead compared to simpler smoothers
4. **Parameter sensitivity**: Requires tuning for specific vehicle dynamics

## References

- OSQP Solver: <https://osqp.org/>
- Autoware OSQP Interface: `autoware_osqp_interface` package
- Parent Package: `autoware_trajectory_optimizer`
