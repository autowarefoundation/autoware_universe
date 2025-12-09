# MPT (Model Predictive Trajectory) Optimizer

## Overview

The MPT optimizer is a plugin in the trajectory optimizer that refines trajectory geometry using quadratic programming-based optimization with adaptive corridor bounds. It leverages the `autoware_path_optimizer` package's MPT solver to optimize the trajectory path while ensuring the vehicle stays within dynamically-computed lateral bounds.

After geometric optimization, the plugin recalculates trajectory timing (time_from_start) and accelerations based on the optimized positions and velocities using kinematic equations, ensuring physical consistency.

## Architecture

### Two-Stage Process

1. **Geometric Optimization**: Uses MPT solver to optimize trajectory positions within adaptive corridor bounds
2. **Dynamics Recalculation**: Recomputes accelerations and time stamps from optimized geometry using kinematic relationships

## Corridor Bounds

### Purpose

The MPT optimizer requires lateral bounds to constrain the optimization. Normally these bounds come from the lanelet map (road boundaries), but to reduce dependency on map data, this plugin generates simple geometric bounds by offsetting perpendicular to the input trajectory.

This is a pragmatic workaround - the bounds are just perpendicular offsets from the trajectory centerline, nothing more.

### Bound Generation

For each trajectory point, left and right bounds are created by offsetting perpendicular to the heading:

```text
left_bound  = pose + corridor_width * [cos(yaw + π/2), sin(yaw + π/2)]
right_bound = pose + corridor_width * [cos(yaw - π/2), sin(yaw - π/2)]
```

### Width Calculation

When `enable_adaptive_width` is true, the corridor width is adjusted based on simple heuristics:

#### 1. Base Width

```text
corridor_width = base_corridor_width_m
```

#### 2. Curvature-Based Adjustment

Add extra width in curves:

```text
curvature_addition = curvature_width_factor * |κ|
```

where `κ` is the local path curvature (computed from three consecutive points).

#### 3. Velocity-Based Adjustment

Add extra width at low speeds:

```text
velocity_factor = max(0, (v_max - v) / v_max)
velocity_addition = velocity_width_factor * velocity_factor
```

where `v_max = 15.0 m/s`.

#### 4. Final Width

```text
corridor_width = max(
  base_width + curvature_addition + velocity_addition,
  vehicle_width + min_clearance
)
```

### Curvature Calculation

Local curvature is estimated using three consecutive points:

```text
θ_1 = atan2(y_i - y_{i-1}, x_i - x_{i-1})
θ_2 = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
Δθ = normalize(θ_2 - θ_1)
arc_length = ||p_i - p_{i-1}|| + ||p_{i+1} - p_i||
κ = Δθ / arc_length
```

## Acceleration and Timing Recalculation

After MPT optimization modifies trajectory positions, accelerations and time stamps must be recomputed to maintain physical consistency with the new geometry.

### Acceleration Calculation

Acceleration between consecutive points is computed using the kinematic equation relating velocity change to distance:

```text
v² = v₀² + 2as
```

Solving for acceleration:

```text
a = (v² - v₀²) / (2s)
```

where:

- `v₀` = velocity at current point
- `v` = velocity at next point
- `s` = distance between points
- `a` = average acceleration over segment

### Time Interval Calculation

Time interval between points uses kinematic motion equations. For constant acceleration motion:

```text
s = v₀t + ½at²
```

Rearranging using `v = v₀ + at`:

```text
v² = v₀² + 2as  →  v = √(v₀² + 2as)
t = (v - v₀) / a
```

**Special cases:**

- **Zero acceleration** (`|a| < 1e-6`): Uses constant velocity model `t = s / v`
- **Near-zero velocity** (`|v| < 1e-3 m/s`): Returns fixed `t = 0.1 s` to avoid division by zero
- **Invalid discriminant** (`v₀² + 2as < 0`): Falls back to constant velocity model

### Time from Start Propagation

Starting from the first point (set to `t = 0`), time is propagated forward:

```text
t_{i+1} = t_i + Δt_i
```

Time is split into ROS message format:

```cpp
sec = floor(t)
nanosec = (t - sec) * 1e9
```

### Acceleration Smoothing

Raw accelerations computed from kinematic equations may exhibit high-frequency noise due to discrete sampling. A backward-looking moving average filter is applied:

```text
a_smoothed[i] = (1/N) * Σ_{j=i-N+1}^{i} a_original[j]
```

where `N` is the window size (configurable via `acceleration_moving_average_window`).

**Properties:**

- **Causal filter**: Only uses past values (no future information)
- **Larger window**: Smoother acceleration but more lag
- **Window = 1**: No smoothing (preserves original values)
- **Last point**: Always set to zero acceleration

## Parameters

### Corridor Bounds Parameters

| Parameter                | Type   | Default | Range       | Description                                                       |
| ------------------------ | ------ | ------- | ----------- | ----------------------------------------------------------------- |
| `corridor_width_m`       | double | 3.5     | [2.0, 10.0] | Base perpendicular offset distance from trajectory centerline [m] |
| `enable_adaptive_width`  | bool   | true    | -           | Enable simple width adjustments based on curvature and velocity   |
| `curvature_width_factor` | double | 0.5     | [0.0, 2.0]  | Additional width per unit curvature [m/rad]                       |
| `velocity_width_factor`  | double | 0.3     | [0.0, 2.0]  | Additional width scaling at low speeds [m]                        |
| `min_clearance_m`        | double | 0.5     | [0.0, 2.0]  | Minimum clearance from vehicle edges [m]                          |

### MPT Behavior Parameters

| Parameter                            | Type | Default | Description                                                                                                                   |
| ------------------------------------ | ---- | ------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `reset_previous_data_each_iteration` | bool | true    | Reset MPT warm start data each cycle. Use true for planners that output new trajectories each cycle (e.g., diffusion planner) |
| `enable_debug_info`                  | bool | false   | Publish visualization markers for bounds and reference trajectory                                                             |

### Output Parameters

| Parameter                       | Type   | Default | Range       | Description                                |
| ------------------------------- | ------ | ------- | ----------- | ------------------------------------------ |
| `output_delta_arc_length_m`     | double | 1.0     | [0.1, 5.0]  | Point spacing in optimized trajectory [m]  |
| `output_backward_traj_length_m` | double | 5.0     | [0.0, 20.0] | Trajectory extension backward from ego [m] |

### Ego Matching Parameters

| Parameter                       | Type   | Default | Range        | Description                                        |
| ------------------------------- | ------ | ------- | ------------ | -------------------------------------------------- |
| `ego_nearest_dist_threshold_m`  | double | 3.0     | [0.5, 10.0]  | Distance threshold for ego-trajectory matching [m] |
| `ego_nearest_yaw_threshold_deg` | double | 45.0    | [10.0, 90.0] | Yaw threshold for ego-trajectory matching [deg]    |

### Acceleration Smoothing Parameters

| Parameter                            | Type | Default | Range   | Description                                                       |
| ------------------------------------ | ---- | ------- | ------- | ----------------------------------------------------------------- |
| `acceleration_moving_average_window` | int  | 5       | [1, 20] | Moving average window size. Larger values = smoother but more lag |

## Comparison with Other Plugins

### MPT vs QP Smoother

**QP Smoother** - Geometric smoothing in Cartesian space:

- Minimizes path curvature via second-order finite differences
- Balances smoothness vs fidelity to original path
- No vehicle kinematic model
- Fast (~1-2ms), no bounds required

**MPT Optimizer** - Model predictive with bicycle kinematics:

- Optimizes in Frenet frame (lateral error, yaw error, steering)
- Uses bicycle kinematics model with steering dynamics
- Minimizes steering angle, rate, and acceleration explicitly
- Slower (~5-20ms), requires corridor bounds

**Use QP when:** Simple geometric smoothing needed, tight computational budget.
**Use MPT when:** Kinematically-aware refinement needed, working with learning-based planners.

### MPT vs Kinematic Feasibility Enforcer

**Kinematic Feasibility Enforcer** - Hard constraint enforcement:

- Forward propagation with yaw rate clamping
- Hard constraints (must satisfy, no violations)
- Greedy point-by-point approach
- Fast, deterministic

**MPT Optimizer** - Optimization with soft constraints:

- QP optimization over prediction horizon
- Soft constraints with slack variables (can violate if beneficial)
- Globally optimizes steering smoothness
- Finds best trajectory balancing multiple objectives

**Use Enforcer when:** Need minimum feasibility guarantee, computational budget critical.
**Use MPT when:** Need smooth, optimized trajectories considering full vehicle dynamics.

## Interaction with Other Plugins

### Recommended Plugin Order

The MPT optimizer should run **after all geometric smoothing** but **before velocity optimization**:

```yaml
plugin_names:
  - TrajectoryPointFixer
  - TrajectoryKinematicFeasibilityEnforcer
  - TrajectoryQPSmoother
  - TrajectoryKinematicFeasibilityEnforcer # Second pass after QP smoothing
  - TrajectoryEBSmootherOptimizer
  - TrajectorySplineSmoother
  - TrajectoryMPTOptimizer # Refine geometry with bounds
  - TrajectoryVelocityOptimizer # Optimize velocity profile
  - TrajectoryExtender
```

### Compatibility Notes

- **Requires non-empty bounds**: MPT optimization fails if bounds are empty
- **Modifies positions**: Subsequent plugins must handle changed geometry
- **Preserves velocities**: Velocity values from input trajectory are maintained
- **Recalculates accelerations**: Overwrites acceleration values from previous plugins
- **Recalculates time stamps**: Overwrites `time_from_start` based on kinematic consistency

## Collision Avoidance Capability

**Note:** The underlying MPT solver from `autoware_path_optimizer` supports collision avoidance optimization when provided with drivable area bounds (where obstacles are removed from the drivable area, creating bounds that inherently avoid collisions).

However, this plugin implementation does **not utilize collision avoidance** because:

- Bounds are generated as simple perpendicular offsets from the trajectory centerline
- No obstacle or predicted object data is integrated
- The drivable area is not used

The collision-related parameters in the configuration (`soft_collision_free_weight`, `collision_free_constraints`, `avoidance` weights, vehicle circle approximations) are part of the underlying MPT solver's capabilities but remain unused in this geometric-only implementation.

**Future work:** Collision avoidance could be added by integrating obstacle data and generating bounds from actual drivable area instead of geometric offsets.

## Limitations

### 1. Bound Requirement

The MPT optimizer **requires valid corridor bounds** to operate. Empty bounds cause optimization to fail and the original trajectory is preserved.

### 2. Velocity Profile Assumptions

- Assumes velocity profile from input trajectory is appropriate
- Does not modify velocities during geometric optimization
- Acceleration recalculation assumes constant acceleration between points

### 3. Computational Cost

MPT optimization using quadratic programming is more expensive than local smoothing methods:

- Typical runtime: 5-20 ms for 80-100 point trajectory
- Should be disabled (`use_mpt_optimizer: false`) if computational budget is tight

### 4. Acceleration Smoothing Lag

Moving average filter introduces lag proportional to window size:

- Larger window → smoother acceleration but delayed response
- May affect real-time control performance if window is too large

### 5. Geometric vs Dynamic Optimization

The MPT solver optimizes geometric criteria (smoothness, bounds compliance) but does not directly optimize for:

- Jerk minimization
- Passenger comfort metrics
- Longitudinal acceleration limits

These should be handled by dedicated plugins (e.g., `TrajectoryVelocityOptimizer`).

### 6. Reverse driving

Assumes forward motion. May need special handling for reverse trajectories.

## When to Use MPT Optimizer

### Recommended Use Cases

- Trajectories from learning-based planners requiring geometric refinement
- When you want to avoid dependency on lanelet map bounds
- Situations where QP smoother alone produces insufficient smoothing

### When to Disable

- Computational resources are limited (MPT is expensive)
- Input trajectories are already smooth and kinematically feasible
- Real-time performance is critical (< 10 ms budget)

## Debug Visualization

When `enable_debug_info: true`, the plugin publishes visualization markers to `~/debug/mpt_bounds_markers`:

- **Green lines**: Left bound (perpendicular offset)
- **Red lines**: Right bound (perpendicular offset)
- **Blue line**: Reference trajectory

These markers help visualize the generated bounds and tune width parameters.
