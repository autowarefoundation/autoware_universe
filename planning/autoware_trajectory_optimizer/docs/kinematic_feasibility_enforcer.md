# Ackermann Kinematic Feasibility Enforcer

## Overview

The Ackermann Kinematic Feasibility Enforcer is a plugin in the trajectory
optimizer that ensures planned trajectories respect vehicle kinematic
constraints. It enforces steering limits and yaw rate constraints based on
the Ackermann steering model, preventing the planner from generating
trajectories with infeasible sharp turns or rapid heading changes.

This plugin operates on trajectory positions and orientations while
preserving velocities and time stamps to maintain compatibility with
downstream optimization plugins (e.g., QP smoother).

## Mathematical Formulation

### Vehicle Model

The plugin uses an Ackermann bicycle model with the following parameters:

- **L**: Wheelbase (distance between front and rear axles) [m]
- **δ_max**: Maximum steering angle [rad]
- **ψ_dot_max**: Maximum yaw rate [rad/s]

### Kinematic Constraints

#### 1. Geometric Constraint (Ackermann Model)

The maximum path curvature is limited by the maximum steering angle:

```text
κ_max = tan(δ_max) / L
```

Over a distance `s`, the maximum achievable yaw change is:

```text
Δψ_geom = κ_max * s = (tan(δ_max) / L) * s
```

#### 2. Rate Constraint

The yaw rate limit restricts heading changes over time:

```text
Δψ_rate = ψ_dot_max * Δt
```

where `Δt` is the time interval between trajectory points.

#### 3. Combined Constraint

At each trajectory segment, the feasible yaw change is:

```text
Δψ_max = min(Δψ_geom, Δψ_rate)
```

The actual yaw change is clamped to this limit:

```text
Δψ_actual = clamp(Δψ_desired, -Δψ_max, Δψ_max)
```

## Algorithm

### Forward Propagation Approach

The plugin processes the trajectory using forward propagation from the ego
vehicle pose:

1. **Initialize anchor**: Use current ego pose as initial anchor point
2. **For each trajectory segment**:
   - Compute desired heading from current position toward original next point
   - Calculate desired yaw change: `Δψ_desired = normalize(ψ_desired - ψ_current)`
   - Compute feasibility limits from both constraints
   - Clamp yaw change: `Δψ_clamped = clamp(Δψ_desired, -Δψ_max, Δψ_max)`
   - Update heading: `ψ_new = ψ_current + Δψ_clamped`
   - Recompute next point position preserving segment distance
   - Update anchor to current point
3. **Preserve arc lengths**: Original segment distances are maintained to
   preserve trajectory timing structure

### Key Algorithm Properties

- **Arc length preservation**: Maintains `dt = s / v` relationship for each
  segment
- **Forward causality**: Each point depends only on previous points (no
  backward propagation)
- **Velocity preservation**: Original velocity profile unchanged
- **Time stamp preservation**: Trajectory timing structure maintained for QP
  smoother

## Implementation Details

### Segment Distance Pre-computation

Before modifying any positions, all segment distances from the original
trajectory are pre-computed and stored:

```cpp
for (size_t i = 0; i < traj_points.size() - 1; ++i) {
  const auto dist = calc_distance2d(traj_points[i], traj_points[i + 1]);
  segment_distances.push_back(max(dist, min_segment_distance));
}
```

This ensures arc lengths remain constant during forward propagation, preserving
the implicit timing: `dt_i = s_i / v_i`.

### Heading Update Logic

At each iteration:

1. **Extract current state**: Position and heading from anchor point
2. **Compute desired heading**: From current position toward original next
   point
3. **Normalize angle difference**: Handle ±π wrapping correctly
4. **Apply constraints**: Both geometric and rate limits
5. **Update position**: Place next point at fixed distance `s` along new
   heading
6. **Update orientation**: Set quaternion from clamped yaw angle

### Numerical Stability

- **Minimum segment distance**: Enforced to be ≥ 1e-6 m for numerical stability
- **Angle normalization**: Uses `autoware_utils_math::normalize_radian()` to
  handle wraparound
- **Clamping**: Uses `std::clamp()` for symmetric limit enforcement

## Parameters

### Vehicle Parameters (from Vehicle Info)

These are automatically loaded from vehicle configuration:

- `wheel_base_m`: Wheelbase distance [m]
- `max_steer_angle_rad`: Maximum steering angle [rad]

### Plugin Parameters

- `trajectory_kinematic_feasibility.max_yaw_rate_rad_s` (default: 0.7 rad/s)
  - Maximum allowable yaw rate
  - Default: 0.7 rad/s (~40 deg/s) aligns with MPC controller
  - Typical values: 0.5-1.0 rad/s (≈ 29-57 deg/s) for autonomous vehicles
  - Higher values → less restrictive, more aggressive turns
  - Lower values → smoother, more conservative trajectories
  - Should be tuned based on vehicle dynamics and comfort requirements

## Interaction with Other Plugins

### Plugin Execution Order

The kinematic feasibility enforcer should run **before** the QP smoother in
the plugin pipeline to ensure:

1. Trajectory points respect kinematic constraints
2. QP smoother operates on kinematically feasible input
3. Arc lengths and timing structure are preserved for QP optimization

## Limitations

1. **Path deviation**: Kinematic constraints may cause significant deviation
   from the original planner path, especially for aggressive maneuvers.

2. **Velocity-time assumptions**: Algorithm assumes constant velocity within
   each segment (`dt = s / v`). Large velocity changes may affect accuracy.

3. **Steering dynamics**: Does not model steering rate limits or steering
   system dynamics - only considers geometric and yaw rate constraints.

4. **Lateral acceleration**: Does not directly constrain lateral acceleration
   (though indirectly limited by yaw rate constraint).

5. **Reverse driving**: Assumes forward motion. May need special handling for
   reverse trajectories.

6. **Computation time**: Forward propagation through entire trajectory adds
   overhead (typically < 1 ms for 100-point trajectory).
