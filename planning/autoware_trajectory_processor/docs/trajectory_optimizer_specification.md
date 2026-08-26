# Trajectory Optimizer - Specification and Assumptions

## Overview

This document describes the current specification, design assumptions, and known limitations of
`autoware_trajectory_optimizer`. It is written for engineers integrating upstream planners or
modifying the pipeline. Read this before touching plugin ordering or resampling logic.

---

## Core Design Principle: Positions Are Primary, Kinematics Are Derived

The optimizer works in the **position domain**. The path geometry (x, y per point) is what the
optimization models operate on. Velocity and acceleration fields in the output `TrajectoryPoint`
messages are **not preserved from the input** - they are recomputed as numerical derivatives of
the smoothed positions over a fixed time step.

This is not an approximation or a convenience. It is the fundamental contract of the pipeline:

- The upstream planner provides positions sampled at a **constant time interval** (dt).
- The optimizer adjusts those positions (smooths the path).
- Kinematics are recalculated from the new positions using the same dt.

If you treat the velocity or acceleration fields coming out of the planner as optimization inputs
to be preserved, you have misunderstood the system.

---

## Input Requirements

The optimizer expects trajectory points sampled at a **constant dt**, currently provided by the
diffusion planner upstream. This means:

- `time_from_start[i+1] - time_from_start[i]` must be constant and equal to the configured
  `trajectory_qp_smoother.time_step_s` (default: 0.1 s).
- Arc length between consecutive points is **not** constant - it varies with velocity.
- Points at low speed are close together in space. Points at high speed are far apart.

Do not arc-length resample the input trajectory before feeding it to the optimizer. Arc-length
resampling destroys the constant-dt structure and breaks the QP smoother assumptions. The QP
smoother never reads `time_from_start` from the input - it blindly assumes the configured
`time_step_s` applies uniformly to every consecutive point pair. If your input has variable dt,
the QP solver will silently compute wrong velocities and accelerations.

---

## How Velocity and Acceleration Are Computed

After the QP smoother adjusts point positions, the output kinematics are derived as follows:

**Velocity** (second pass, `post_process_trajectory`):

```text
v[i] = || p[i] - p[i-1] || / dt        (for i >= 1)
v[0] = input_trajectory[0].longitudinal_velocity_mps   (copied from input before smoothing)
```

A 3-point moving average is then applied starting at index 0, which overwrites v[0] with
`(v_input[0] + v_geom[1] + v_geom[2]) / 3`. v[0] is therefore not purely preserved - it is
blended with the geometric velocities of the next two points.

**Acceleration** (fourth pass, `recalculate_longitudinal_acceleration`):

```text
a[i] = (v[i+1] - v[i]) / dt
```

The last point gets `0.0` (explicitly zeroed).

**Consequences:**

- Any velocity profile set by the upstream planner (including stop points, speed limits,
  deceleration ramps) is **overwritten** for all points. Even v[0], which is initially copied
  from the input, is blended with the next two geometric velocities by the moving average.
- The velocity at each point reflects only how far the optimizer moved that point relative
  to the previous one over one dt interval.
- There is no mechanism inside the QP smoother to respect a maximum speed constraint per point.
  Speed limiting happens in a separate plugin (`TrajectoryVelocityOptimizer`) that runs later.

---

## Plugin Pipeline

Plugins are loaded in the order specified by `plugin_names`. Execution order is critical.

### Default Pipeline

The full `plugin_names` list (from `config/trajectory_optimizer.param.yaml`) and each plugin's
enabled state:

```text
TrajectoryPointFixer                      [ON]
TrajectoryKinematicFeasibilityEnforcer    [ON]   <- first pass, before QP
TrajectoryQPSmoother                      [ON]
TrajectoryKinematicFeasibilityEnforcer    [ON]   <- second pass, after QP
TrajectoryEBSmootherOptimizer             [OFF]
TrajectorySplineSmoother                  [ON]
TrajectoryMPTOptimizer                    [OFF]
TrajectoryVelocityOptimizer               [ON]
TrajectoryExtender                        [OFF]
```

Note that `TrajectoryKinematicFeasibilityEnforcer` is listed twice in `plugin_names` and runs
both before and after the QP smoother. Each pass is a separate plugin instance operating
independently.

### Plugin Descriptions

#### TrajectoryPointFixer (enabled)

Removes duplicate or numerically degenerate points (zero-distance, NaN positions). Optionally
resamples closely-spaced points. Also runs stop-approach detection and populates the shared
`SemanticSpeedTracker` for use by downstream plugins.

Does not modify velocities or time stamps.

**Stop-approach detection:** Points that are much closer together than normal represent a stop
zone. With a constant `dt`, `v = ds/dt` approaches zero as `ds` approaches zero, so geometric
proximity IS a reliable stop indicator. Detection runs in three stages:

1. `remove_close_proximity_points`: marks points that are geometric near-duplicates (`ds <
min_dist_to_remove_m`) as stop candidates.
2. `resample_close_proximity_points`: resamples clusters of closely-spaced points and marks the
   cluster endpoint as a stop candidate if the speed is decreasing.
3. `build_stop_approach_ranges`: processes all candidates, confirms speed is decreasing toward
   the candidate (not increasing — that would be a take-off), traces back the deceleration onset,
   and stores the resulting range in `SemanticSpeedTracker.slow_down_ranges`.

If no candidates produce valid stop approach ranges, a velocity-profile scan
(`detect_velocity_based_stop`) is run as a fallback for constant-spacing trajectories where no
geometric clusters exist.

#### TrajectoryKinematicFeasibilityEnforcer (enabled, runs twice)

Forward-propagating filter that clamps heading changes at each trajectory segment to respect
Ackermann steering geometry and a maximum yaw rate limit. At each step:

```text
delta_psi_max = min(kappa_max * s, psi_dot_max * avg_dt)
```

where `kappa_max = tan(delta_max) / L`, `s` is the segment arc length, and `avg_dt` is a single
average time step computed from all `time_from_start` deltas in the input trajectory (fallback:
0.1 s). The same `avg_dt` is used for every segment - per-segment dt is not recomputed from
distance or velocity.

Positions are adjusted to enforce this limit while preserving segment arc lengths. Velocities
and `time_from_start` are not modified.

Running it **before** the QP smoother pre-conditions the path so the QP operates on a
kinematically plausible input. Running it **after** the QP smoother catches any constraint
violations reintroduced by the smoothing. The second pass uses the same parameter set as the
first.

#### TrajectoryQPSmoother (enabled)

The primary path smoother. Solves a quadratic program that minimizes path curvature
(`||p[i+1] - 2*p[i] + p[i-1]||^2 / dt^2`) while penalizing deviation from the input path.

Decision variables: `[x_0, y_0, ..., x_{N-1}, y_{N-1}]` - positions only.

Hard constraints: the first `num_constrained_points_start` and last `num_constrained_points_end`
points are fixed to their input positions (defaults: start = 3, end = 0). Additionally, any
stop points detected by `TrajectoryPointFixer` (via `SemanticSpeedTracker`) are added as hard
equality constraints, pinning their positions regardless of smoothness weight.

After solving, velocities and accelerations are recomputed from the smoothed positions as
described above. For points within detected stop approach ranges, the planner's original velocity
profile is restored after the moving average step, preserving the intended deceleration curve.
The stop point itself is forced to zero velocity unconditionally. Orientations are recalculated
from the smoothed path geometry. Optionally, orientations can instead be copied from the original
input trajectory via nearest-neighbor matching (`preserve_input_trajectory_orientation`), but
this is **disabled by default**.

**The QP smoother must run before any plugin that resamples the trajectory.** Resampling
(changing the number of points or their spacing in arc length) breaks the constant-dt assumption
and invalidates the curvature penalty scaling.

#### TrajectorySplineSmoother (enabled)

Resamples the trajectory using Akima spline interpolation at a fixed arc-length resolution
(`interpolation_resolution_m`, default: 0.2 m).
`preserve_input_trajectory_orientation` is available but **disabled by default**; orientations
are computed from the spline geometry. After resampling, `time_from_start` is recomputed for all trajectory points; any `time_from_start` values supplied by upstream modules are therefore overwritten.

This plugin **breaks the constant-dt property**. After it runs, points are no longer at uniform
time intervals - they are at uniform arc-length intervals. This is intentional and necessary:
the downstream controller requires points that are not too close in space, regardless of velocity.
If points are too close (as they are at low speed with constant-dt sampling), the controller
produces poor curvature tracking.

The spline smoother must run **after** the QP smoother for this reason.

#### TrajectoryVelocityOptimizer (enabled)

Operates on the velocity profile of the trajectory after the path geometry has been fixed by
the smoothers. It does not modify positions.

Responsibilities:

- **Speed cap** (`limit_speed`, enabled by default): applies a global maximum speed from an
  external velocity limit topic or the configured default.
- **Lateral acceleration limiting** (`limit_lateral_acceleration`, disabled by default): for each consecutive point pair, it
  computes `yaw_rate = |delta_yaw / delta_time|` from orientation differences and
  `time_from_start`. If `v * yaw_rate > a_lat_max`, the point's speed is capped to
  `v_limit = a_lat_max / yaw_rate`.
- **Jerk-filtered velocity smoothing** (`smooth_velocities`, disabled by default): runs the
  in-package `ContinuousJerkSmoother` (QP-based, via `autoware_qp_interface`) that enforces
  `limit.max_acc`, `limit.min_acc`, `limit.max_jerk`, `limit.min_jerk` on the velocity profile.
- **Pull-out speed** (`set_engage_speed`, disabled by default): clamps initial velocity to
  `target_pull_out_speed_mps` when the vehicle is near standstill.

The velocity optimizer is the only stage where velocity values in the output trajectory are
computed with awareness of physical limits. Everything before it treats velocity as a side effect
of position geometry.

### Disabled Plugins

#### TrajectoryEBSmootherOptimizer (disabled)

Elastic Band path smoother wrapping `autoware_path_smoother`. Resamples the trajectory
internally. After smoothing, recomputes `time_from_start` from velocity and arc length.

This plugin is effectively deprecated in the current pipeline. It conflicts with the QP smoother
ordering constraint and its output quality relative to the QP smoother does not justify its
presence. It exists for historical reasons.

#### TrajectoryExtender (disabled)

Extends the trajectory backward by prepending past ego states stored in a rolling history buffer.
The intent is to provide the controller a trajectory that starts behind the vehicle's current
position, giving it a smooth reference even when the vehicle has advanced past the planner's
output head.

Known issue: the extender introduces a discontinuity at the junction between the history points
and the planner output, which causes artifacts in the QP smoother and degrades smoothing results.
For this reason it is disabled and its placement in the pipeline is near the end (after smoothers)
to minimize damage if enabled. This discontinuity is an open problem.

#### TrajectoryMPTOptimizer (disabled, experimental)

Model Predictive Trajectory optimizer wrapping `autoware_path_optimizer`'s `MPTOptimizer`.
Takes the trajectory and constructs simple perpendicular corridor bounds around it, then solves
a bicycle-model QP to refine the path within those bounds.

After optimization, recomputes kinematics using kinematic equations (`v^2 = v0^2 + 2as`) rather
than constant dt, propagating `time_from_start` forward through the trajectory.

This plugin was conceived as a complement to or replacement for the spline smoother, but it did
not produce results good enough to justify enabling it by default. The corridor bounds are
synthetic (perpendicular offsets from the reference path) rather than derived from map data,
which limits the optimizer's ability to meaningfully reshape the trajectory.

---

## Ordering Constraints Summary

| Rule                                      | Reason                                                   |
| ----------------------------------------- | -------------------------------------------------------- |
| Kinematic enforcer before QP              | Pre-conditions path so QP operates on feasible input     |
| QP smoother before spline/EB smoother     | Both resampling plugins destroy constant-dt; QP needs it |
| Kinematic enforcer after QP (second pass) | Catches constraint violations reintroduced by smoothing  |
| QP smoother before velocity optimizer     | Velocity optimizer works on the final path geometry      |
| Point fixer at start                      | Degenerate inputs break the QP solver                    |
| Extender after smoothers (if enabled)     | Discontinuity artifacts damage smoother output           |

---

## Why Arc-Length Resampling Is Avoided

The optimizer operates under a contract with the upstream planner: the planner outputs (x, y)
positions at a constant time interval. The optimizer's job is to refine those positions while
introducing the minimum changes necessary to produce a smooth, feasible path.

This minimal-change philosophy is deliberate. If the optimizer aggressively resamples and
reshapes the trajectory, it becomes impossible to attribute failures correctly. When something
goes wrong in the vehicle's behavior, you need to know whether the fault is on the model side
(the planner produced a bad trajectory) or on the optimizer side (the optimizer distorted a
good one). Heavy arc-length resampling muddles that boundary: the optimizer's output no longer
resembles the planner's intent, and debugging becomes guesswork.

By preserving the relative spacing of points - moving them laterally to smooth the path, but
not redistributing them along the arc - the optimizer keeps the planner's intent legible in its
output. Each output point corresponds directly to an input point. Deviations are local and
bounded by the fidelity weight.

Arc-length resampling also has a concrete technical incompatibility with the QP smoother. The
curvature penalty term:

```text
J_smooth = weight_smoothness / dt^2 * sum || p[i+1] - 2*p[i] + p[i-1] ||^2
```

is a second-order finite difference approximation of acceleration, valid only when consecutive
points are separated by a constant time interval `dt`. After arc-length resampling, points are
separated by a constant spatial interval `ds` instead. The QP smoother has no way to detect
this - it will still run and converge, but the weight scaling is wrong and the
smoothness-vs-fidelity tradeoff will be miscalibrated relative to the configured parameters.

The spline smoother at the end of the pipeline does arc-length resample. This is a known and
accepted exception: the downstream controller requires points that are not too close in space,
and at low speed with constant-dt sampling they are. The spline smoother runs after all
position-domain optimization is complete, so it does not corrupt any optimizer's assumptions.
Any plugin that arc-length resamples before the QP smoother has no place in this pipeline.

---

## Orientation Handling

Orientations are not optimized directly by any plugin. They are recomputed from path geometry
after position changes:

```text
yaw[i] = atan2(y[i+1] - y[i], x[i+1] - x[i])
```

Both the QP smoother and the spline smoother offer `preserve_input_trajectory_orientation`, which
copies orientations from the original input via nearest-neighbor matching instead of computing
them from the smoothed geometry. This option is **disabled by default in both plugins**, so
orientations are derived purely from the smoothed path tangent.

When enabled, the rationale is that the diffusion planner produces orientations that encode the
intended heading, which may differ from the pure geometric tangent of the smoothed path. If the
smoothing deviation is small relative to the configured distance threshold (QP smoother:
`max_distance_for_orientation_m`; spline smoother: `max_distance_discrepancy_m`; both default
to 5.0 m), the copied orientation is close enough to the geometric tangent that the error is
acceptable.

---

## Known Limitations

1. **Stop velocity preservation depends on detection.** Stop points are detected from geometric
   proximity (`ds < min_dist_to_remove_m`) and velocity profile. If a planner encodes a stop
   zone with point spacings above the detection threshold and no near-zero velocities below
   `stop_detection_velocity_threshold_mps`, the stop will not be detected and the QP smoother
   will overwrite the velocity profile as normal.

2. **Constant-dt is broken at the pipeline end.** The spline smoother as the last geometric step
   converts the trajectory to arc-length parameterization. Downstream consumers receive a
   trajectory that does not have constant-dt points. This is currently acceptable for the
   controller but means the optimizer output cannot be cleanly fed back into another constant-dt
   optimizer pass.

3. **The QP smoother ignores `time_from_start` in the input.** It uses its own configured
   `time_step_s` uniformly. If the input trajectory has a different dt (e.g., because a resampler
   ran earlier), the QP smoother will produce incorrect velocities without warning.

4. **Velocities outside detected stop zones are geometric.** For points not in a detected stop
   approach range, velocities are recomputed from position differences and blended through a
   3-point moving average. The planner's velocity values are not preserved there. For detected
   stop approach zones, the planner's original velocity profile is restored after the moving
   average and the stop point is forced to zero (see TrajectoryQPSmoother section).

5. **Trajectory extender discontinuity is unresolved.** The history-based backward extension
   creates a positional gap at the junction point that propagates through the smoothers. No fix
   is currently planned.

6. **MPT bounds are not map-aware.** The MPT optimizer's corridor bounds are simple perpendicular
   offsets, not derived from lanelet or drivable area data. The optimizer has no knowledge of
   actual road boundaries. Currently, there are no plans to introduce a map dependency into the optimizer node.
