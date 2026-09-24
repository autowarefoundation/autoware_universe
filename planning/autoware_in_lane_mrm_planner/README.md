# autoware_in_lane_mrm_planner

MRM in-lane stop trajectory planner (Phase1).

## Design

See `docs/design_phase1.md` and `docs/implementation_plan_phase1.md`.

## Build

From `pilot-auto.x2.v4.3.2` repository root (after workspace dependencies are installed):

```bash
./cmd_helper.sh --build_ccache --packages-select autoware_in_lane_mrm_planner
```

Do not run package builds in parallel with a full workspace build.

## Launch

```bash
ros2 launch autoware_in_lane_mrm_planner in_lane_mrm_planner.launch.xml
```

## Debug topic: planner status

Published every control cycle to explain why `~/output/trajectory` was or was not published.

| Item                           | Value                                                       |
| ------------------------------ | ----------------------------------------------------------- |
| Topic (node)                   | `~/debug/planner_status`                                    |
| Default remap                  | `/planning/in_lane_mrm_planner/debug/planner_status`        |
| Domain 1 (vehicle, via bridge) | `/mrm/planning/in_lane_mrm_planner/debug/planner_status`    |
| Type                           | `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped` |

Use `data[0]` as the primary reason code when plotting in PlotJuggler or post-processing rosbags.
Other fields are boolean flags (0/1) or numeric diagnostics.

### `data[]` layout

| Index | Field                  | Unit / type | Description                                                                                                                 |
| ----- | ---------------------- | ----------- | --------------------------------------------------------------------------------------------------------------------------- |
| 0     | `reason_code`          | int         | Primary status (see table below)                                                                                            |
| 1     | `trigger_active`       | 0/1         | MRM trigger is true                                                                                                         |
| 2     | `is_latched`           | 0/1         | Trajectory latch is active                                                                                                  |
| 3     | `has_latest_candidate` | 0/1         | A candidate trajectory is stored in the latcher                                                                             |
| 4     | `data_ready`           | 0/1         | Map, route, odometry, and acceleration are available                                                                        |
| 5     | `plan_ok`              | 0/1         | Path planning succeeded this cycle (non-latched mode only)                                                                  |
| 6     | `validation_ok`        | 0/1         | Trajectory validator passed (non-latched mode only)                                                                         |
| 7     | `planned_points`       | count       | Trajectory point count after plan/smooth/modifier/velocity                                                                  |
| 8     | `published_points`     | count       | Point count of the trajectory actually published (0 if none)                                                                |
| 9     | `cycle_time_ms`        | ms          | Wall time for this timer callback                                                                                           |
| 10    | `odom_vx`              | m/s         | Longitudinal velocity from input odometry                                                                                   |
| 11    | `sanitized_points`     | count       | Overlapping points removed before publish (should stay 0; nonzero means an upstream stage produced (near-)duplicate points) |

### `reason_code` values

| Code | Name                                 | Meaning                                                              |
| ---- | ------------------------------------ | -------------------------------------------------------------------- |
| 0    | `published_ok`                       | Non-latched mode: planned, validated, and published                  |
| 1    | `waiting_map`                        | Missing `~/input/vector_map`                                         |
| 2    | `waiting_route`                      | Missing `~/input/route`                                              |
| 3    | `waiting_odometry`                   | Missing `~/input/kinematic_state`                                    |
| 4    | `waiting_accel`                      | Missing `~/input/acceleration`                                       |
| 10   | `plan_failed_update_current_lanelet` | Ego pose not on route lanelets                                       |
| 11   | `plan_failed_backward_lanelets`      | Failed to extend lanelets backward on route                          |
| 12   | `plan_failed_forward_lanelets`       | Failed to extend lanelets forward on route                           |
| 13   | `plan_failed_invalid_s_range`        | Invalid path range (`s_end <= s_start`)                              |
| 14   | `plan_failed_generate_path`          | Path generation failed (empty lanelet, no points, crop failed, etc.) |
| 15   | `plan_failed_no_output`              | Plan/validation succeeded but no trajectory to publish               |
| 20   | `validation_failed_point_count`      | Validator: fewer than `min_point_count` points (also covers empty)   |
| 21   | `validation_failed_non_finite`       | Validator: non-finite pose/velocity/acceleration                     |
| 30   | `latched_output_published`           | Latched mode: publishing frozen trajectory                           |
| 31   | `latched_without_candidate`          | Latched mode: no candidate stored (nothing published)                |
| 99   | `unknown`                            | Fallback / unclassified                                              |

Codes are defined in `src/in_lane_mrm_planner_node.cpp` (`StatusReasonCode`).

The validator is a publish-gate sanity check only (point count and finite values).
Trajectory shape and longitudinal feasibility are owned by obstacle-stop and the
velocity planner, not the validator.

### Correlating with follower target speed drops

If `reason_code` is not `0` or `30`, or `published_points` is 0 while the vehicle is moving,
`~/output/trajectory` was not updated that cycle. The longitudinal follower then keeps the
previous reference, which can make target speed appear to drop to zero in diagnostics.

## Future tasks

Deferred follow-ups from the 2026-06-30 incident investigation (MRM trajectory follower
abort crash caused by duplicate points in the published trajectory; rosbags
`..._2026-06-30-11-36-50_p0900_7.db3` and `..._2026-06-30-15-12-51_p0900_8.db3`, both while
driving manually near the same U-turn lanelet junction around map coordinates
(x=89149, y=42425)). The duplicate-point generation itself was fixed
(`densify_near_arc_length()` sample spacing guard + `remove_overlap_points()` publish net),
but the upstream degeneracies below remain:

1. **`shift_trajectory_to_ego()` short-trajectory fallback** (`src/path_planner.cpp`,
   `merge_idx = size - 2` branch): when the remaining trajectory ahead of ego is shorter than
   the shift length `L` (ego overrunning the path end), the fallback degenerates down to a
   3-point trajectory `[ego, end-1, end]` whose tail can lie behind ego. Agreed direction:
   when no intermediate shift point can be generated, skip shifting and return the input
   trajectory unchanged (the input already passes through ego laterally via
   `apply_lateral_offset()`).
2. **`apply_lateral_offset()` self-intersection at sharp centerline kinks**
   (`src/in_lane_mrm_trajectory_planner.cpp`): offsetting every point along its own normal
   self-intersects where the lateral offset exceeds the local curvature radius (observed at a
   lanelet junction with ~107 deg heading discontinuity within 0.2 m). Additionally, near the
   kink the ego projection flips between branches, so the applied offset `d` jumps frame to
   frame (up to 1.7 m observed). Agreed direction: detect the kink during path generation and
   truncate the path before it (which then relies on item 1 / item 3 for the degraded output).
3. **Explicit diagnostics / "when does the MRM planner give up" design**: needs a top-down
   decision before implementation. Candidate signals identified in the investigation: ego far
   off the reference path (large lateral offset or yaw deviation; ~2 rad observed before the
   crash), plan failure (`invalid_s_range` after ego passes the path end), and validation
   failure. Today these only reach the debug `planner_status` topic; whether to promote them
   to `/diagnostics` (and with which severities/thresholds) is undecided. Related question:
   explicit diag vs. relying on trajectory-timeout diag downstream.
4. **Drive-side delay modeling in the `brake_delay_time` hold**
   (`src/mrm_stop_velocity_planner.cpp`, `effective_initial_accel()`): the hold model assumes
   the drive command cuts immediately at the MRM trigger, so a positive current acceleration
   is clamped to zero during the hold (valid for BEVs such as J6). For vehicles with slow
   drive-torque decay (e.g. ICE with a torque converter), consider a separate
   `drive_delay_time` or a parameter to disable the clamp. Until then the residual effect can
   be absorbed by tuning `brake_delay_time` upward (agreed 2026-07-09).
5. **Upper-bound validation for `brake_delay_time`**
   (`param/in_lane_mrm_planner_parameters.yaml`): the parameter has no upper bound, so a
   misconfigured value (e.g. 50.0) makes the required stop distance exceed the 500 m cap in
   `calc_required_stop_distance()`, every plan becomes infeasible, and the planner permanently
   publishes the zero-velocity fallback — functionally safe but silently degraded, visible
   only in a throttled error log. Add an `lt_eq<>` bound (a few seconds) mirrored as
   `maximum` in the JSON schema (review finding, 2026-07-09).
6. **Regression test that `brake_delay_time = 0.0` does not clamp a positive `a0`**
   (`test/test_mrm_stop_velocity_planner.cpp`): the clamp in `effective_initial_accel()` must
   apply only when the delay is positive, but no test pins the delay-zero half; a regression
   that clamps unconditionally would pass the current suite (review finding, 2026-07-09).

## Dependencies

- Path / obstacle-stop logic: vendored from `pilot-auto.x2.v4.3.e2e` (see `docs/*_sync.md`)
- EB smoothing: `autoware_path_smoother` (v4.3.2 workspace)
