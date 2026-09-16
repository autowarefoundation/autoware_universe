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

| Index | Field                  | Unit / type | Description                                                  |
| ----- | ---------------------- | ----------- | ------------------------------------------------------------ |
| 0     | `reason_code`          | int         | Primary status (see table below)                             |
| 1     | `trigger_active`       | 0/1         | MRM trigger is true                                          |
| 2     | `is_latched`           | 0/1         | Trajectory latch is active                                   |
| 3     | `has_latest_candidate` | 0/1         | A candidate trajectory is stored in the latcher              |
| 4     | `data_ready`           | 0/1         | Map, route, odometry, and acceleration are available         |
| 5     | `plan_ok`              | 0/1         | Path planning succeeded this cycle (non-latched mode only)   |
| 6     | `validation_ok`        | 0/1         | Trajectory validator passed (non-latched mode only)          |
| 7     | `planned_points`       | count       | Trajectory point count after plan/smooth/modifier/velocity   |
| 8     | `published_points`     | count       | Point count of the trajectory actually published (0 if none) |
| 9     | `cycle_time_ms`        | ms          | Wall time for this timer callback                            |
| 10    | `odom_vx`              | m/s         | Longitudinal velocity from input odometry                    |

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

## Dependencies

- Path / obstacle-stop logic: vendored from `pilot-auto.x2.v4.3.e2e` (see `docs/*_sync.md`)
- EB smoothing: `autoware_path_smoother` (v4.3.2 workspace)
