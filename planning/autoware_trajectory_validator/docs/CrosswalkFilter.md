# Crosswalk Filter

## Purpose/Role

<!-- cspell: ignore unsignaled -->

This filter rejects trajectories that would proceed to cross an unsignaled crosswalk without waiting long enough for nearby VRUs (vulnerable road users) that appear to be waiting to cross.

It focuses on pedestrians/bicycles **waiting** at the sidewalk near the crosswalk edge. Objects that are already inside the crosswalk area are ignored. Signaled (traffic-light-controlled) crosswalks are excluded; those cases are expected to be handled by traffic-light logic elsewhere.

A trajectory is rejected when:

1. It reaches / crosses the crosswalk stop line, AND
2. There is at least one non-ignored target object associated with that crosswalk, AND
3. The planned stop duration given by the trajectory is shorter than the remaining required wait time.

## Algorithm Overview

The following diagram shows the overall logic flow of the `CrosswalkFilter`:

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE
start
if (Is Invalid Input?) then (yes)
:return unexpected;<<#LightYellow>>
stop
else (no)
endif
:Collect unsignaled crosswalks on route;<<#LightBlue>>
:Generate trajectory linestring;<<#LightBlue>>
:Find crosswalks whose stop lines intersect the trajectory;<<#LightBlue>>
if (No target crosswalks?) then (yes)
:Set to Feasible;<<#LightGreen>>
:Return Validation Result;<<#LightBlue>>
stop
else (no)
endif
:Generate sidewalk detection areas;<<#LightBlue>>
:Update target objects in detection areas;<<#LightBlue>>
:Check obstruction (wait duration vs planned start-move time);<<#LightBlue>>
:Set Debug Data & Metrics Report;<<#LightBlue>>
if (Is Obstructing?) then (yes)
:Set to NOT Feasible;<<#LightPink>>
else (no)
:Set to Feasible;<<#LightGreen>>
endif
:Return Validation Result;<<#LightBlue>>
stop
@enduml
```

### Target crosswalk selection

1. Collect `Crosswalk` regulatory elements attached to preferred route lanelets via `lanelet::utils::query::crosswalks`.
2. Skip crosswalks whose lanelet has a traffic-light regulatory element (signaled crosswalks).
3. Build a 2D linestring from the candidate trajectory (skipping points behind ego), truncated at the first near-zero velocity point or at the computed lookahead distance:
   - Lookahead ≈ ego stop distance + `arrived_distance_threshold`
4. Extend the linestring by the vehicle front offset (and, if needed, pad up to `arrived_distance_threshold`) so nearby stop lines can still be found when the trajectory is short.
5. Keep crosswalks whose stop lines intersect that linestring, recording arc length to the stop line.
6. Mark `is_crossing = true` when the trajectory footprint length reaches the stop line.

### Detection areas (waiting end caps)

For each target crosswalk, sidewalk-side detection areas are built from the crosswalk lanelet bounds:

- **Longitudinal**: extrude each end of the left/right bounds outward along the crosswalk direction by `lon_detection_margin`.
- **Lateral**: widen each end cap along the end edge (left ↔ right) by `lat_detection_margin`.

The crosswalk polygon itself is **not** included, so objects already on the crosswalk are not treated as waiting targets.

### Target object tracking

For each target crosswalk:

1. Filter predicted objects by configured `object_types` (default: pedestrian, bicycle).
2. Keep objects whose position lies inside the detection areas.
3. Associate objects across frames by UUID, or by type + proximity within `distance_hysteresis_th`.
4. Maintain per-object timers:
   - `first_seen_time` / `last_seen_time`
   - While ego is **not** stopped near the stop line, `first_seen_time` is refreshed every cycle (wait timer does not accumulate).
   - While ego **is** stopped near the stop line, `first_seen_time` is held so the wait duration grows.
   - When wait duration exceeds `stop_duration`, the object is marked `ignore` (wait considered complete).
5. Clear objects not seen for longer than `object_clear_time_th`.
6. Drop object history for crosswalks that are no longer in the target set.

### Obstruction check

For each target crosswalk with `is_crossing == true`:

1. Compute the remaining required wait:
   - `required_waiting_time = stop_duration - min(non-ignored object wait durations)`
2. If `required_waiting_time ≈ 0`, the crosswalk is not obstructing.
3. Otherwise, find the first trajectory point with longitudinal velocity above a small threshold and take its `time_from_start` as the planned start-move time.
4. Reject the trajectory if `start_move_time < required_waiting_time`.

Fully stopped trajectories (no point with velocity above the threshold) are not rejected by this check.

## Interface

### Context

The filter utilizes the following data from the `FilterContext`:

- **Lanelet Map**: Used to query crosswalk regulatory elements, stop lines, and crosswalk lanelet geometry.
- **Route**: Used to select crosswalks attached to preferred route lanelets.
- **Predicted Objects**: Used to detect waiting VRUs in the sidewalk detection areas.
- **Odometry**: Provides ego pose/velocity for lookahead, stop-at-crosswalk detection, and timestamps.
- **Acceleration**: Used with velocity to estimate the ego stop / lookahead distance.
- **Vehicle Info**: Used for front overhang when measuring distance to the stop line and extending the trajectory footprint.

### Parameters

| Parameter name                         | Type         | Default                 | Description                                                                                            |
| -------------------------------------- | ------------ | ----------------------- | ------------------------------------------------------------------------------------------------------ |
| `crosswalk.object_types`               | string array | `[pedestrian, bicycle]` | Object classification labels treated as target VRUs.                                                   |
| `crosswalk.lon_detection_margin`       | double       | 2.0                     | [m] Longitudinal extrusion of sidewalk detection end caps beyond the crosswalk.                        |
| `crosswalk.lat_detection_margin`       | double       | 1.0                     | [m] Lateral widening of each detection end cap along the crosswalk end edge.                           |
| `crosswalk.object_clear_time_th`       | double       | 0.3                     | [s] Time after which an unseen target object is removed from tracking.                                 |
| `crosswalk.distance_hysteresis_th`     | double       | 0.3                     | [m] Position hysteresis used to match the same object across frames when UUID matching is not enough.  |
| `crosswalk.overshoot_tolerance`        | double       | 0.1                     | [m] Tolerance subtracted from the vehicle front overhang when extending the checked trajectory.        |
| `crosswalk.nominal_deceleration`       | double       | 1.0                     | [m/s²] Nominal deceleration used for stop-distance / lookahead calculation.                            |
| `crosswalk.nominal_jerk`               | double       | 1.0                     | [m/s³] Nominal jerk used for stop-distance / lookahead calculation.                                    |
| `crosswalk.stop_duration`              | double       | 3.0                     | [s] Minimum wait duration required at the stop line when target objects are present.                   |
| `crosswalk.arrived_distance_threshold` | double       | 5.0                     | [m] Distance threshold (front bumper to stop line) used to decide if ego has arrived at the crosswalk. |

## Logging and Visualization

The `CrosswalkFilter` publishes debug markers under `~/debug/markers/crosswalk_filter`.

### Debug Markers

| Namespace                 | Type               | Color     | Description                                                                  |
| ------------------------- | ------------------ | --------- | ---------------------------------------------------------------------------- |
| `target_crosswalks`       | `LINE_STRIP`       | Magenta   | Crosswalk lanelet polygon for each target crosswalk.                         |
| `detection_areas`         | `LINE_LIST`        | Yellow    | Sidewalk-side waiting end caps used for object association.                  |
| `target_stop_lines`       | `LINE_STRIP`       | Magenta   | Stop line associated with each target crosswalk.                             |
| `target_objects`          | `LINE_STRIP`       | Red/Green | Tracked target object footprints (`red` = still waiting, `green` = ignored). |
| `target_objects_duration` | `TEXT_VIEW_FACING` | White     | Current wait duration [s] for non-ignored target objects.                    |
