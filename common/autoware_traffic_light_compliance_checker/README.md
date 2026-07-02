# Traffic Light Compliance Checker

The `traffic_light_compliance_checker` package provides a deterministic validation layer that cross-references planned vehicle trajectories against real-time perceived traffic light signals and High-Definition (HD) vector maps to enforce strict traffic compliance.

## Core Features

1. **Signal State Tracking (`TrafficLightStatusTracker`)**
   - Eliminates perception jitter and signal flickering by maintaining a temporal state history for each traffic light group ID.
   - Leverages stable duration thresholds before validating a transition to `RED` or `AMBER`.
   - Utilizes a hysteresis buffer to sustain known states during transient object occlusions.

2. **Trajectory Validation (`TrafficLightComplianceChecker`)**
   - Scans forward trajectory segments sequentially to isolate intersection entry points.
   - Appends a physical front-bumper projection to ensure the vehicle footprint stays behind regulatory stop lines.
   - Implements a kinematic pass/stop feasibility matrix for `AMBER` signals based on comfortable braking and intersection clearance times.
   - Returns prioritized, chronological arrays of `Violation` metadata if an unvalidated stop line overshoot is detected.

## Inner Workings

### Main Processing Pipeline

The execution flow follows a sequential logical from signal processing & filtering down to stop line interaction evaluations:

1. **Filter signals and update status tracker:** Feeds raw perception data into the status tracker to clean up transient noise and tracking dropouts.
2. **Generate Geometric Trajectory Linestring:** Removes path points situated behind the ego vehicle, clamps the forward path length at the maximum required stop distance, and extends trajectory end to account for ego front offset.
3. **Extract and group map stop lines:** Sorts overlapping intersection lines into separate evaluation queues for red and amber constraints.
4. **Evaluate Stop Line Violations:** Checks the trajectory linestring against active stop lines, records detected violations and generate compliance result.

```plantuml
@startuml
skinparam defaultTextAlignment center
skinparam backgroundColor #WHITE

start

:Filter signals and update status tracker;<<#LightBlue>>
:Generate Trajectory Linestring\n(Cull backward points & clamp at max stop distance);<<#LightBlue>>
:Extend trajectory linestring\n(Add physical front bumper footprint offset);<<#LightBlue>>
:Extract and group map stop lines\n(Categorize into RED vs. AMBER targets);<<#LightBlue>>

if (Is check_red_lights enabled?) then (yes)
  group Get red stop line violations #Lavender {
    if (Trajectory Intersects stop line?) then (yes)
      if (Trajectory end exceeds tolerance threshold?) then (yes)
        :Record RED Light Violation; <<#LightPink>>
      else (no)
      endif
    else (no)
    endif
  }
else (no)
endif

if (Is check_amber_lights enabled?) then (yes)
  group Get amber stop line violations #LightYellow {
    if (Trajectory Intersects stop line?) then (yes)
      if (Trajectory end exceeds tolerance threshold?) then (yes)
        :Compute time_to_cross AND ego_stopping_distance; <<#LightBlue>>
        if (ego_stopping_distance< distance to stop line OR time_to_cross > crossing_time_limit?) then (yes)
          :Record Amber Light Violation; <<#LightPink>>
        else (no)
        endif
      else (no)
      endif
    else (no)
    endif
  }
else (no)
endif

:Return Compliance Check Result; <<#LightGreen>>
stop

@enduml
```

### Signal Status Tracker Filtering

To prevent sudden, harsh emergency braking maneuvers caused by raw perception noise, the status tracker filters raw signals through three deterministic mechanisms before passing them to the validation layer:

- **State Persistence Buffering:** Incoming state transitions (e.g., green to amber/red) must continuously persist for a minimum time window (`stable_duration_threshold_red` or `stable_duration_threshold_amber`) before the new state is considered valid. If a signal flips color or alters elements before this duration threshold is reached, its active elements array is cleared to suppress transient sensor noise.
- **History Eviction Buffer:** If a traffic light group drops out of the incoming message matrix completely, the tracker retains its record for a brief clearing window. The duration an un-updated signal persists in memory is dynamically determined by its last recorded color state: `stable_duration_threshold_red` for red signals and `stable_duration_threshold_amber` for amber or other non-red conditions. If the signal remains un-detected beyond this frame timeout, its stale context is permanently erased from the tracking ledger.
- **Ego-Stopped Pass-Through Gate:** The tracker continuously evaluates the current motion of the ego vehicle via `is_ego_stopped`. When the vehicle has brought itself to a halt beneath the configured `ego_stopped_velocity_threshold`, the entire stability duration filtering logic is completely bypassed. This pass-through guarantees that the vehicle maintains maximum responsiveness to incoming state changes while stationary, eliminating filtering-induced latency during intersection departures.

### Safety & Compliance Logic

- **Segment-by-Segment Geometric Scan:** The checker evaluates trajectory segments sequentially against mapped stop lines using a localized `boost::geometry::intersection` check. The loop breaks immediately upon finding the first chronological intersection point, calculating the dynamic distance-to-stop-line and interpolating the exact crossing timestamp (for amber light) using `autoware::interpolation::lerp`.
- **Red Light Evaluation:** Generates a violation if an intersection occurs, unless the trajectory makes the ego come to a complete stop within the specified `stop_overshoot_margin`.
- **Amber Light Evaluation:** Computes the dynamic stopping distance based on current velocity, acceleration, applied deceleration, and system response latency. If the vehicle can stop safely before the line, or if it cannot clear the intersection within the `crossing_time_limit`, an amber violation is recorded to enforce a stop.

## Structs and Interface Definitions

The interfaces pass inputs and output results through the following standard data types:

### Inputs

| Field Name             | Data Type                      |
| :--------------------- | :----------------------------- |
| `trajectory`           | `std::vector<TrajectoryPoint>` |
| `map`                  | `lanelet::LaneletMapPtr`       |
| `route`                | `LaneletRoute`                 |
| `signals`              | `TrafficLightGroupArray`       |
| `current_time`         | `rclcpp::Time`                 |
| `current_velocity`     | `double`                       |
| `current_acceleration` | `double`                       |

### Violation

| Struct Field                | Data Type                    |
| :-------------------------- | :--------------------------- |
| `type`                      | `ViolationType`              |
| `stop_line`                 | `lanelet::BasicLineString2d` |
| `traffic_light_id`          | `int64_t`                    |
| `cross_point`               | `lanelet::BasicPoint2d`      |
| `arc_length_to_cross_point` | `double`                     |

## Parameters

| Parameter Name                                 | Type     | Description                                                                                |
| :--------------------------------------------- | :------- | :----------------------------------------------------------------------------------------- |
| `deceleration_limit`                           | `double` | Max deceleration limit during braking ($m/s^2$) for assessing stopping feasibility.        |
| `jerk_limit`                                   | `double` | Max jerk limit during braking ($m/s^3$) for assessing stopping feasibility.                |
| `delay_response_time`                          | `double` | Combined latency buffer for compute cycle lag and brake actuation (seconds).               |
| `crossing_time_limit`                          | `double` | Maximum duration allowed for the vehicle to clear an amber light intersection (seconds).   |
| `stop_overshoot_margin`                        | `double` | Allowed physical distance buffer beyond a stop line for a stopped vehicle (meters).        |
| `stable_duration_threshold_red`                | `double` | Required continuous duration for a `RED` state to be confirmed as valid (seconds).         |
| `stable_duration_threshold_amber`              | `double` | Required continuous duration for an `AMBER` state to be confirmed as valid (seconds).      |
| `amber_rejection_hysteresis_duration`          | `double` | Duration to retain an active amber light state if perception updates drop out (seconds).   |
| `ego_stopped_velocity_threshold`               | `double` | Velocity threshold beneath which the ego vehicle is considered completely stopped ($m/s$). |
| `treat_amber_light_as_red_light`               | `bool`   | If true, disables amber passing logic and treats all amber states as strict red signals.   |
| `treat_unknown_light_as_red_light`             | `bool`   | If true, evaluates unclassified or blank signal states as strict red signals.              |
| `checked_trajectory_length.deceleration_limit` | `double` | Comfortable stop deceleration limit ($m/s^2$) for computing trajectory checking length.    |
| `checked_trajectory_length.jerk_limit`         | `double` | Comfortable stop jerk limit ($m/s^3$) for computing trajectory checking length.            |
