# Side Shift design

(For remote control) Shift the path to left or right according to an external instruction.

## Overview of the Side Shift Module Process

1. Receive the required lateral offset from the topic and/or the `~/set_lateral_offset` service.
2. Update the `requested_lateral_offset_` under the following conditions:
   a. Verify if the last update time has elapsed.
   b. Ensure the required lateral offset value is different from the previous one.
3. Insert the shift points into the path if the side shift module's status is not in the SHIFTING status.

Please be aware that `requested_lateral_offset_` is continuously updated with the latest values and is not queued.

## Lateral offset inputs and outputs

Offsets use the same sign convention as `tier4_planning_msgs/msg/LateralOffset` (**positive = left** of the reference path, **negative = right**).

| Interface    | Message / service                                               | Role                                                                                                                                                                                               |
| ------------ | --------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Subscription | `~/input/lateral_offset` (`tier4_planning_msgs/LateralOffset`)  | Stream of requested lateral offset [m].                                                                                                                                                            |
| Service      | `~/set_lateral_offset` (`tier4_planning_msgs/SetLateralOffset`) | Request a new offset with a response: `EXPLICIT_LATERAL_OFFSET_AMOUNT` sets `shift_value` [m] directly; `LATERAL_OFFSET_DIRECTION` uses `LEFT` / `RIGHT` / `RESET` with step size from parameters. |
| Publisher    | `~/output/lateral_offset` (`tier4_planning_msgs/LateralOffset`) | Reports the **current** inserted lateral offset [m] for feedback.                                                                                                                                  |

The same validation (magnitude limits, minimum gap versus the previous offset, and so on) applies whether the request arrives on the topic or through the service.
It is recommended to use services so that the user can understand whether the request is accepted or not.

### `~/set_lateral_offset` response `status.code` values

The service fills `autoware_common_msgs/ResponseStatus` (`success`, `code`, `message`). The numeric `code` is either a constant from `tier4_planning_msgs/srv/SetLateralOffset` (service-specific) or from `autoware_common_msgs/msg/ResponseStatus` (shared infrastructure).

| `code` value | Constant                    | Typical `success` | Meaning                                                                                                                                                                                                                  |
| -----------: | --------------------------- | :---------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|            0 | `SUCCESS`                   |      `true`       | Request accepted; lateral offset updated as requested.                                                                                                                                                                   |
|        10000 | `ERROR_UNKNOWN`             |      `false`      | Unspecified error.                                                                                                                                                                                                       |
|        10001 | `ERROR_INVALID_MODE`        |      `false`      | `shift_mode` is not a supported value.                                                                                                                                                                                   |
|        10002 | `ERROR_INVALID_DIRECTION`   |      `false`      | In `LATERAL_OFFSET_DIRECTION` mode, `shift_direction_value` is not `RESET`, `LEFT`, or `RIGHT`.                                                                                                                          |
|        10003 | `ERROR_EXCEEDED_LIMIT`      |      `false`      | The path is shifted **already at limit** (configured by parameters) and your request is trying to go even further.                                                                                                       |
|        10004 | `ERROR_SHIFT_GAP_TOO_SMALL` |      `false`      | Change versus the current offset is smaller than `min_shift_gap` (treated as no meaningful update).                                                                                                                      |
|        10005 | `ERROR_MODULES_CONFLICTING` |      `false`      | Cannot perform side shift because modules that cannot be run together with the side_shift module is active in the `behavior_path_planner`                                                                                |
|        20000 | `WARN_UNKNOWN`              |      `false`      | Unspecified warning.                                                                                                                                                                                                     |
|        20001 | `WARN_EXCEEDED_LIMIT`       |      `true`       | You request reached the limit (configured by parameters) and side_shift module tries to shift the path to the possible maximum. If you try shift even further next, then the service will return `ERROR_EXCEEDED_LIMIT`. |
|        50001 | `SERVICE_UNREADY`           |      `false`      | `behavior_path_planner` is not ready; request ignored.                                                                                                                                                                   |
|        50004 | `PARAMETER_ERROR`           |      `false`      | Invalid side-shift node parameters were detected (e.g. non-positive `unit_shift_amount` when using direction steps).                                                                                                     |

`success` is set to `true` only when `code` is `SUCCESS` (0) or `WARN_EXCEEDED_LIMIT` (20001); in those cases the requested offset (possibly clamped) is applied. For all other rows, the previous requested offset is left unchanged.

## Statuses of the Side Shift

The side shift has three distinct statuses. Note that during the SHIFTING status, the path cannot be updated:

1. BEFORE_SHIFT: Preparing for shift.
2. SHIFTING: Currently in the process of shifting.
3. AFTER_SHIFT: Shift completed.

<figure markdown>
  ![case1](images/side_shift_status.drawio.svg){width=1000}
  <figcaption>side shift status</figcaption>
</figure>

## Flowchart

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title callback function of lateral offset input
start

partition onLateralOffset {
:**INPUT** double new_lateral_offset;

if (abs(inserted_lateral_offset_ - new_lateral_offset) < 1e-4) then ( true)
  stop
else ( false)
  if (interval from last request is too short) then ( no)
  else ( yes)
  :requested_lateral_offset_ = new_lateral_offset \n lateral_offset_change_request_ = true;
  endif
stop
@enduml
```

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title path generation

start
partition plan {
if (lateral_offset_change_request_ == true \n && \n (shifting_status_ == BEFORE_SHIFT \n || \n shifting_status_ == AFTER_SHIFT)) then ( true)
  partition replace-shift-line {
    if ( shift line is inserted in the path ) then ( yes)
      :erase left shift line;
    else ( no)
    endif
    :calcShiftLines;
    :add new shift lines;
    :inserted_lateral_offset_ = requested_lateral_offset_ \n inserted_shift_lines_ = new_shift_line;
  }
else( false)
endif
stop
@enduml
```

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title update state

start
partition updateState {
  :last_sp = path_shifter_.getLastShiftLine();
  note left
  get furthest shift lines
  end note
  :calculate max_planned_shift_length;
  note left
  calculate furthest shift length of previous shifted path
  end note
  if (abs(inserted_lateral_offset_ - inserted_shift_line_.end_shift_length) < 1e-4 \n && \n abs(max_planned_shift_length) < 1e-4 \n && \n abs(requested_lateral_offset_) < 1e-4) then ( true)
    :current_state_ = BT::NodeStatus::SUCCESS;
  else (false)
    if (ego's position is behind of shift line's start point) then( yes)
      :shifting_status_ = BEFORE_SHIFT;
    else ( no)
      if ( ego's position is between shift line's start point and end point) then (yes)
        :shifting_status_ = SHIFTING;
      else( no)
        :shifting_status_ = AFTER_SHIFT;
      endif
    endif
    :current_state_ = BT::NodeStatus::RUNNING;
  endif
  stop
}

@enduml
```
