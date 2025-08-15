# autoware_extra_scenario_selector

## extra_scenario_selector_node

`extra_scenario_selector_node` is a ROS 2 node that switches trajectories among **LaneDriving / Parking / WaypointFollowing** scenarios.

### Input topics

| Name                                   | Type                                               | Description                                                        |
|----------------------------------------|----------------------------------------------------|--------------------------------------------------------------------|
| `~input/lane_driving/trajectory`       | `autoware_planning_msgs::Trajectory`               | Trajectory for the LaneDriving scenario                            |
| `~input/parking/trajectory`            | `autoware_planning_msgs::Trajectory`               | Trajectory for the Parking scenario                                |
| `~input/waypoint_following/trajectory` | `autoware_planning_msgs::Trajectory`               | Trajectory for the WaypointFollowing scenario                      |
| `~input/lanelet_map`                   | `autoware_map_msgs::msg::LaneletMapBin`            | Lanelet2 map                                                        |
| `~input/route`                         | `autoware_planning_msgs::LaneletRoute`             | Route and goal pose                                                 |
| `~input/odometry`                      | `nav_msgs::Odometry`                               | Used to check whether the vehicle is stopped                       |
| `~input/operation_mode_state`          | `autoware_adapi_v1_msgs::msg::OperationModeState`  | Used to check if autonomous driving mode and control are enabled   |
| `is_parking_completed`                 | `bool` (rosparam)                                  | Whether all split trajectories of the Parking scenario are complete |
| `is_waypoint_following_completed`      | `bool` (rosparam)                                  | Whether WaypointFollowing scenario is completed                    |

### Output topics

| Name                 | Type                                         | Description                                              |
|----------------------|----------------------------------------------|----------------------------------------------------------|
| `~output/scenario`   | `autoware_internal_planning_msgs::Scenario`  | Current scenario and the list of active scenarios        |
| `~output/trajectory` | `autoware_planning_msgs::Trajectory`         | The trajectory to be followed by the vehicle             |

### Output TFs

None

### How to launch

1. Write your remapping configuration in `extra_scenario_selector.launch` or pass the arguments when launching.
2. `roslaunch autoware_extra_scenario_selector extra_scenario_selector.launch`
   - If you would like to use only a single scenario, `roslaunch autoware_extra_scenario_selector dummy_extra_scenario_selector_{scenario_name}.launch`

### Parameters

{{ json_to_markdown("planning/autoware_scenario_selector/schema/scenario_selector.schema.json") }}

### Flowchart

```plantuml
@startuml
title onTimer
start

:get current pose;

if (all input data are ready?) then (yes)
else (no)
  stop
endif

if (scenario is initialized?) then (yes)
else (no)
  :initialize scenario;
endif

:select scenario;

:publish scenario;

:extract scenario trajectory;

if (scenario trajectory is empty?) then (yes)
else (no)
  :publish trajectory;
endif

stop
@enduml
```

```plantuml
@startuml
title Scenario Transition
start

if (current_scenario is completed?\n()) then (yes)
else (no)
  stop
endif

' Empty
if (scenario is initialized?) then (yes)
else (no)
  if (is in parking lot?) then (yes)
    :set Parking;
  else (no)
    if (is in waypoint-following zone?) then (yes)
      :set WaypointFollowing;
    else (no)
      if (is in lane?) then (yes)
        :set LaneDriving;
      else (no)
        :set LaneDriving;
      endif
    endif
  endif

  stop
endif

' LaneDriving
if (current scenario is LaneDriving?) then (yes)
  if (is in parking lot & goal is not in lane?) then (yes)
    :set Parking;
    stop
  else (no)
    if (is in waypoint-following zone?) then (yes)
      :set WaypointFollowing;
    endif
  endif
endif

' Parking
if (current scenario is Parking?) then (yes)
  if (parking is completed and is in lane?) then (yes)
    :set LaneDriving;
    stop
  else (no)
    if (parking is completed and is in waypoint-following zone?) then (yes)
      :set WaypointFollowing;
      stop
    endif
  endif
endif

' WaypointFollowing
if (current scenario is WaypointFollowing?) then (yes)
  if (is in parking lot & goal is not in lane?) then (yes)
    :set Parking;
  else (no)
    if (is in lane and (waypoint following is completed or not in waypoint-following zone)?) then (yes)
      :set LaneDriving;
    endif
  endif
endif

:continue previous scenario;

stop
@enduml
```
