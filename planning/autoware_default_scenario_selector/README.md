# default_scenario_selector

## default_scenario_selector_node

`default_scenario_selector_node` is a ROS 2 node that switches trajectories between **LaneDriving** and **Parking** scenarios.

### Input topics

| Name                             | Type                                  | Description                                           |
|----------------------------------|---------------------------------------|-------------------------------------------------------|
| `~input/lane_driving/trajectory` | `autoware_planning_msgs::Trajectory`  | Trajectory of the LaneDriving scenario                |
| `~input/parking/trajectory`      | `autoware_planning_msgs::Trajectory`  | Trajectory of the Parking scenario                    |
| `~input/lanelet_map`             | `autoware_map_msgs::msg::LaneletMapBin` | Lanelet map                                         |
| `~input/route`                   | `autoware_planning_msgs::LaneletRoute` | Route and goal pose                                  |
| `~input/odometry`                | `nav_msgs::Odometry`                  | Used to check whether the vehicle is stopped          |
| `is_parking_completed`           | `bool` (rosparam)                     | Whether all split trajectories of Parking are published |

### Output topics

| Name                 | Type                                         | Description                                    |
|----------------------|----------------------------------------------|------------------------------------------------|
| `~output/scenario`   | `autoware_internal_planning_msgs::Scenario`  | Current scenario and scenarios to be activated |
| `~output/trajectory` | `autoware_planning_msgs::Trajectory`         | Trajectory to be followed                      |

### Output TFs

None

### How to launch

1. Write your remapping info in `default_scenario_selector.launch` or add args when executing `roslaunch`
2. `roslaunch autoware_default_scenario_selector default_scenario_selector.launch`
   - If you would like to use only a single scenario, `roslaunch autoware_default_scenario_selector dummy_default_scenario_selector_{scenario_name}.launch`

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
  if (is in lane?) then (yes)
    :set LaneDriving;
  else (no)
    :set Parking;
  endif

  stop
endif

' LaneDriving
if (current scenario is LaneDriving?) then (yes)
  if (is in parking lot & goal is not in lane?) then (yes)
    :set Parking;
    stop
  endif
endif

' Parking
if (current scenario is Parking?) then (yes)
  if (parking is completed and is in lane?) then (yes)
    :set LaneDriving;
    stop
  endif
endif

:continue previous scenario;

stop
@enduml
```
