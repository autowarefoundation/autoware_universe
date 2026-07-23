# autoware_path_distance_calculator

## Purpose

This node publishes the remaining distance from the self-position to the route goal.
The distance is the arc-length along the route's planned lanelet sequence between the current position and the goal, not the Euclidean distance between the two points.

## Inner-workings / Algorithms

On every timer tick (1 Hz):

1. If a new map or route message has arrived, it is handed to the calculator.
   - On a new map, the lanelet2 map is (re)built.
   - On a new route, the lanelet sequence is cached from `LaneletRoute::segments[].preferred_primitive.id`, looked up directly in the map. This is the lanelet sequence mission_planner actually planned the route through (accounting for lane exclusions, required lane changes, etc.), so it is used as-is instead of re-deriving a path with an independent shortest-path search, which could disagree with the actually planned route.
2. The self-position is matched to a lanelet in the cached sequence, and the remaining distance is computed as the sum of: the remaining length in the current lanelet, the full length of every lanelet strictly between the current and goal lanelet, and the length up to the goal pose within the goal lanelet.
3. The result is published. If the map/route are not ready yet, or the self-position cannot be matched to the cached sequence, nothing is published for that tick.

The core calculation (`RouteDistanceCalculator` in `src/main.cpp`) has no ROS node dependency; `src/ros_interface.cpp` only wires up polling subscribers, the timer, and the publisher around it.

## Inputs / Outputs

### Input

| Name            | Type                                        | Description                                                      |
| --------------- | ------------------------------------------- | ---------------------------------------------------------------- |
| `~/input/route` | `autoware_planning_msgs::msg::LaneletRoute` | Route, used to resolve the goal and the planned lanelet sequence |
| `~/input/map`   | `autoware_map_msgs::msg::LaneletMapBin`     | Lanelet2 map, used to look up lanelets by ID                     |
| `/tf`           | `tf2_msgs/TFMessage`                        | TF (self-pose)                                                   |

By default (see `launch/path_distance_calculator.launch.xml`), `~/input/route` is remapped to `/planning/mission_planning/route` and `~/input/map` to `/map/vector_map`.

### Output

| Name         | Type                                                | Description                                                     |
| ------------ | --------------------------------------------------- | --------------------------------------------------------------- |
| `~/distance` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Remaining distance from the self-position to the route goal [m] |

## Parameters

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

- The lanelet sequence is cached from `LaneletRoute::segments` when a new route message is received; it is not recomputed while driving on that same route. If the vehicle temporarily leaves that cached lanelet sequence without a reroute (e.g. a lane change to pass an obstacle), the self-position cannot be matched to the cached sequence and no distance is published until the vehicle returns to a lanelet on it.
- No distance is published until both a route and a map have been received and the self-position can be matched to a lanelet on the cached sequence.
