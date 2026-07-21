# autoware_path_distance_calculator

## Purpose

This node publishes the remaining distance from the self-position to the route goal.
The distance is the arc-length along the shortest path (in lanelets) between the current position and the goal, not the Euclidean distance between the two points.

## Inner-workings / Algorithms

On every timer tick (1 Hz):

1. If a new map or route message has arrived, it is handed to the calculator.
   - On a new map, the lanelet2 routing graph is (re)built.
   - On a new route, the shortest path (sequence of lanelets) from the self-position at that moment to the route goal is resolved and cached via the routing graph. This avoids re-searching the graph every tick.
2. The self-position is matched to a lanelet in the cached shortest path, and the remaining distance is computed as the sum of: the remaining length in the current lanelet, the full length of every lanelet strictly between the current and goal lanelet, and the length up to the goal pose within the goal lanelet.
3. The result is published. If the map/route are not ready yet, or the self-position cannot be matched to the cached path, nothing is published for that tick.

The core calculation (`RouteDistanceCalculator` in `src/main.cpp`) has no ROS node dependency; `src/ros_interface.cpp` only wires up polling subscribers, the timer, and the publisher around it.

## Inputs / Outputs

### Input

| Name            | Type                                        | Description                                           |
| --------------- | ------------------------------------------- | ----------------------------------------------------- |
| `~/input/route` | `autoware_planning_msgs::msg::LaneletRoute` | Route, used to resolve the goal and the shortest path |
| `~/input/map`   | `autoware_map_msgs::msg::LaneletMapBin`     | Lanelet2 map, used to build the routing graph         |
| `/tf`           | `tf2_msgs/TFMessage`                        | TF (self-pose)                                        |

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

- The shortest path (a sequence of lanelets) is resolved via the lanelet2 routing graph and cached when a new route message is received; it is not recomputed while driving on that same route. If the vehicle temporarily leaves that cached lanelet sequence without a reroute (e.g. a lane change to pass an obstacle), the self-position cannot be matched to the cached path and no distance is published until the vehicle returns to a lanelet on it.
- No distance is published until both a route and a map have been received and the self-position can be matched to a lanelet on the cached path.
