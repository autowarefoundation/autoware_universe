# Run Out

## Role

The `run_out` module adds deceleration and stop points to the ego trajectory in order to prevent collisions with objects that are moving towards the ego vehicle path.

## Activation

This module is activated if the launch parameter `launch_mvp_run_out_module` is set to true.

## Inner-workings / Algorithms

This module calculates the times when the ego vehicle and the objects are predicted to overlaps each other's trajectories.
These times are then used to decide whether to stop before the overlap or not.

Next we explain the inner-workings of the module in more details.

### 1. Ego trajectory footprint

In this first step, the trajectory footprint is constructed from the corner points of the vehicle.
4 linestrings are constructed from the 4 corners (front left, front right, rear left, rear right) projected at each trajectory point.

At this step, the footprint size can be adjusted using the `ego.lateral_margin` and `ego.longitudinal_margin` parameters.

The following figures show the 4 corner linestrings corresponding calculate for the red trajectory.

| front left                                                | front right                                                 | rear left                                               | rear right                                                |
| --------------------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------------- | --------------------------------------------------------- |
| ![ego_front_left_footprint](./docs/front_left_ego_ls.png) | ![ego_front_right_footprint](./docs/front_right_ego_ls.png) | ![ego_rear_left_footprint](./docs/rear_left_ego_ls.png) | ![ego_rear_right_footprint](./docs/rear_right_ego_ls.png) |

These can be visualized with the debug markers with the `ego_footprint_(front|rear)_(left|right)` namespaces.

### 2. Extracting map filtering data

In the second step, we extract geometric information from the vector map that will be used to filter dynamic objects.
For each object classification label,
we prepare geometries based on the parameters defined for that label (`objects.{CLASSIFICATION_LABEL}`).

Two sets of geometries (polygons and segments) will be prepared and stored in Rtree objects for efficient spatial queries in the next step.

#### Polygons to ignore objects

2D polygons are extracted from crosswalk polygons if parameter `ignore.if_on_crosswalk` is `true`.
Additionally, two polygons built from the ego trajectory footprint's front and rear linestrings are used if `ignore.if_on_trajectory` is `true`.

#### Segments to cut predicted paths

2D segments are extracted from linestrings and polygons based on the types provided in the `cut_predicted_paths.polygon_types` and `cut_predicted_paths.linestring_types` parameters.
Additionally, if `cut_predicted_paths.if_crossing_ego_from_behind` is `true` then the rear segment of the ego vehicle footprint will also be used.

TODO: figure

TODO: add debug marker explanation

### 3. Dynamic objects filtering

In this step, objects and their predicted paths are filtered based on its classification label and the corresponding parameters `objects.{CLASSIFICATION_LABEL}`.

An object is ignored if one of the following condition is true:

- its classification label is not in the list defined by the `objects.target_labels` parameter;
- its velocity is bellow the `ignore.stopped_velocity_threshold` and `ignore.if_stopped` is set to `true`;
- its current footprint is inside one of the polygons prepared in the previous step.

However, if it was decided to stop for the object in the previous iteration,
or if a collision was detected with the object, then it cannot be ignored.

If an object is not ignored, its predicted path footprints are generated similarly to the ego footprint
First, we only keep predicted paths that have a confidence value above the `confidence_filtering.threshold` parameter.
If, `confidence_filtering.only_use_highest` is set to `true` then for each object only the predicted paths that have the higher confidence value are kept.
Next, the remaining predicted paths are cut according to the segments prepared in the previous step.

### 4. Collision detection

Now that we prepared the ego trajectory footprint, the dynamic objects, and their predicted paths,
we will calculate the times when they are predicted to collide.

The following operation are performed for each object that was not ignored in the previous iteration.

First, we calculate the intersections between each pair of linestrings between the ego and object footprints.
For each intersection, we calculate the corresponding point,
the time when ego and the object are predicted to reach that point,
and the location of that point on the ego footprint (e.g., on the rear left linestring).

All these intersections are then combined into intervals representing when the overlap between the ego trajectory and object predict paths starts and ends.
An overlap is represented by the entering and exiting intersections for both ego and the object.

These overlaps calculated for all the object's predicted paths are then combined if overlapping in time
and classified into the following collision types:

- `pass_first_collision` if ego is predicted to enter the overlap before the object.
  - only used if parameter `passing.enable_passing_margin` is set to `true`.
  - ego must enter the overlap at least `passing.time_margin` seconds before the object.
  - ego must not be overlapping the object's path by more than `passing.max_overlap_duration` seconds.
- `collision` if ego and the object are predicted to be in the overlap at the same time (i.e., the overlap time intervals overlap).
- `pass_first_no_collision` if ego is predicted to exit the overlap before the object enters it.
- `no_collision` in all other cases.

TODO: figure

TODO: add debug marker explanation

### 5. Decisions

We will now make the decision towards each object on whether to stop, slowdown, or do nothing.
For each object, we consider what decision each collision require, and keep the one with highest priority as follows:

- `stop` types have higher priority than `slowdown` types.
- for two decisions of the same type, the one whose predicted collision time is earlier has higher priority.

Once a decision is made, the history of the object is updated and will allow to know,
for each previous time step, what was the decision made and the type of collision identified.

To decide the type of a collision, we use the decision history of the object
and first check if it satisfies the following conditions to stop:

- if the current collision type is `collision` and collisions with the object have been identified for a consecutive duration of at least `stop.on_time_buffer` seconds.
- if the previous decision was `stop` and the time since the last identified collision with the object was less than `stop.off_time_buffer` seconds ago.

If the condition to stop is not met, we check the following conditions to slowdown as follows:

- if the current collision type is `collision` and collisions with the object have been identified for a consecutive duration of at least `preventive_slowdown.on_time_buffer` seconds.
- if the previous decision was `slowdown` and the time since the last identified collision with the object was less than `preventive_slowdown.off_time_buffer` seconds ago.

TODO: figure

TODO: add debug marker explanation

### 6. Calculate the stop or slowdowns

Finally, for each object, we calculate how the velocity profile will be modified based on the decision made:

- `stop`: insert a `0` velocity ahead of the predicted collision point by the distance set in the `stop.distance_buffer` parameter.
- `slowdown`: insert a $V_{slow}$ velocity between the collision point and the point ahead of collision point by the distance set in the `preventive_slowdown.distance_buffer` parameter.
  - $V_{slow}$ is calculated as the maximum between the safe velocity and the comfortable velocity.
    - safe velocity: velocity required to be able to stop over the `distance_buffer` assuming a deceleration as set by the
      `preventive_slowdown.safe_deceleration` parameter.
    - comfortable velocity: velocity ego would reached assuming it constantly decelerates at the
      `preventive_slowdown.comfortable_deceleration` parameter.

The slowdowns and stops inserted in the trajectory are visualized with the virtual walls.

![virtual_walls](./docs/stop_slow_virtual_walls.png)

### Use of Rtree for fast spatial queries

In step 1, each segment of the 4 linestrings of the ego trajectory footprint are stored in a Rtree
along with the corresponding trajectory point index.
This allows to efficiently find intersections with an object's predicted path along with the corresponding ego trajectory segment
from which the interpolated `time_from_start` can be calculated.

In step 2, the polygons and linestrings used for filtering the objects are stored in Rtree objects to efficiently find whether
an object is inside a polygon or if its predicted path intersects a linestring.

### Accounting for prediction inaccuracies

When calculating predicted collisions between ego and the objects,
we assume that the input ego trajectory contains accurate `time_from_start` values.
Similarly, accurate predicted paths are expected to be provided for the objects.

To allow for errors in these predictions, margins around the time intervals can be added using the parameters
`ego_time_interval_expansion` and `TODO`.
Higher values of this parameter will make it more likely to detect a collision and generate a stop.

## Module Parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/run_out.schema.json") }}

### Ego parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/sub/ego.sub_schema.json") }}

### Objects parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/sub/object.sub_schema.json") }}

### Passing parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/sub/passing.sub_schema.json") }}

### Preventive slowdown parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/sub/preventive_slowdown.sub_schema.json") }}

### Stop parameters

{{ json_to_markdown("planning/motion_velocity_planner/autoware_motion_velocity_run_out_module/schema/sub/stop.sub_schema.json") }}

## Flow Diagram

![Flow diagram](./docs/flow_mermaid_diagram.svg)
