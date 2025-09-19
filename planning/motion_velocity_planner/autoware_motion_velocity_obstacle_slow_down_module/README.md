# Obstacle Slow Down

## Role

The `obstacle_slow_down` module does the slow down planning when there is a static/dynamic obstacle near the trajectory.

## Activation

This module is activated if the launch parameter `launch_obstacle_slow_down_module` is set to true.

## Inner-workings / Algorithms

### Obstacle Filtering

Obstacles meeting the following condition are determined as obstacles for slowing down.

- The object type is for slowing down according to `obstacle_filtering.object_type.*`.
- The lateral distance from the object to the ego's trajectory is smaller than `obstacle_filtering.max_lat_margin` and larger than `obstacle_filtering.min_lat_margin`.
  - For the stable decision making, `obstacle_filtering.lat_hysteresis_margin` is applied for the hysteresis of the lateral margin.
- The obstacle which meets the condition `obstacle_filtering.successive_num_to_entry_slow_down_condition` times in a row will be a target obstacle
- The obstacle which was previously the target obstacle but does not meet the condition `obstacle_filtering.successive_num_to_entry_slow_down_condition` times in a row will not be a target obstacle.

### Slow Down Planning

The role of the slow down planning is inserting slow down velocity in the trajectory where the trajectory points are close to the obstacles. The parameters can be customized depending on the obstacle type, making it possible to adjust the slow down behavior depending if the obstacle is a pedestrian, bicycle, car, etc. Each obstacle type has a `static` and a `moving` parameter set, so it is possible to customize the slow down response of the ego vehicle according to the obstacle type and if it is moving or not. If an obstacle is determined to be moving, the corresponding `moving` set of parameters will be used to compute the vehicle slow down, otherwise, the `static` parameters will be used. The `static` and `moving` separation is useful for customizing the ego vehicle slow down behavior to, for example, slow down more significantly when passing stopped vehicles that might cause occlusion or that might suddenly open its doors.

An obstacle is classified as `static` if its total speed is less than the `moving_object_speed_threshold` parameter. Furthermore, a hysteresis based approach is used to avoid chattering, it uses the `moving_object_hysteresis_range` parameter range and the obstacle's previous state (`moving` or `static`) to determine if the obstacle is moving or not. In other words, if an obstacle was previously classified as `static`, it will not change its classification to `moving` unless its total speed is greater than `moving_object_speed_threshold` + `moving_object_hysteresis_range`. Likewise, an obstacle previously classified as `moving`, will only change to `static` if its speed is lower than `moving_object_speed_threshold` - `moving_object_hysteresis_range`.

The closest point on the obstacle to the ego's trajectory is calculated.
Then, the slow down velocity is calculated by linear interpolation with the distance between the point and trajectory as follows.

![slow_down_velocity_calculation](./docs/slow_down_velocity_calculation.svg)

| Variable   | Description                         |
| ---------- | ----------------------------------- |
| `v_{out}`  | calculated velocity for slow down   |
| `v_{min}`  | `min_ego_velocity`                  |
| `v_{max}`  | `max_ego_velocity`                  |
| `l_{min}`  | `min_lat_margin`                    |
| `l_{max}`  | `max_lat_margin`                    |
| `l'_{max}` | `obstacle_filtering.max_lat_margin` |

The parameters `min/max_ego_velocity` and `min/max_lat_margin` can be adjusted based on the obstacle type,
its side relative to the ego trajectory, and whether it is moving or not.
Default values must be provided at launch (`object_type_specified_params.default...`),
but each parameter can be overridden for specific cases (obstacle type, left or right, moving or not).
For example, the `min_ego_velocity` can be modified for moving cars on the left by setting a different value with parameter `object_type_specified_params.default.left.moving.min_ego_velocity`.

The calculated velocity is inserted in the trajectory where the obstacle is inside the area with `obstacle_filtering.max_lat_margin`.
More precisely, the velocity is inserted `<slow down velocity>` \* `slow_down_planning.time_margin_on_target_velocity` meters behind the obstacle.

![slow_down_planning](./docs/slow_down_planning.drawio.svg)

### Sudden deceleration suppression

When the slow down point is inserted, the deceleration and jerk is supposed to be higher than `slow_down_planning.slow_down_min_acc` and `slow_down_planning.slow_down_min_jerk` respectively. If the slow down point does not follow this condition, the velocity will be increased and the slow down point does not change.

### Stable slow down planning

The following low-pass filters will be applied.

- `slow_down_planning.lpf_gain_slow_down_vel`
  - slow down velocity
- `slow_down_planning.lpf_gain_lat_dist`
  - lateral distance of obstacles to the ego's trajectory to calculate the target velocity
- `slow_down_planning.lpf_gain_dist_to_slow_down`
  - distance to the slow down point

## Debugging

### Obstacle for slow down

Yellow sphere which is an obstacle for slow_down is visualized by `obstacles_to_slow_down` in the `~/debug/marker` topic.

Yellow wall which means a safe distance to slow_down if the ego's front meets the wall is visualized in the `~/debug/slow_down/virtual_wall` topic.

![slow_down_visualization](./docs/slow_down_visualization.png)
