# Rear Collision Checker

The `rear_collision_checker` is a plugin module of the `autoware_planning_validator` node. It validates the planned trajectory by verifying that it does **not** lead to a collision with other road users (primarily vehicles, but also pedestrian/cyclists/motorcycles) approaching from lateral/rear directions. In particular, it is designed to detect potential rear-end collisions and entrapment risks when the ego vehicle is making a right/left turn or merging into an adjacent lane.

## Inner Workings

The module operates by:

1. **Identifying conflict lanes**
   - Uses the current route and lanelet topology to determine lanes that the ego vehicle’s trajectory may intersect with during right/left turns or lane changes.
   - Focuses on lanes where vehicles, bicycles, or motorcycles could approach from behind or from a lateral direction, posing a potential rear-end or entrapment risk.
   - Considers the `turn_direction` of the approaching lanelets to filter out irrelevant lanes.

2. **Filtering perception data**
   - Subscribes to obstacle pointcloud and first filters by the conflict region, keeping only points that fall within relevant spatial bounds.
   - Performs **clustering** on the filtered points to form obstacle candidates.
   - Associates clusters with nearby lanelets and computes the **nearest face of each obstacle along the lane direction**.
   - Applies configurable range gates (forward/backward, lateral, height) and basic outlier rejection to discard distant or irrelevant clusters.

3. **Estimating motion**
   - For each selected object, determines its motion relative to the lane direction.
   - Estimates speed and direction based on frame-to-frame position changes along the lane’s centerline.
   - Maintains a per-lane tracking list to stabilize velocity estimation and reduce noise from perception flicker.

4. **Calculating metrics**
   - **Depending on the selected metric (e.g., TTC, RSS), the calculation method and decision logic differ.** The chosen metric is computed, and the result is used to determine whether there is a potential collision risk.

5. **Judging collision risk**
   - Evaluates the selected metric results against predefined thresholds to determine whether a collision is likely.
   - Applies hysteresis logic:
     - **`on_time_buffer`**: Hazardous state must persist for this duration before marking the trajectory unsafe.
     - **`off_time_buffer`**: Safe state must persist for this duration before clearing the unsafe flag.
   - If `check_on_unstoppable=false`, the module will skip collision warnings in cases where the ego vehicle cannot realistically stop before the conflict area (to prevent unnecessary alerts when stopping is already infeasible).

### Flowchart

WIP

## General Parameters

| Name                   | Unit | Type   | Description                                                                                                                                                                                                                                                                 | Default value |
| :--------------------- | ---- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `on_time_buffer`       | [s]  | double | Time buffer before enabling detection after a relevant condition is met                                                                                                                                                                                                     | 0.5           |
| `off_time_buffer`      | [s]  | double | Time buffer before disabling detection after the condition clears                                                                                                                                                                                                           | 1.5           |
| `check_on_unstoppable` | [-]  | bool   | If `true`, the module continues collision checking even when the ego vehicle cannot stop before entering a potential collision area, and outputs an **ERROR** if a collision risk is detected. If `false`, checking is skipped in such cases to avoid unnecessary warnings. | false         |

### Pointcloud Preprocess

| Name                                              | Unit     | Type   | Description                                                               | Default value |
| :------------------------------------------------ | -------- | ------ | ------------------------------------------------------------------------- | ------------- |
| `pointcloud.range.dead_zone`                      | [m]      | double | Distance in front of the ego vehicle ignored for collision detection      | 0.3           |
| `pointcloud.range.buffer`                         | [m]      | double | Additional margin around detection range                                  | 1.0           |
| `pointcloud.crop_box_filter.x.max`                | [m]      | double | Maximum X coordinate for cropped detection range                          | 30.0          |
| `pointcloud.crop_box_filter.x.min`                | [m]      | double | Minimum X coordinate for cropped detection range                          | -100.0        |
| `pointcloud.crop_box_filter.z.max`                | [m]      | double | Maximum Z coordinate for cropped detection range                          | -1.0          |
| `pointcloud.crop_box_filter.z.min`                | [m]      | double | Minimum Z coordinate for cropped detection range                          | 0.3           |
| `pointcloud.voxel_grid_filter.x`                  | [m]      | double | Voxel grid filter size in X direction                                     | 0.1           |
| `pointcloud.voxel_grid_filter.y`                  | [m]      | double | Voxel grid filter size in Y direction                                     | 0.1           |
| `pointcloud.voxel_grid_filter.z`                  | [m]      | double | Voxel grid filter size in Z direction                                     | 0.5           |
| `pointcloud.clustering.cluster_tolerance`         | [m]      | double | Maximum distance between points to be considered part of the same cluster | 0.15          |
| `pointcloud.clustering.min_cluster_height`        | [m]      | double | Minimum height of a cluster to be considered valid                        | 0.1           |
| `pointcloud.clustering.min_cluster_size`          | [points] | int    | Minimum number of points in a valid cluster                               | 5             |
| `pointcloud.clustering.max_cluster_size`          | [points] | int    | Maximum number of points in a valid cluster                               | 10000         |
| `pointcloud.velocity_estimation.observation_time` | [s]      | double | Time window used for velocity estimation                                  | 0.3           |
| `pointcloud.velocity_estimation.max_acceleration` | [m/s^2]  | double | Maximum allowed acceleration in velocity estimation                       | 10.0          |
| `pointcloud.latency`                              | [s]      | double | Assumed system latency for point cloud processing                         | 0.3           |

### Object Filtering

| Name                  | Unit  | Type   | Description                                                             | Default value |
| :-------------------- | ----- | ------ | ----------------------------------------------------------------------- | ------------- |
| `filter.min_velocity` | [m/s] | double | Minimum velocity threshold for objects to be considered moving          | 1.0           |
| `filter.moving_time`  | [s]   | double | Minimum time duration an object must be moving to be considered as such | 0.5           |

### TTC

| Name                       | Unit | Type   | Description                                              | Default value |
| :------------------------- | ---- | ------ | -------------------------------------------------------- | ------------- |
| `time_to_collision.margin` | [s]  | double | Additional margin added to time-to-collision calculation | 2.0           |

### Ego Behavior

| Name                        | Unit    | Type   | Description                                                         | Default value |
| :-------------------------- | ------- | ------ | ------------------------------------------------------------------- | ------------- |
| `ego.reaction_time`         | [s]     | double | Reaction time of the ego vehicle                                    | 1.2           |
| `ego.min_velocity`          | [m/s]   | double | Minimum considered velocity of the ego vehicle                      | 1.38          |
| `ego.max_velocity`          | [m/s]   | double | Maximum considered velocity of the ego vehicle                      | 16.6          |
| `ego.max_acceleration`      | [m/s^2] | double | Maximum considered acceleration of the ego vehicle                  | 1.5           |
| `ego.max_deceleration`      | [m/s^2] | double | Maximum considered deceleration of the ego vehicle (negative value) | -4.0          |
| `ego.max_positive_jerk`     | [m/s^3] | double | Maximum allowed positive jerk for the ego vehicle                   | 5.0           |
| `ego.max_negative_jerk`     | [m/s^3] | double | Maximum allowed negative jerk for the ego vehicle                   | -5.0          |
| `ego.nominal_deceleration`  | [m/s^2] | double | Nominal deceleration used for calculations                          | -1.5          |
| `ego.nominal_positive_jerk` | [m/s^3] | double | Nominal positive jerk used for calculations                         | 0.6           |
| `ego.nominal_negative_jerk` | [m/s^3] | double | Nominal negative jerk used for calculations                         | -0.6          |

### Collision Check For Blind Spot

| Name                                       | Unit    | Type   | Description                                                  | Default value |
| :----------------------------------------- | ------- | ------ | ------------------------------------------------------------ | ------------- |
| `blind_spot.lookahead_time`                | [s]     | double | Lookahead time for blind spot detection                      | 4.0           |
| `blind_spot.metric`                        | [-]     | string | Metric used for blind spot detection (`ttc`, etc.)           | ttc           |
| `blind_spot.check.front`                   | [-]     | bool   | Whether to check for blind spot in the front                 | false         |
| `blind_spot.check.left`                    | [-]     | bool   | Whether to check for blind spot on the left                  | true          |
| `blind_spot.check.right`                   | [-]     | bool   | Whether to check for blind spot on the right                 | false         |
| `blind_spot.check.yaw_th`                  | [rad]   | double | Yaw threshold for blind spot detection                       | 0.78          |
| `blind_spot.offset.inner`                  | [m]     | double | Inner offset for blind spot detection zone                   | 0.1           |
| `blind_spot.offset.outer`                  | [m]     | double | Outer offset for blind spot detection zone                   | 0.3           |
| `blind_spot.participants.reaction_time`    | [s]     | double | Reaction time of participants in blind spot detection        | 1.2           |
| `blind_spot.participants.max_velocity`     | [m/s]   | double | Maximum velocity of participants in blind spot detection     | 5.5           |
| `blind_spot.participants.max_deceleration` | [m/s^2] | double | Maximum deceleration of participants in blind spot detection | -2.0          |

### Collision Check For Adjacent Lane

| Name                                          | Unit    | Type   | Description                                                               | Default value |
| :-------------------------------------------- | ------- | ------ | ------------------------------------------------------------------------- | ------------- |
| `adjacent_lane.lookahead_time`                | [s]     | double | Lookahead time for adjacent lane collision detection                      | 4.0           |
| `adjacent_lane.metric`                        | [-]     | string | Metric used for adjacent lane collision detection (`rss`, etc.)           | rss           |
| `adjacent_lane.check.front`                   | [-]     | bool   | Whether to check for adjacent lane collision in the front                 | true          |
| `adjacent_lane.offset.left`                   | [m]     | double | Left offset for adjacent lane collision detection                         | -0.5          |
| `adjacent_lane.offset.right`                  | [m]     | double | Right offset for adjacent lane collision detection                        | -0.5          |
| `adjacent_lane.participants.reaction_time`    | [s]     | double | Reaction time of participants in adjacent lane collision detection        | 1.2           |
| `adjacent_lane.participants.max_velocity`     | [m/s]   | double | Maximum velocity of participants in adjacent lane collision detection     | 16.6          |
| `adjacent_lane.participants.max_deceleration` | [m/s^2] | double | Maximum deceleration of participants in adjacent lane collision detection | -2.0          |
