# autoware_obstacle_grid_utils

Consumer helpers for the obstacle grid published on
`/sensing/obstacle_segmentation/obstacle_grid` (`grid_map_msgs/msg/GridMap`, frame `base_link`,
layers `max_height` / `min_height` / `point_count`, `NaN` = empty cell).

The grid is a shared last-resort safety product. These helpers exist so every consumer
(collision_detector, surround_obstacle_checker, obstacle_collision_checker, planning_validator,
trajectory ObstacleStop, AEB, ...) queries the grid through one reviewed implementation instead of
hand-copying the same per-cell math.

## Contract reminder

Consumers must validate `frame_id == base_link` and the presence of the required layers on intake,
and treat any contract violation or a stale stamp as **unavailable** (never as "clear"). Staleness
is a per-consumer stamp-age watchdog (parameter name and default timeout TBD per consumer); an
all-`NaN` grid with a fresh stamp is a valid alive heartbeat.

## Helpers

All helpers are declared in namespace `autoware::obstacle_grid_utils` in
`include/autoware/obstacle_grid_utils/obstacle_grid_utils.hpp` and implemented in
`src/obstacle_grid_utils.cpp`. The cell footprint box that makes the distance queries edge-aware
is an implementation detail of that translation unit, not part of the public surface.

| Helper                                      | Purpose                                                                                                                                                                                                                                                                                      |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Gate{min_point_count_cell, min_height}`    | Per-cell qualification threshold: a cell qualifies iff `point_count >= min_point_count_cell && max_height >= min_height`. Height is `max_height`-based.                                                                                                                                      |
| `cell_qualifies(grid, index, gate)`         | Whether one cell passes the gate (`NaN`-safe).                                                                                                                                                                                                                                               |
| `cell_corners(center, resolution)`          | The 4 corner points of the cell footprint (min-min, min-max, max-max, max-min) for edge-conservative corridor/lane membership.                                                                                                                                                               |
| `nearest_distance(grid, ego_polygon, gate)` | Edge-aware 2D distance from `ego_polygon` to the nearest qualifying cell; `+inf` when nothing qualifies inside the ROI.                                                                                                                                                                      |
| `nearest_cell(grid, ego_polygon, gate)`     | Same edge-aware distance **and** where the nearest qualifying cell is (`std::optional<NearestCell{distance, position, index}>`, `std::nullopt` when nothing qualifies). On a distance tie the first cell reached by the grid iterator wins (either-of).                                      |
| `connected_components(grid, qualifying)`    | 8-connected labeling of the **caller-supplied** qualifying cell indices; returns `std::vector<CellComponent{cells, point_sum}>` with `point_sum` the sum of `point_count` over each component. Does not gate — the caller applies its `Gate` first. Components come out in first-seen order. |
