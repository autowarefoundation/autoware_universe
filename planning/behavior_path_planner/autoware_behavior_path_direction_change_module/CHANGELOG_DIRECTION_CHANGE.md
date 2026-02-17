# Changelog – Direction Change Module

## [Unreleased]

### Changed

- **Reference path from direction_change lanelet centerline (path flip fix)**
  - When the path runs through lanelets with `direction_change_lane: "yes"`, the module now builds the reference path from the **full centerline of those lanelets** (from the map) instead of using the path from the previous module.
  - **Reason:** On a single lanelet that forms a cross (e.g. two cusps), the upstream reference path is built from the "current" position on the centerline. Near the cross center, the nearest point on the centerline can jump between branches (vertical vs horizontal), so the reference path could flip segment (vertical ↔ horizontal). Using the lanelet centerline directly gives a stable, fixed geometry for the direction_change segment.
  - **Impact:** Cusp detection and segment publishing (forward/reverse, multi-cusp, sustained stop) now operate on a stable reference path in direction_change areas. No change to behavior outside direction_change lanelets (previous module output is still used there).
  - **Resampling:** No resampling is applied inside the module for this path; the planner manager resamples the final path with `output_path_interval` after all modules run.
  - **Files:**
    - `include/.../utils.hpp`: added `getReferencePathFromDirectionChangeLanelets()` declaration.
    - `src/utils.cpp`: implemented `getReferencePathFromDirectionChangeLanelets()` (collects direction_change lane_ids from path in order, gets full centerline via `route_handler->getCenterLinePath(..., 0, max)`, returns path with route header).
    - `src/scene.cpp`: `updateData()` calls the helper when `planner_data_` and `route_handler` are available; if the returned path is non-empty, uses it as `reference_path_`, otherwise falls back to previous module output.
