# autoware_obstacle_grid_extractor

A stateless, `base_link`, per-cloud producer that rasterizes the no-ground obstacle point cloud into a single 2.5D occupancy/height grid (`grid_map_msgs/msg/GridMap`) for the perception-independent last-resort safety path.

## Purpose

The last-resort emergency-stop consumers (AEB pointcloud branch, collision/surround/obstacle-collision checkers, planning-validator collision checkers, trajectory-modifier obstacle-stop) each used to re-implement the same crop + voxel + Euclidean cluster + convex-hull pipeline on the raw no-ground cloud. This node collapses that duplicated preprocessing into one single-pass rasterizer and gives the last-resort path logical independence from the perception modeling stack (it subscribes only to the no-ground cloud and the static sensor→`base_link` extrinsic — never to objects, tracking, occupancy, map, trajectory, or the localization pose).

## Algorithm

`ObstacleGridExtractor::extract()` is a plain `sensor_msgs/PointCloud2` → `grid_map_msgs/GridMap` converter: it rasterizes the cloud in the cloud's **own** frame (the ROI is interpreted there) and the returned grid inherits the input header — frame and stamp — verbatim. Transforming the cloud into the frame the ROI was configured for is the caller's job; the node does it with TF and drops the cloud outright when that lookup fails.

For each input cloud (already in / transformed to `base_link`):

1. Drop non-finite points, then points outside `[crop.z_min, crop.z_max]`.
2. Compute the single grid cell index of each remaining point (`grid_map::getIndex`).
3. Update that cell's `max_height` (max z), `min_height` (min z), `point_count` (+1), and — for points at or below `overhead_split` — `low_max_height` (max in-band z).

Points are read straight out of the `PointCloud2` with `sensor_msgs::PointCloud2ConstIterator`, so any layout carrying `float32` `x`/`y`/`z` works and no intermediate point-cloud copy is made. A cloud with no points — including a bare one carrying no field descriptors — yields the heartbeat grid described below.

No KdTree, no Euclidean clustering, no convex hull. Empty cells stay `NaN`. An empty cloud yields an all-`NaN` grid with a fresh stamp — the "alive, nothing detected" heartbeat. On a TF failure nothing is published at all, so consumers must treat a stale grid stamp as "unavailable" (fail-safe), never as "clear". Each consumer re-applies its own per-cell size/height gate downstream, so no central tuning is imposed.

Cost is `O(N + C)` over `N` input points and `C` grid cells. At the ROI the consumers require, the **`C` term dominates**: the per-frame clear of the four layers and the conversion to the message are paid whether or not any point arrives, so a near-empty cloud costs about as much as a dense one (measured in [`benchmark/README.md`](benchmark/README.md)). Enlarging the ROI is therefore not free, and the ROI defaults are set from the consumers' declared ranges rather than from the sensor's full reach.

`low_max_height` exists because a per-cell `(min_height, max_height)` pair cannot distinguish a wall (continuous mass from the ground up) from an overhead structure sharing its cell with ground returns (a gantry above ground residue) — both give the same signature. Consumers that brake on cell height gate on `low_max_height` (the tallest return at or below `overhead_split`) so overhead-only content can never qualify a cell, while tall in-band obstacles still do.

## I/O

| Direction | Topic                                                                     | Type                      | QoS                            |
| --------- | ------------------------------------------------------------------------- | ------------------------- | ------------------------------ |
| sub       | `~/input/pointcloud` (`/perception/obstacle_segmentation/pointcloud`)     | `sensor_msgs/PointCloud2` | `SensorDataQoS` `KEEP_LAST(1)` |
| pub       | `~/output/obstacle_grid` (`/sensing/obstacle_segmentation/obstacle_grid`) | `grid_map_msgs/GridMap`   | `RELIABLE` `KEEP_LAST(1)`      |

Layers (`float32`, empty = `NaN`): `max_height`, `min_height`, `point_count`, `low_max_height`.

At the default ROI the grid is 900 × 500 = 450 000 cells, so a message is 450 000 × 4 layers × 4 B = **7.2 MB**, or 72 MB/s at 10 Hz. `KEEP_LAST(1)` bounds the queue: a consumer that cannot keep up drops frames — detectable as a stale stamp — rather than accumulating them.

## Parameters

See [`config/obstacle_grid_extractor.param.yaml`](config/obstacle_grid_extractor.param.yaml) and [`schema/obstacle_grid_extractor.schema.json`](schema/obstacle_grid_extractor.schema.json). Defaults: ROI `x ∈ [-100, +80]`, `y ∈ [-50, +50]` m at 0.2 m cell, z-band `[-2.5, 3.5]` m, `overhead_split` 2.5 m (must be at least every consumer's own z-band top).

The ROI is the union of the consumers' own configured ranges, not the sensor's reach:

- `rear_collision_checker` — crop box `x ∈ [-100, 30]`.
- `intersection_collision_checker` — crop box `x ∈ [-50, 50]`, and it extends the crossing lanelet by 50 m, so the lateral reach is 50 m.
- AEB / `obstacle_collision_checker` — unbounded forward; `+80` m is the RSS stopping distance at their configured `t_response` / `a_ego_min`.

The z-band matches `ground_segmentation` (`margin_min_z` -2.5, `detection_range_z_max` 3.5) so that returns the segmentation deliberately keeps — far obstacles on a vertical curve, sub-ground returns — are not discarded here before any consumer can gate on them.
