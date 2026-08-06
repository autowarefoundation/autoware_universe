# Obstacle grid — ROI sizing, producer cost, message size & coverage

Two things are validated here: that the grid **covers what the consumers need**, and that the cost
of covering it — CPU per frame and bytes per message — is acceptable on the last-resort path.

`test/test_grid_coverage.cpp` is the CI-resident gate (synthetic clouds spread over the whole ROI):
geometry, no-silent-drop, and an informational P99 / max latency recorded as gtest properties, not a
hard bound. Everything below runs the **same `ObstacleGridExtractor`** over recorded real frames.

## Why the ROI is not simply the sensor's reach

A `GridMap` is dense: every cell costs 4 layers × 4 B whether or not anything is in it, and the
per-frame clear of those layers is paid whether or not any point arrives. The no-ground obstacle
cloud is the opposite — sparse, and sparser with distance — so widening the ROI buys progressively
less information per added cell.

Measured over 1200 frames of real full no-ground clouds from two recorded urban drives
(125–132 k points/frame), at 0.2 m resolution. "in-ROI returns" is the share of the frame's points
that land in a cell:

| ROI (`x` × `y`)     | cells   | msg size | occupied cells | occupancy | in-ROI returns |
| ------------------- | ------- | -------- | -------------- | --------- | -------------- |
| `[-10, 50] × ±20`   | 60 000  | 0.96 MB  | 4 000–4 100    | 6.7 %     | 68–78 %        |
| `[-100, 50] × ±50`  | 375 000 | 6.0 MB   | 7 700–8 200    | 2.1 %     | 96–97 %        |
| `[-100, 80] × ±50`  | 450 000 | 7.2 MB   | 8 100–9 000    | 1.8–2.0 % | 98–99 %        |
| `[-100, 50] × ±70`  | 525 000 | 8.4 MB   | 7 900–8 400    | 1.5–1.6 % | 96–98 %        |
| `[-100, 100] × ±50` | 500 000 | 8.0 MB   | 8 200–9 300    | 1.6–1.9 % | 98–99 %        |
| `[-100, 80] × ±60`  | 540 000 | 8.6 MB   | 8 200–9 200    | 1.5–1.7 % | 98–99 %        |
| `[-100, 150] × ±70` | 875 000 | 14.0 MB  | 8 500–9 700    | 1.0–1.1 % | 99 %           |

Reading the two ends: growing the grid **14.6×** (60 k → 875 k cells) raises the occupied-cell count
only **~2.2×**. The last row is also a 14 MB message summarising a cloud that is itself ~2.1 MB on
the wire (132 k points as `PointXYZIRC`), i.e. the "compression" has become a 6× expansion.

The efficient point is `[-100, 80] × ±50`, which is also exactly the union of the consumers'
declared ranges (see the table in the top-level README) — beyond it, added cells are nearly all
empty. Note that widening `y` is a strictly worse trade than extending `x`: `[-100, 50] × ±70` costs
17 % more bytes than `[-100, 80] × ±50` and holds **fewer** occupied cells, because obstacle returns
concentrate along the road rather than out to the sides.

Resolution stays at 0.2 m. It is not a free knob: the consumer-parity evaluation was validated
against the 0.283 m cell diagonal, and 0.3 m would take that to 0.424 m.

## Producer cost

`extract()` at the default `[-100, 80] × ±50` @0.2 m ROI, same real frames:

| recorded drive | points/frame | mean    | p99     | max     |
| -------------- | ------------ | ------- | ------- | ------- |
| urban A        | 131 751      | 2.31 ms | 3.30 ms | 3.58 ms |
| urban B        | 125 208      | 2.28 ms | 2.73 ms | 6.98 ms |
| sparse scene   | 306          | 0.95 ms | 0.99 ms | 4.33 ms |

All inside the 15 ms emergency-path budget, and well below the existing sensing producer baseline it
sits alongside (`scan_ground_filter` + `concatenate`, ~32 ms/frame).

The sparse-scene row is the important one: **306 points per frame still costs ~1 ms**, because
that is the per-cell term, not the per-point term. The same bag costs 0.06 ms at a 60 k-cell ROI and
2.67 ms at 875 k cells — cost tracks grid area almost independently of the cloud. This is why the
ROI is bounded by consumer requirements rather than set to the sensor's reach.

Each configuration is timed in its own process; timing several geometries in one process interleaves
their allocations and inflates the larger ones by up to 2×.

Reading the `PointCloud2` directly (rather than via `pcl::fromROSMsg`) leaves the per-frame cost
unchanged within run-to-run noise and removes the per-callback point-cloud copy the node used to
pay. The node additionally skips `tf2::doTransform` outright when the cloud already declares
`base_link`, which is the production case.

## Crop z-band

The band matches `ground_segmentation`'s own (`margin_min_z` -2.5, `detection_range_z_max` 3.5).
Share of real returns that the previous `[-1.0, 3.0]` band discarded:

| recorded drive | below -1.0 m | above 3.0 m | still outside `[-2.5, 3.5]` |
| -------------- | ------------ | ----------- | --------------------------- |
| urban A        | 0.03 %       | 1.35 %      | 0.00 %                      |
| urban B        | 0.01 %       | 4.19 %      | 1.36 %                      |
| sparse scene   | 8.31 %       | 0.03 %      | 0.29 %                      |

Widening costs nothing in message size — the grid is 2D — and recovers 1.0–2.4 pp of in-ROI returns.
The sparse scene's 8.31 % below -1.0 m are the far-field sub-ground returns on a vertical curve that
the consumer-parity run flagged; the old band discarded them at the producer.

## Coverage vs. the legacy per-consumer pipelines

| quantity                                | result (real sensor data)                              |
| --------------------------------------- | ------------------------------------------------------ |
| coverage (in-ROI point → occupied cell) | ~1.000                                                 |
| grid-vs-raw nearest distance            | conservative on 100 % of frames; error ≤ 0.26 m @0.2 m |

Methodology: each recorded frame is transformed `map -> base_link` with the time-matched
kinematic-state pose and fed to the extractor at the production ROI; `d_legacy` (distance to the
nearest raw point) is compared per frame against the grid query `d_grid` (distance to the nearest
qualifying cell footprint), so "conservative" means `d_grid <= d_legacy` on every frame.
