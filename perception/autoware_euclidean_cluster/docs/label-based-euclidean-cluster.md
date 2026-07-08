<!-- cspell:ignore pullable -->

# label_based_euclidean_cluster

## Purpose

`label_based_euclidean_cluster` converts a semantically segmented pointcloud into `DetectedObjects`.
It groups points by semantic class, clusters each label bucket independently, and estimates a shape for every resulting cluster.

This node is intended for segmented pointcloud inputs that provide semantic labels as `class_id`, and optionally per-point confidence as `probability`.

## Inputs / Outputs

### Input

| Name    | Type                            | Description                                 |
| ------- | ------------------------------- | ------------------------------------------- |
| `input` | `sensor_msgs::msg::PointCloud2` | segmented pointcloud with `x`, `y`, and `z` |

Optional input fields used by the node:

- `class_id` (`uint8`): semantic class index for each point.
- `probability` (`float32`): semantic confidence for each point.

### Output

| Name     | Type                                             | Description                            |
| -------- | ------------------------------------------------ | -------------------------------------- |
| `output` | `autoware_perception_msgs::msg::DetectedObjects` | detected objects estimated per cluster |

The packaged launch file remaps these by default to the following topics:

- `input` -> `/perception/ptv3/segmented/pointcloud`
- `output` -> `objects`

## Processing Flow

1. Validate that the incoming pointcloud contains `x`, `y`, and `z`.
2. Read the configured `class_names.*` mapping in YAML declaration order and treat that order as the incoming `class_id` index.
3. Drop points whose mapped class is configured as `ignore`.
4. Split the remaining points into buckets keyed by the mapped Autoware object label.
5. Run `VoxelGridBasedEuclideanCluster` independently for each label bucket, using the per-label parameter overrides from `label_cluster_params.*` where configured and the global defaults otherwise.
6. Merge over-segmented clusters across labels that belong to the same confusable label group (`confusable_label_groups.*`).
7. Compute the average semantic probability for each cluster from the points that ended up in that cluster. This uses the source-point indices returned by the clustering backend for the per-label cloud, rather than rematching points by coordinate.
8. Drop clusters whose average semantic probability is below `min_probability`.
9. Estimate a shape and pose for each remaining cluster with `ShapeEstimator`.
10. If shape estimation does not produce a usable shape, fall back to an axis-aligned bounding shape computed from the clustered points.
11. Publish one `DetectedObject` per cluster.

## Label Mapping

`class_names.<original_class_name>` maps each incoming semantic class to an Autoware object class.

- YAML declaration order defines the `class_id` expected in the input pointcloud.
- Entries mapped to `ignore` are skipped before clustering.
- Unsupported mapped labels are ignored with a warning.

Supported mapped labels are:

- `unknown`
- `car`
- `bus`
- `truck`
- `motorcycle`
- `bicycle`
- `pedestrian`
- `animal`
- `trailer`
- `hazard`
- `ignore`

If no supported non-ignored mapping remains after parsing `class_names.*`, the node throws during startup.

## Shape Estimation Behavior

After clustering, the node converts each cluster into a `DetectedObject`.

- The output classification label is the mapped object label for that bucket.
- The output existence probability is the average of the clustered point probabilities for that object instance.
- `min_probability` is applied to that cluster-average probability, not to individual points before clustering.
- Shape estimation is delegated to `autoware::shape_estimation::ShapeEstimator`.
- `shape_policy=0` (`ALL_POLYGON`) estimates whole shapes as polygon.
- `shape_policy=1` (`LABEL_DEPEND`) estimates shapes with the mapped object label.

If the estimator does not return a usable shape:

- `pedestrian` falls back to `CYLINDER`
- all other labels fall back to `BOUNDING_BOX`

The fallback shape uses the cluster axis-aligned min/max extents, with each dimension clamped to at least `0.1` m.

## Parameters

Parameters are loaded from [config/label_based_euclidean_cluster.param.yaml](../config/label_based_euclidean_cluster.param.yaml).

{{ json_to_markdown("perception/autoware_euclidean_cluster/schema/label_based_euclidean_cluster.schema.json") }}

## Per-Label Clustering Parameter Overrides

The clustering parameters at the top level (`use_height`, `tolerance_m`, `voxel_leaf_size_m`,
`min_points_per_voxel`, `min_points_per_cluster`, `large_cluster_voxel_count_threshold`,
`large_cluster_max_points_per_voxel`, `max_voxels_per_cluster`) define a single global
`VoxelGridBasedEuclideanCluster` that is used for every label bucket by default.

`label_cluster_params.<label>` lets each label run with its own tuned clustering instance instead.
This is useful because the spatial density and size of objects differ by class: pedestrians and
hazards need a tighter tolerance and finer voxels than cars or trucks.

- `<label>` must be one of the supported Autoware object labels (`unknown`, `car`, `bus`, `truck`,
  `trailer`, `motorcycle`, `bicycle`, `pedestrian`, `animal`, `hazard`).
- Each override block accepts the same keys as the global clustering parameters.
- Any key omitted inside an override block falls back to the corresponding global value, so a block
  only needs to list the parameters that differ.
- A label with no override block (or an empty one) keeps using the global default cluster.
- When a label-specific cluster is created, the node logs `Using custom cluster params for label '<label>'` at startup.

Example: give pedestrians a tighter tolerance and finer voxels while leaving everything else on the
global defaults.

```yaml
tolerance_m: 0.65
voxel_leaf_size_m: 0.2
# ... other global defaults ...
label_cluster_params:
  pedestrian:
    tolerance_m: 0.3
    voxel_leaf_size_m: 0.1
    min_points_per_voxel: 1
    min_points_per_cluster: 3
    large_cluster_voxel_count_threshold: 5
    large_cluster_max_points_per_voxel: 30
    max_voxels_per_cluster: 200
```

## Confusable-Label Merge

Because each label bucket is clustered independently, a single physical object whose points carry two
semantically confusable labels (for example a truck and its trailer, where the segmentation model
splits one vehicle across the `truck` and `trailer` classes) is split into separate clusters.
`confusable_label_groups.*` performs an optional post-clustering merge that stitches such pieces back
together.

Each entry under `confusable_label_groups` is a user-named group:

- `labels`: the list of labels (two or more) that may be merged with each other. Only clusters of
  **different** labels within the same group are considered for merging; clusters that share a label
  are never merged here.
- `cross_label_tolerance_m`: the maximum point-to-point XY gap between two clusters for them to be
  joined. The gap is the true minimum distance between the closest points of the two clusters
  (measured in the XY plane, height is ignored).
- `max_merged_size_m` (default `25.0`): an upper bound on the diameter of a merged component. A union
  that would produce a bounding circle larger than this is refused, so a chain of nearby clusters
  cannot collapse a dense scene into one oversized blob.

Notes:

- A label may appear in at most one group. If it is listed in several, the first group wins and a
  warning is logged.
- Labels not listed in any group are passed through unchanged.

Example: merge `truck` and `trailer` fragments that are within 0.5 m of each other, capping the
merged object at a 25 m diameter.

```yaml
confusable_label_groups:
  truck_trailer:
    labels: ["truck", "trailer"]
    cross_label_tolerance_m: 0.5
    max_merged_size_m: 25.0
```

## Default Configuration Notes

The default parameter file keeps common road users and filters out map/background classes.

- `car`, `bus`, `truck`, `motorcycle`, `bicycle`, and `pedestrian` are preserved directly.
- `tractor_unit` and `semi_trailer` are mapped to `trailer`.
- several small or ambiguous classes such as `train`, `pushable_pullable`, `traffic_cone`, and `debris` are mapped to `unknown`.
- ground and background classes such as `drivable_surface`, `vegetation`, and `other_stuff` are mapped to `ignore`.

## Assumptions / Known Limits

- The node assumes the incoming pointcloud already represents semantically segmented points.
- `class_id` is interpreted only by order in `class_names.*`; changing YAML order changes the expected semantic index mapping.
- When the input has no `class_id` field, all points are clustered together as `UNKNOWN`.
- When the input has no `probability` field, every point is treated as confidence `1.0`.
- Clustering is spatial only; there is no temporal association or tracking.

## Intended Usage

Use this node when a segmentation model already separates obstacle classes in the pointcloud and the next step is to produce object-level `DetectedObjects` without mixing points from different semantic classes into the same cluster.

## Future Extensions / Follow-up Works

The following items came up during PR review as acceptable short-term trade-offs, but they remain good candidates for follow-up work.

- Load semantic label order from the segmentation model artifact, such as `label.txt`, and keep the ROS parameter file focused on label selection and Autoware label remapping instead of treating YAML declaration order as the source of truth.
- Improve uncertainty-aware clustering so point-level probabilities can be propagated with richer statistics than a simple per-cluster average, such as entropy-aware aggregation or quantile-based gating.
- Improve large-cluster downsampling in the internal voxel-grid-based cluster with deterministic voxel-wise, spatially uniform, or edge-preserving sampling so shape estimation keeps outline points more reliably.
