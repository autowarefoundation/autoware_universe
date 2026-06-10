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
4. If the pointcloud has a `probability` field, drop points with `probability < min_probability`.
5. Split the remaining points into buckets keyed by the mapped Autoware object label.
6. Run `VoxelGridBasedEuclideanCluster` independently for each label bucket.
7. Estimate a shape and pose for each cluster with `ShapeEstimator`.
8. If shape estimation does not produce a usable shape, fall back to an axis-aligned bounding shape computed from the clustered points.
9. Publish one `DetectedObject` per cluster.

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
- The output existence probability is the average of the bucket point probabilities.
- Shape estimation is delegated to `autoware::shape_estimation::ShapeEstimator`.
- `shape_policy=0` (`ALL_POLYGON`) estimates whole shapes as polygon.
- `shape_policy=1` (`LABEL_DEPEND`) estimates shapes with the mapped object label.

If the estimator does not return a usable shape:

- `pedestrian` falls back to `CYLINDER`
- all other labels fall back to `BOUNDING_BOX`

The fallback shape uses the cluster axis-aligned min/max extents, with each dimension clamped to at least `0.1` m.

## Parameters

Parameters are loaded from [config/label_based_euclidean_cluster.param.yaml](../config/label_based_euclidean_cluster.param.yaml).

### Clustering Parameters

| Name                                    | Type  | Description                                                         |
| --------------------------------------- | ----- | ------------------------------------------------------------------- |
| `use_height`                            | bool  | Use the `z` coordinate during clustering.                           |
| `tolerance`                             | float | Euclidean clustering tolerance.                                     |
| `min_cluster_size`                      | int   | Minimum number of points required to keep a cluster.                |
| `max_cluster_size`                      | int   | Maximum number of points allowed in a cluster.                      |
| `voxel_leaf_size`                       | float | Voxel size used by the internal voxel-grid-based cluster.           |
| `min_points_number_per_voxel`           | int   | Minimum number of points required to keep a voxel centroid.         |
| `min_voxel_cluster_size_for_filtering`  | int   | Minimum voxel-cluster size before applying large-cluster filtering. |
| `max_points_per_voxel_in_large_cluster` | int   | Maximum number of points kept per voxel for large clusters.         |
| `max_voxel_cluster_for_output`          | int   | Maximum number of voxel clusters emitted by the internal cluster.   |

### Semantic Filtering Parameters

| Name              | Type   | Description                                                                          |
| ----------------- | ------ | ------------------------------------------------------------------------------------ |
| `min_probability` | float  | Minimum per-point semantic confidence when the input contains a `probability` field. |
| `class_names.*`   | string | Semantic class remapping table. YAML order is treated as input `class_id`.           |

### Shape Estimation Parameters

| Name                             | Type | Description                                                           |
| -------------------------------- | ---- | --------------------------------------------------------------------- |
| `shape_policy`                   | int  | Shape estimation mode: `0` for `ALL_POLYGON`, `1` for `LABEL_DEPEND`. |
| `use_shape_estimation_corrector` | bool | Enable the standard shape estimation corrector.                       |
| `use_shape_estimation_filter`    | bool | Enable the standard shape estimation filter.                          |
| `use_boost_bbox_optimizer`       | bool | Enable the boost-based bounding box optimizer in shape estimation.    |

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
- The current existence probability is averaged per label bucket before clustering, not per individual cluster.
- Clustering is spatial only; there is no temporal association or tracking.

## Intended Usage

Use this node when a segmentation model already separates obstacle classes in the pointcloud and the next step is to produce object-level `DetectedObjects` without mixing points from different semantic classes into the same cluster.

## Future Extensions / Follow-up Works

The following items came up during PR review as acceptable short-term trade-offs, but they remain good candidates for follow-up work.

- Load semantic label order from the segmentation model artifact, such as `label.txt`, and keep the ROS parameter file focused on label selection and Autoware label remapping instead of treating YAML declaration order as the source of truth.
- Preserve source point indices through clustering so `existence_probability` can be computed per output cluster rather than once per semantic bucket.
  - Or uncertainty aware clustering can be applied to propagate point-level probabilities into clusters using entropy field values.
- Improve large-cluster downsampling in the internal voxel-grid-based cluster with deterministic voxel-wise, spatially uniform, or edge-preserving sampling so shape estimation keeps outline points more reliably.
