# autoware_euclidean_cluster

## Purpose

autoware_euclidean_cluster is a package for clustering points into smaller parts to classify objects.

This package has two clustering methods: `euclidean_cluster` and `voxel_grid_based_euclidean_cluster`.

## Inner-workings / Algorithms

### euclidean_cluster

`pcl::EuclideanClusterExtraction` is applied to points. See [official document](https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html) for details.

### voxel_grid_based_euclidean_cluster

1. A centroid in each voxel is calculated by `pcl::VoxelGrid`.
2. The centroids are clustered by `pcl::EuclideanClusterExtraction`.
3. The input points are clustered based on the clustered centroids.

## Inputs / Outputs

### Input

| Name    | Type                            | Description      |
| ------- | ------------------------------- | ---------------- |
| `input` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name             | Type                                                     | Description                                  |
| ---------------- | -------------------------------------------------------- | -------------------------------------------- |
| `output`         | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | cluster pointcloud                           |
| `debug/clusters` | `sensor_msgs::msg::PointCloud2`                          | colored cluster pointcloud for visualization |

## Parameters

### Core Parameters

#### euclidean_cluster

{{ json_to_markdown("perception/autoware_euclidean_cluster/schema/euclidean_cluster.schema.json") }}

#### voxel_grid_based_euclidean_cluster

{{ json_to_markdown("perception/autoware_euclidean_cluster/schema/voxel_grid_based_euclidean_cluster.schema.json") }}

#### label_based_euclidean_cluster

| Name                                | Type   | Description                                                                                   |
| ----------------------------------- | ------ | --------------------------------------------------------------------------------------------- |
| `use_height`                        | bool   | use point.z for clustering                                                                    |
| `min_cluster_size`                  | int    | minimum number of points required to keep a cluster                                           |
| `max_cluster_size`                  | int    | maximum number of points allowed in a cluster                                                 |
| `tolerance`                         | float  | Euclidean clustering tolerance                                                                |
| `min_probability`                   | float  | minimum point probability to keep a point when the input has a `probability` field            |
| `class_names.<original_class_name>` | string | mapped label keyed by original class name; YAML declaration order is used as input `class_id` |
| `use_shape_estimation_corrector`    | bool   | pass clusters through the standard shape estimation corrector                                 |
| `use_shape_estimation_filter`       | bool   | pass estimated boxes through the standard shape estimation filter                             |
| `use_boost_bbox_optimizer`          | bool   | use the boost optimizer in the shape estimation backend                                       |

## Assumptions / Known limits

<!-- Write assumptions and limitations of your implementation.

Example:
  This algorithm assumes obstacles are not moving, so if they rapidly move after the vehicle started to avoid them, it might collide with them.
  Also, this algorithm doesn't care about blind spots. In general, since too close obstacles aren't visible due to the sensing performance limit, please take enough margin to obstacles.
-->

## (Optional) Error detection and handling

<!-- Write how to detect errors and how to recover from them.

Example:
  This package can handle up to 20 obstacles. If more obstacles found, this node will give up and raise diagnostic errors.
-->

## (Optional) Performance characterization

<!-- Write performance information like complexity. If it wouldn't be the bottleneck, not necessary.

Example:
  ### Complexity

  This algorithm is O(N).

  ### Processing time

  ...
-->

## (Optional) References/External links

<!-- Write links you referred to when you implemented.

Example:
  [1] {link_to_a_thesis}
  [2] {link_to_an_issue}
-->

## (Optional) Future extensions / Unimplemented parts

The `use_height` option of `voxel_grid_based_euclidean_cluster` isn't implemented yet.

`label_based_euclidean_cluster` reads `class_names.<original_class_name>` in YAML declaration order and uses that order as `class_id`. Classes mapped to `car`, `bus`, `truck`, `motorcycle`, `bicycle`, or `pedestrian` are kept, and classes mapped to `ignore` are skipped.

If the input pointcloud has no `probability` field, the node skips probability filtering and treats each point as probability `1.0`.

If the input pointcloud has no `class_id` field, the node clusters all points together as `UNKNOWN` and runs shape estimation with the `UNKNOWN` label.
