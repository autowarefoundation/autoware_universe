// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>

#include <pcl/search/pcl_search.h>

#include <map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
/** \brief Polar Voxel Outlier Filter Component
 *
 * This filter removes outlier points from LiDAR point clouds using a polar voxel grid approach.
 * Unlike traditional Cartesian voxel grids, this filter uses polar coordinates (radius, azimuth,
 * elevation) which are more suitable for LiDAR sensor characteristics.
 *
 * The filter supports two point cloud formats:
 * - Standard PointXYZ: Computes polar coordinates from Cartesian coordinates
 * - PointXYZIRCAEDT: Uses pre-computed polar coordinates for optimal performance with return type
 * classification
 *
 * ## Filtering Methodology
 * Points are grouped into polar voxels and filtered using a sophisticated two-criteria approach:
 *
 * ### For PointXYZ format:
 * - Simple occupancy threshold: voxels with >= voxel_points_threshold points are kept
 *
 * ### For PointXYZIRCAEDT format (with return type classification):
 * - **Criterion 1**: Primary returns >= voxel_points_threshold
 * - **Criterion 2**: Primary-to-secondary ratio >= secondary_noise_threshold
 * - **Both criteria must be satisfied** for a voxel to be kept
 *
 * ## Return Type Management
 * - **use_return_type_classification**: Enable/disable return type-based filtering
 * - **filter_secondary_returns**: When true, only primary returns are included in output
 * - **primary_return_types**: List of return types classified as primary (typically [1])
 * - **secondary_noise_threshold**: Minimum ratio of primary to secondary returns (default: 1.0)
 *
 * Note: All return types not specified in primary_return_types are considered secondary returns.
 *
 * ## Diagnostics and Debug Features
 * - **Filter Ratio**: Always published - ratio of output points to input points
 * - **Visibility**: Published only for PointXYZIRCAEDT with return type classification enabled
 *   Represents percentage of voxels passing ratio test, computed efficiently during filtering
 * - **Noise Point Cloud**: Debug topic with filtered-out points for analysis
 * - **Dynamic Parameters**: All thresholds can be updated at runtime
 */
class PolarVoxelOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

private:
  // Polar voxel parameters
  double radial_resolution_m_;       // Resolution in radial direction (meters)
  double azimuth_resolution_rad_;    // Resolution in azimuth direction (radians)
  double elevation_resolution_rad_;  // Resolution in elevation direction (radians)
  int voxel_points_threshold_;       // Minimum points required per voxel
  double min_radius_m_;              // Minimum radius to consider
  double max_radius_m_;              // Maximum radius to consider

  // Return type classification parameters
  bool use_return_type_classification_;  // Whether to use return type classification
  bool filter_secondary_returns_;        // Whether to filter secondary returns
  int secondary_noise_threshold_;        // Threshold for primary to secondary return classification
  std::vector<int64_t> primary_return_types_;  // Return types considered as primary returns

  // Diagnostics parameters
  double visibility_error_threshold_;    // Threshold for visibility diagnostics
  double visibility_warn_threshold_;     // Warning threshold for visibility diagnostics
  double filter_ratio_error_threshold_;  // Error threshold for filter ratio diagnostics
  double filter_ratio_warn_threshold_;   // Warning threshold for filter ratio diagnostics

  // Diagnostics members
  double visibility_;    // Current visibility value
  double filter_ratio_;  // Current filter ratio
  diagnostic_updater::Updater updater_{this};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;

  /** \brief Structure to represent a polar voxel index */
  struct PolarVoxelIndex
  {
    int radius_idx;
    int azimuth_idx;
    int elevation_idx;

    bool operator<(const PolarVoxelIndex & other) const
    {
      if (radius_idx != other.radius_idx) return radius_idx < other.radius_idx;
      if (azimuth_idx != other.azimuth_idx) return azimuth_idx < other.azimuth_idx;
      return elevation_idx < other.elevation_idx;
    }
  };

  /** \brief Structure to hold both primary and secondary returns for each voxel */
  struct VoxelPoints
  {
    std::vector<size_t> primary_returns;    // Points classified as primary returns
    std::vector<size_t> secondary_returns;  // Points classified as secondary returns
  };

  /** \brief Filter using regular PointXYZ (compute polar coordinates)
   * Uses simple occupancy threshold filtering - voxels with >= voxel_points_threshold points are
   * kept. No return type classification or ratio-based filtering is applied.
   */
  void filter_point_xyz(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Filter using PointXYZIRCAEDT (use pre-computed polar coordinates)
   * Uses advanced two-criteria filtering when return type classification is enabled:
   * 1. Primary returns >= voxel_points_threshold
   * 2. Primary-to-secondary ratio >= secondary_noise_threshold
   * Both criteria must be satisfied for a voxel to be kept.
   */
  void filter_point_xyzircaedt(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Convert Cartesian coordinates to polar voxel index */
  PolarVoxelIndex cartesian_to_polar_voxel(double x, double y, double z) const;

  /** \brief Convert Cartesian point to polar coordinates */
  static void cartesian_to_polar(
    double x, double y, double z, double & radius, double & azimuth, double & elevation);

  /** \brief Convert pre-computed polar coordinates to voxel index */
  PolarVoxelIndex polar_to_polar_voxel(double radius, double azimuth, double elevation) const;

  /** \brief Validate polar coordinates */
  bool validate_polar_coordinates(double radius, double azimuth, double elevation) const;

  /** \brief Check if return type is in primary returns list */
  bool is_primary_return_type(uint8_t return_type) const;

  /** \brief Classify point by return type for voxel mapping
   * Classifies points into primary or secondary returns based on configured primary return types.
   * Points matching primary_return_types are classified as primary, all others as secondary.
   */
  void classify_point_by_return_type(
    uint8_t return_type, size_t point_idx, PolarVoxelIndex voxel_idx,
    std::map<PolarVoxelIndex, VoxelPoints> & voxel_point_map) const;

  /** \brief Diagnostics callback for visibility validation
   * Visibility represents the percentage of voxels that pass the primary-to-secondary ratio test.
   * Statistics are efficiently collected during main filtering loop (single-pass optimization).
   * Only published when return type classification is enabled for PointXYZIRCAEDT input.
   */
  void onVisibilityChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief Diagnostics callback for filter ratio validation
   * Filter ratio represents the ratio of output points to input points.
   * Always published for both PointXYZ and PointXYZIRCAEDT input formats.
   */
  void onFilterRatioChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolarVoxelOutlierFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
