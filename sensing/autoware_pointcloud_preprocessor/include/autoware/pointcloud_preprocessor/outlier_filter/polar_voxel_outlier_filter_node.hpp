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
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
// Forward declarations for structures used in extracted methods
struct VoxelInfo;
struct VoxelCounts;
struct PointProcessingResult;
struct PointCounts;

class PolarVoxelOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
public:
  explicit PolarVoxelOutlierFilterComponent(const rclcpp::NodeOptions & options);

  // Custom coordinate types for type safety and self-documenting code
  struct CartesianCoordinate
  {
    double x, y, z;
    CartesianCoordinate(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}
  };

  struct PolarCoordinate
  {
    double radius, azimuth, elevation;
    PolarCoordinate(double radius_val, double azimuth_val, double elevation_val)
    : radius(radius_val), azimuth(azimuth_val), elevation(elevation_val)
    {
    }
  };

  struct PolarVoxelIndex
  {
    int32_t radius_idx, azimuth_idx, elevation_idx;

    bool operator==(const PolarVoxelIndex & other) const
    {
      return radius_idx == other.radius_idx && azimuth_idx == other.azimuth_idx &&
             elevation_idx == other.elevation_idx;
    }
  };

  struct PolarVoxelIndexHash
  {
    std::size_t operator()(const PolarVoxelIndex & idx) const
    {
      return std::hash<int32_t>{}(idx.radius_idx) ^ (std::hash<int32_t>{}(idx.azimuth_idx) << 1) ^
             (std::hash<int32_t>{}(idx.elevation_idx) << 2);
    }
  };

private:
  // Type aliases to eliminate long type name duplication
  using VoxelCountMap = std::unordered_map<PolarVoxelIndex, VoxelCounts, PolarVoxelIndexHash>;
  using VoxelIndexSet = std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash>;
  using VoxelInfoVector = std::vector<std::optional<VoxelInfo>>;
  using ValidPointsMask = std::vector<bool>;

  // Processing context to eliminate parameter list duplication
  struct ProcessingContext
  {
    const PointCloud2ConstPtr & input;
    const ValidPointsMask & valid_points_mask;
    const VoxelCountMap & voxel_counts;
    const VoxelIndexSet & valid_voxels;
  };

  // Main filter method
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  // Main filtering functions
  void filter_point_xyz(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);
  void filter_point_xyzircaedt(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  // Phase 1: Data collection methods
  VoxelInfoVector collect_voxel_info_xyz(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input) const;
  VoxelInfoVector collect_voxel_info_xyzircaedt(const PointCloud2ConstPtr & input) const;

  // Phase 2: Voxel validation methods
  VoxelCountMap count_voxels(const VoxelInfoVector & point_voxel_info) const;
  VoxelIndexSet determine_valid_voxels_simple(const VoxelCountMap & voxel_counts) const;
  VoxelIndexSet determine_valid_voxels_with_return_types(
    const VoxelCountMap & voxel_counts, size_t & voxels_passed_secondary_test) const;

  // Phase 3: Point filtering methods
  ValidPointsMask create_valid_points_mask(
    const VoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels,
    bool filter_secondary = false) const;

  // Phase 4: Output creation methods
  void create_filtered_output_xyz(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input,
    const ValidPointsMask & valid_points_mask, PointCloud2 & output) const;

  void create_filtered_output_xyzircaedt(
    const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
    PointCloud2 & output) const;

  // Phase 5: Noise cloud publishing methods
  void publish_noise_cloud_xyz(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_input,
    const ValidPointsMask & valid_points_mask, const VoxelInfoVector & point_voxel_info,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input) const;

  void publish_noise_cloud_xyzircaedt(
    const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask) const;

  // Phase 6: Diagnostic publishing methods
  void publish_diagnostics_simple(const ProcessingContext & context) const;
  void publish_diagnostics_with_return_types(
    const ProcessingContext & context, size_t voxels_passed_secondary_test) const;

  // Common logic extraction methods
  PointProcessingResult create_processing_result(
    const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
    const VoxelCountMap & voxel_counts, const VoxelIndexSet & valid_voxels,
    size_t voxels_passed_secondary_test = 0) const;

  sensor_msgs::msg::PointCloud2 create_noise_cloud(
    const PointCloud2ConstPtr & input, size_t noise_count) const;

  PointCounts calculate_point_counts(const ValidPointsMask & valid_points_mask) const;

  void setup_output_header(
    PointCloud2 & output, const PointCloud2ConstPtr & input, size_t valid_count) const;

  // Helper functions
  static PolarCoordinate cartesian_to_polar(const CartesianCoordinate & cartesian);
  PolarVoxelIndex cartesian_to_polar_voxel(const CartesianCoordinate & cartesian) const;
  PolarVoxelIndex polar_to_polar_voxel(const PolarCoordinate & polar) const;
  bool is_primary_return_type(uint8_t return_type) const;
  bool validate_point_basic(const PolarCoordinate & polar) const;

  void publish_diagnostics(
    const PointProcessingResult & result, bool has_return_type_classification);

  // Parameter callback
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  // Diagnostic functions
  void on_visibility_check(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_filter_ratio_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double radial_resolution_m_;
  double azimuth_resolution_rad_;
  double elevation_resolution_rad_;
  int voxel_points_threshold_;
  double min_radius_m_;
  double max_radius_m_;
  bool use_return_type_classification_;
  bool filter_secondary_returns_;
  int secondary_noise_threshold_;
  std::vector<int> primary_return_types_;
  double visibility_error_threshold_;
  double visibility_warn_threshold_;
  double filter_ratio_error_threshold_;
  double filter_ratio_warn_threshold_;

  // State variables using std::optional for safe initialization
  mutable std::optional<double> visibility_;
  mutable std::optional<double> filter_ratio_;

  // Publishers
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;

  // Diagnostics
  diagnostic_updater::Updater updater_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Thread safety
  mutable std::mutex mutex_;
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
