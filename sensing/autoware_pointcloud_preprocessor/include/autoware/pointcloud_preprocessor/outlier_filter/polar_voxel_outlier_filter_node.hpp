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

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Polar voxel index for 3D polar coordinate space discretization
struct PolarVoxelIndex
{
  int32_t radius_idx, azimuth_idx, elevation_idx;

  bool operator==(const PolarVoxelIndex & other) const
  {
    return radius_idx == other.radius_idx && azimuth_idx == other.azimuth_idx &&
           elevation_idx == other.elevation_idx;
  }
};

// Hash function for PolarVoxelIndex to use in unordered containers
struct PolarVoxelIndexHash
{
  std::size_t operator()(const PolarVoxelIndex & idx) const
  {
    auto h1 = std::hash<int32_t>{}(idx.radius_idx);
    auto h2 = std::hash<int32_t>{}(idx.azimuth_idx);
    auto h3 = std::hash<int32_t>{}(idx.elevation_idx);

    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

// Information about a point's relationship to its voxel
struct PointVoxelInfo
{
  PolarVoxelIndex voxel_idx;
  bool is_primary;

  explicit PointVoxelInfo(const PolarVoxelIndex & idx, bool primary)
  : voxel_idx(idx), is_primary(primary)
  {
  }
};

// Count statistics for points within a voxel
struct VoxelPointCounts
{
  size_t primary_count = 0;
  size_t secondary_count = 0;
  bool is_in_visibility_range = true;

  bool meets_primary_threshold(int threshold) const
  {
    return primary_count >= static_cast<size_t>(threshold);
  }

  bool meets_secondary_threshold(int threshold) const
  {
    return secondary_count <= static_cast<size_t>(threshold);
  }
};

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

private:
  // Type aliases to eliminate long type name duplication
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using IndicesPtr = pcl::IndicesPtr;
  using VoxelPointCountMap =
    std::unordered_map<PolarVoxelIndex, VoxelPointCounts, PolarVoxelIndexHash>;
  using VoxelIndexSet = std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash>;
  using PointVoxelInfoVector = std::vector<std::optional<PointVoxelInfo>>;
  using ValidPointsMask = std::vector<bool>;

  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  PointVoxelInfoVector collect_voxel_info(const PointCloud2ConstPtr & input) const;
  VoxelPointCountMap count_voxels(const PointVoxelInfoVector & point_voxel_info) const;
  VoxelPointCountMap filter_voxels_by_range(const VoxelPointCountMap & voxel_counts) const;
  VoxelIndexSet determine_valid_voxels_simple(const VoxelPointCountMap & voxel_counts) const;
  VoxelIndexSet determine_valid_voxels_with_return_types(
    const VoxelPointCountMap & voxel_counts) const;
  VoxelIndexSet determine_valid_voxels(const VoxelPointCountMap & voxel_counts) const;
  ValidPointsMask create_valid_points_mask(
    const PointVoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const;
  void create_filtered_output(
    const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask,
    PointCloud2 & output) const;
  void publish_noise_cloud(
    const PointCloud2ConstPtr & input, const ValidPointsMask & valid_points_mask) const;
  void publish_diagnostics(
    const VoxelPointCountMap & voxel_counts, const ValidPointsMask & valid_points_mask) const;

  // Helper methods
  void setup_output_header(
    PointCloud2 & output, const PointCloud2ConstPtr & input, size_t valid_count) const;
  sensor_msgs::msg::PointCloud2 create_noise_cloud(
    const PointCloud2ConstPtr & input, size_t noise_count) const;

  // Coordinate conversion methods
  static PolarCoordinate cartesian_to_polar(const CartesianCoordinate & cartesian);
  PolarVoxelIndex cartesian_to_polar_voxel(const CartesianCoordinate & cartesian) const;
  PolarVoxelIndex polar_to_polar_voxel(const PolarCoordinate & polar) const;

  // Return type and validation methods
  bool is_primary_return_type(uint8_t return_type) const;
  bool validate_point_polar(const PolarCoordinate & polar) const;
  bool has_return_type_field(const PointCloud2ConstPtr & input) const;
  bool has_polar_coordinates(const PointCloud2ConstPtr & input) const;

  // Parameter callback and diagnostics
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);
  void on_visibility_check(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_filter_ratio_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double radial_resolution_m_;
  double azimuth_resolution_rad_;
  double elevation_resolution_rad_;
  int voxel_points_threshold_;
  double min_radius_m_;
  double max_radius_m_;
  double visibility_estimation_max_range_m_;
  bool use_return_type_classification_;
  bool filter_secondary_returns_;
  int secondary_noise_threshold_;
  std::vector<int> primary_return_types_;
  bool publish_noise_cloud_;

  // Diagnostic thresholds
  double visibility_error_threshold_;
  double visibility_warn_threshold_;
  double filter_ratio_error_threshold_;
  double filter_ratio_warn_threshold_;

  // State variables
  mutable std::optional<double> visibility_;
  mutable std::optional<double> filter_ratio_;

  // Publishers and diagnostics
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;
  diagnostic_updater::Updater updater_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  mutable std::mutex mutex_;
};

// Forward declaration for implementation-specific structs
struct PointCounts;

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
