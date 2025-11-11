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

#include "autoware/pointcloud_preprocessor/diagnostics/hysteresis_state_machine.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
  int32_t radius_idx{};
  int32_t azimuth_idx{};
  int32_t elevation_idx{};

  PolarVoxelIndex() = default;
  PolarVoxelIndex(int32_t radius, int32_t azimuth, int32_t elevation)
  : radius_idx(radius), azimuth_idx(azimuth), elevation_idx(elevation)
  {
  }

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
    // Fowler–Noll–Vo style hash combine for better distribution
    auto hash = std::hash<int32_t>{}(idx.radius_idx);
    hash ^= static_cast<std::size_t>(std::hash<int32_t>{}(idx.azimuth_idx)) + 0x9e3779b9u +
            (static_cast<std::size_t>(hash) << 6u) + (static_cast<std::size_t>(hash) >> 2u);
    hash ^= static_cast<std::size_t>(std::hash<int32_t>{}(idx.elevation_idx)) + 0x9e3779b9u +
            (static_cast<std::size_t>(hash) << 6u) + (static_cast<std::size_t>(hash) >> 2u);
    return hash;
  }
};

// Information about a point's relationship to its voxel
struct PointVoxelInfo
{
  PolarVoxelIndex voxel_idx;
  bool is_primary{false};
  bool meets_intensity_threshold{false};

  PointVoxelInfo() = default;
  explicit PointVoxelInfo(
    const PolarVoxelIndex & voxel_idx, bool is_primary, bool meets_intensity_threshold)
  : voxel_idx(voxel_idx),
    is_primary(is_primary),
    meets_intensity_threshold(meets_intensity_threshold)
  {
  }
};

// Count statistics for points within a voxel
struct VoxelPointCounts
{
  size_t primary_count{0};
  size_t secondary_count{0};
  bool is_in_visibility_range{true};

  // Threshold checks (inclusive)
  [[nodiscard]] bool meets_primary_threshold(int threshold) const
  {
    return primary_count >= static_cast<size_t>(threshold);
  }

  [[nodiscard]] bool meets_secondary_threshold(int threshold) const
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
    double x{};
    double y{};
    double z{};
    CartesianCoordinate() = default;
    CartesianCoordinate(double x, double y, double z) : x(x), y(y), z(z) {}
  };

  struct PolarCoordinate
  {
    double radius{};
    double azimuth{};
    double elevation{};
    PolarCoordinate() = default;
    PolarCoordinate(double radius, double azimuth, double elevation)
    : radius(radius), azimuth(azimuth), elevation(elevation)
    {
    }
  };

protected:
  // Parameter update helper methods
  void update_primary_return_types(const rclcpp::Parameter & param);
  void update_publish_noise_cloud(const rclcpp::Parameter & param);

  // Type aliases to eliminate long type name duplication
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using IndicesPtr = pcl::IndicesPtr;
  using VoxelPointCountMap =
    std::unordered_map<PolarVoxelIndex, VoxelPointCounts, PolarVoxelIndexHash>;
  using VoxelIndexSet = std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash>;
  using PointVoxelInfoVector = std::vector<std::optional<PointVoxelInfo>>;
  using ValidPointsMask = std::vector<bool>;

  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  PointVoxelInfoVector collect_voxel_info(const PointCloud2 & input);
  VoxelPointCountMap count_voxel_points(const PointVoxelInfoVector & point_voxel_info) const;
  VoxelIndexSet determine_valid_voxels_simple(const VoxelPointCountMap & voxel_point_counts) const;
  VoxelIndexSet determine_valid_voxels_with_return_types(
    const VoxelPointCountMap & voxel_point_counts) const;
  VoxelIndexSet determine_valid_voxels(const VoxelPointCountMap & voxel_point_counts) const;
  ValidPointsMask create_valid_points_mask(
    const PointVoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const;
  static void create_filtered_output(
    const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output);
  void publish_noise_cloud(
    const PointCloud2 & input, const ValidPointsMask & valid_points_mask) const;
  void publish_diagnostics(
    const VoxelPointCountMap & voxel_point_counts, const ValidPointsMask & valid_points_mask);

  // Point processing helper methods
  void process_polar_points(const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info);

  void process_cartesian_points(const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info);

  std::optional<PointVoxelInfo> process_polar_point(
    float distance, float azimuth, float elevation, uint8_t intensity, uint8_t return_type) const;

  std::optional<PointVoxelInfo> process_cartesian_point(
    float x, float y, float z, uint8_t intensity, uint8_t return_type) const;

  template <typename Predicate>
  VoxelIndexSet determine_valid_voxels_generic(
    const VoxelPointCountMap & voxel_point_counts, Predicate predicate) const;

  std::optional<PolarCoordinate> extract_polar_from_dae(
    float distance, float azimuth, float elevation) const;

  std::optional<PolarCoordinate> extract_polar_from_xyz(float x, float y, float z) const;

  void update_parameter(const rclcpp::Parameter & param);

  static void setup_output_header(
    PointCloud2 & output, const PointCloud2 & input, size_t valid_count);

  // Coordinate conversion methods
  static PolarCoordinate cartesian_to_polar(const CartesianCoordinate & cartesian);
  PolarVoxelIndex cartesian_to_polar_voxel(const CartesianCoordinate & cartesian) const;
  PolarVoxelIndex polar_to_polar_voxel(const PolarCoordinate & polar) const;

  // Return type and validation methods
  bool is_point_primary(uint8_t return_type) const;
  bool is_valid_polar_point(const PolarCoordinate & polar) const;
  bool meets_intensity_threshold(uint8_t intensity) const;
  static bool has_polar_coordinates(const PointCloud2 & input);

  // Parameter callback and diagnostics
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);
  void on_visibility_check(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_filter_ratio_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Constants to handle circular angles
  const double azimuth_domain_min;
  const double azimuth_domain_max;
  const double elevation_domain_min;
  const double elevation_domain_max;

  // Parameters
  double radial_resolution_m_{};
  double azimuth_resolution_rad_{};
  double elevation_resolution_rad_{};
  int voxel_points_threshold_{};
  double min_radius_m_{};
  double max_radius_m_{};
  double visibility_estimation_max_range_m_{};
  double visibility_estimation_min_azimuth_rad_{};
  double visibility_estimation_max_azimuth_rad_{};
  double visibility_estimation_min_elevation_rad_{};
  double visibility_estimation_max_elevation_rad_{};
  bool use_return_type_classification_{};
  bool enable_secondary_return_filtering_{};
  int secondary_noise_threshold_{};
  int intensity_threshold_{};
  std::vector<int> primary_return_types_;
  bool publish_noise_cloud_{};
  int visibility_estimation_max_secondary_voxel_count_{};
  bool visibility_estimation_only_{};

  // Diagnostic thresholds
  double visibility_error_threshold_{};
  double visibility_warn_threshold_{};
  double filter_ratio_error_threshold_{};
  double filter_ratio_warn_threshold_{};

  // State variables (protected by mutex_)
  std::optional<double> visibility_;
  std::optional<double> filter_ratio_;
  std::mutex mutex_;
  std::shared_ptr<custom_diagnostic_tasks::HysteresisStateMachine> hysteresis_state_machine_;

  // Publishers and diagnostics
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr area_marker_pub_;
  diagnostic_updater::Updater updater_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Diagnostic helper methods
  void calculate_visibility_metric(const VoxelPointCountMap & voxel_point_counts);
  void calculate_filter_ratio_metric(const ValidPointsMask & valid_points_mask);
  void publish_visibility_metric();
  void publish_filter_ratio_metric();
  void publish_area_marker(const std_msgs::msg::Header & input_header);

  // Filter pipeline helper methods
  void validate_filter_inputs(const PointCloud2 & input, const IndicesPtr & indices);
  void create_output(
    const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output);
  void create_empty_output(const PointCloud2 & input, PointCloud2 & output);

  // Point validation helper methods
  bool has_finite_coordinates(const PolarCoordinate & polar) const;
  bool is_within_radius_range(const PolarCoordinate & polar) const;
  bool has_sufficient_radius(const PolarCoordinate & polar) const;

  // Point validation helper methods for mask creation
  bool is_point_valid_for_mask(
    const std::optional<PointVoxelInfo> & optional_info, const VoxelIndexSet & valid_voxels) const;
  bool passes_secondary_return_filter(bool is_primary) const;

private:
  using ParamHandler = std::function<bool(const rclcpp::Parameter &, std::string &)>;
  // Validation helper methods
  void validate_indices(const IndicesPtr & indices);
  void validate_required_fields(const PointCloud2 & input);
  void validate_return_type_field(const PointCloud2 & input);
  void validate_intensity_field(const PointCloud2 & input);
  bool has_field(const PointCloud2 & input, const std::string & field_name);

  // Parameter validation helpers (static, private)
  static bool validate_positive_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_double(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_positive_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_non_negative_int(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_intensity_threshold(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_primary_return_types(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_normalized(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_zero_to_two_pi(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_negative_half_pi_to_half_pi(
    const rclcpp::Parameter & param, std::string & reason);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
