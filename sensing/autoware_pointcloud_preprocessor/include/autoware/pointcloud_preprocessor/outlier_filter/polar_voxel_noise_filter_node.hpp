// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_NOISE_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_NOISE_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
  uint8_t intensity{0U};

  PointVoxelInfo() = default;
  PointVoxelInfo(const PolarVoxelIndex & voxel_idx, bool is_primary, uint8_t intensity)
  : voxel_idx(voxel_idx), is_primary(is_primary), intensity(intensity)
  {
  }
};

// Aggregated statistics for points within a voxel
struct VoxelStats
{
  int point_count{0};
  float intensity_sum{0.0f};
  float intensity_avg{0.0f};
  int secondary_return_count{0};

  [[nodiscard]] bool meets_min_points(int threshold) const { return point_count >= threshold; }

  [[nodiscard]] bool meets_max_intensity_avg(int threshold) const
  {
    return intensity_avg <= static_cast<float>(threshold);
  }

  [[nodiscard]] bool meets_max_secondary_returns(int threshold) const
  {
    return secondary_return_count <= threshold;
  }
  [[nodiscard]] bool meets_noise_condition(
    int min_points, float avg_intensity_threshold, int max_secondary_returns) const
  {
    return (point_count <= min_points && intensity_avg <= avg_intensity_threshold) ||
           (secondary_return_count >= max_secondary_returns &&
            intensity_avg <= avg_intensity_threshold);
  }
  [[nodiscard]] bool meets_noise_simple_condition(
    int min_points, float avg_intensity_threshold) const
  {
    return (point_count <= min_points && intensity_avg <= avg_intensity_threshold);
  }
};

class PolarVoxelNoiseFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
public:
  explicit PolarVoxelNoiseFilterComponent(const rclcpp::NodeOptions & options);

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
  enum class InputPointCloudFormat { PointXYZIRC, PointXYZIRCAEDT };

  // Parameter update helper methods
  void update_primary_return_types(const rclcpp::Parameter & param);
  void update_publish_noise_cloud(const rclcpp::Parameter & param);

  // Type aliases to eliminate long type name duplication
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using IndicesPtr = pcl::IndicesPtr;
  using VoxelStatsMap = std::unordered_map<PolarVoxelIndex, VoxelStats, PolarVoxelIndexHash>;
  using VoxelIndexSet = std::unordered_set<PolarVoxelIndex, PolarVoxelIndexHash>;
  using PointVoxelInfoVector = std::vector<std::optional<PointVoxelInfo>>;
  using ValidPointsMask = std::vector<bool>;

  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  PointVoxelInfoVector collect_voxel_info(const PointCloud2 & input);
  VoxelStatsMap analyze_voxel_stats(const PointVoxelInfoVector & point_voxel_info) const;
  VoxelIndexSet determine_valid_voxels_simple(const VoxelStatsMap & voxel_stats_map) const;
  VoxelIndexSet determine_valid_voxels_with_return_types(
    const VoxelStatsMap & voxel_stats_map) const;
  VoxelIndexSet determine_valid_voxels(const VoxelStatsMap & voxel_stats_map) const;
  ValidPointsMask create_valid_points_mask(
    const PointVoxelInfoVector & point_voxel_info, const VoxelIndexSet & valid_voxels) const;
  static void create_filtered_output(
    const PointCloud2 & input, const ValidPointsMask & valid_points_mask, PointCloud2 & output);
  void publish_noise_cloud(
    const PointCloud2 & input, const ValidPointsMask & valid_points_mask) const;

  // Point processing helper methods
  void process_polar_points(const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info);

  void process_cartesian_points(const PointCloud2 & input, PointVoxelInfoVector & point_voxel_info);

  std::optional<PointVoxelInfo> process_polar_point(
    float distance, float azimuth, float elevation, uint8_t intensity, uint8_t return_type) const;

  std::optional<PointVoxelInfo> process_cartesian_point(
    float x, float y, float z, uint8_t intensity, uint8_t return_type) const;

  template <typename Predicate>
  VoxelIndexSet determine_valid_voxels_generic(
    const VoxelStatsMap & voxel_stats_map, Predicate predicate) const;

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
  static bool has_polar_coordinates(const PointCloud2 & input);

  // Parameter callback and diagnostics
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

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
  bool use_return_type_classification_{};
  bool enable_secondary_return_filtering_{};
  int secondary_noise_threshold_{};
  double avg_intensity_threshold_{};
  std::vector<int> primary_return_types_;
  bool publish_noise_cloud_{};
  std::mutex mutex_;
  std::once_flag input_format_once_flag_;
  std::optional<InputPointCloudFormat> input_format_;

  // Publishers and diagnostics
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

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
  static bool validate_primary_return_types(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_normalized(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_zero_to_two_pi(const rclcpp::Parameter & param, std::string & reason);
  static bool validate_negative_half_pi_to_half_pi(
    const rclcpp::Parameter & param, std::string & reason);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_NOISE_FILTER_NODE_HPP_
