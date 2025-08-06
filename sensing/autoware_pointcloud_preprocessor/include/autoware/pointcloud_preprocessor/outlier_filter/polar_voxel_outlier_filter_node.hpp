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
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Forward declaration for PointProcessingResult
struct PointProcessingResult;

using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using IndicesPtr = pcl::IndicesPtr;

/** \brief Polar Voxel Outlier Filter Component
 * Filters outlier points by grouping points into polar voxels and removing voxels
 * with insufficient point density. Supports both PointXYZ and PointXYZIRCAEDT formats,
 * with advanced return type classification for LiDAR data.
 */
class PolarVoxelOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

private:
  /** \brief Structure to represent Cartesian coordinates */
  struct CartesianCoordinate
  {
    double x;
    double y;
    double z;

    CartesianCoordinate(double x_val, double y_val, double z_val) : x(x_val), y(y_val), z(z_val) {}
  };

  /** \brief Structure to represent polar coordinates */
  struct PolarCoordinate
  {
    double radius;
    double azimuth;
    double elevation;

    PolarCoordinate(double r, double a, double e) : radius(r), azimuth(a), elevation(e) {}
  };

  /** \brief Structure to represent a polar voxel index */
  struct PolarVoxelIndex
  {
    int32_t radius_idx;
    int32_t azimuth_idx;
    int32_t elevation_idx;

    bool operator==(const PolarVoxelIndex & other) const
    {
      return radius_idx == other.radius_idx && azimuth_idx == other.azimuth_idx &&
             elevation_idx == other.elevation_idx;
    }
  };

  /** \brief Hash function for PolarVoxelIndex */
  struct PolarVoxelIndexHash
  {
    std::size_t operator()(const PolarVoxelIndex & idx) const
    {
      std::size_t h1 = std::hash<int32_t>{}(idx.radius_idx);
      std::size_t h2 = std::hash<int32_t>{}(idx.azimuth_idx);
      std::size_t h3 = std::hash<int32_t>{}(idx.elevation_idx);

      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  // Thread safety
  mutable std::mutex mutex_;

  // Filter parameters
  double radial_resolution_m_;
  double azimuth_resolution_rad_;
  double elevation_resolution_rad_;
  int voxel_points_threshold_;
  double min_radius_m_;
  double max_radius_m_;
  bool use_return_type_classification_;
  bool filter_secondary_returns_;
  int secondary_noise_threshold_;
  std::vector<int64_t> primary_return_types_;
  double visibility_error_threshold_;
  double visibility_warn_threshold_;
  double filter_ratio_error_threshold_;
  double filter_ratio_warn_threshold_;

  // Metrics - using std::optional to avoid dummy values
  std::optional<double> visibility_;
  std::optional<double> filter_ratio_;

  // Diagnostics and publishers
  diagnostic_updater::Updater updater_{this};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr ratio_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;

  /** \brief Filter using regular PointXYZ (compute polar coordinates) */
  void filter_point_xyz(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Filter using PointXYZIRCAEDT (use pre-computed polar coordinates) */
  void filter_point_xyzircaedt(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Convert Cartesian coordinates to polar voxel index */
  PolarVoxelIndex cartesian_to_polar_voxel(const CartesianCoordinate & cartesian) const;

  /** \brief Convert Cartesian point to polar coordinates */
  static PolarCoordinate cartesian_to_polar(const CartesianCoordinate & cartesian);

  /** \brief Convert pre-computed polar coordinates to voxel index */
  PolarVoxelIndex polar_to_polar_voxel(const PolarCoordinate & polar) const;

  /** \brief Check if return type is in primary returns list */
  bool is_primary_return_type(uint8_t return_type) const;

  /** \brief Diagnostics callback for visibility validation */
  void on_visibility_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief Diagnostics callback for filter ratio validation */
  void on_filter_ratio_check(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /** \brief Validate basic polar coordinate constraints */
  bool validate_point_basic(const PolarCoordinate & polar) const;

  /** \brief Publish processing diagnostics */
  void publish_diagnostics(
    const PointProcessingResult & result, bool has_return_type_classification);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolarVoxelOutlierFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_
