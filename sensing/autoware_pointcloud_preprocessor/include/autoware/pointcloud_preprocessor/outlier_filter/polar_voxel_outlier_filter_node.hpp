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

#include <pcl/search/pcl_search.h>

#include <map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
/** \brief Polar Voxel Outlier Filter Component
 * 
 * This filter removes outlier points from LiDAR point clouds using a polar voxel grid approach.
 * Unlike traditional Cartesian voxel grids, this filter uses polar coordinates (radius, azimuth, elevation)
 * which are more suitable for LiDAR sensor characteristics.
 * 
 * The filter supports two point cloud formats:
 * - Standard PointXYZ: Computes polar coordinates from Cartesian coordinates
 * - PointXYZIRCAEDT: Uses pre-computed polar coordinates for optimal performance
 * 
 * Points are grouped into polar voxels and filtered based on occupancy thresholds.
 * Voxels with fewer points than the threshold are considered outliers and removed.
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
  double radius_resolution_;       // Resolution in radial direction (meters)
  double azimuth_resolution_;      // Resolution in azimuth direction (radians)
  double elevation_resolution_;    // Resolution in elevation direction (radians)
  int voxel_points_threshold_;     // Minimum points required per voxel
  double min_radius_;              // Minimum radius to consider
  double max_radius_;              // Maximum radius to consider
  
  // Return type classification parameters
  bool use_return_type_classification_;         // Whether to use return type classification
  std::vector<int64_t> primary_return_types_;   // Return types considered as primary returns
  std::vector<int64_t> secondary_return_types_; // Return types considered as secondary returns

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
  struct VoxelPoints {
    std::vector<size_t> primary_returns;    // Points classified as primary returns
    std::vector<size_t> secondary_returns;  // Points classified as secondary returns
  };

  /** \brief Filter using regular PointXYZ (compute polar coordinates) */
  void filter_point_xyz(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Filter using PointXYZIRCAEDT (use pre-computed polar coordinates) */
  void filter_point_xyzircaedt(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Convert Cartesian coordinates to polar voxel index */
  PolarVoxelIndex cartesian_to_polar_voxel(double x, double y, double z) const;

  /** \brief Convert Cartesian point to polar coordinates */
  static void cartesian_to_polar(double x, double y, double z, double & radius, double & azimuth, double & elevation);

  /** \brief Convert pre-computed polar coordinates to voxel index */
  PolarVoxelIndex polar_to_polar_voxel(double radius, double azimuth, double elevation) const;

  /** \brief Validate polar coordinates */
  bool validate_polar_coordinates(double radius, double azimuth, double elevation) const;

  /** \brief Check if return type is in primary returns list */
  bool is_primary_return_type(uint8_t return_type) const;

  /** \brief Check if return type is in secondary returns list */
  bool is_secondary_return_type(uint8_t return_type) const;

  /** \brief Classify point by return type for voxel mapping */
  void classify_point_by_return_type(uint8_t return_type, size_t point_idx, 
                                           PolarVoxelIndex voxel_idx, 
                                           std::map<PolarVoxelIndex, VoxelPoints>& voxel_point_map) const;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolarVoxelOutlierFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_