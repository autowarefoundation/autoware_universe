// Copyright 2024 TIER IV, Inc.
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
class PolarVoxelOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

private:
  // Polar voxel parameters
  double radius_resolution_;       // Resolution in radial direction (meters)
  double azimuth_resolution_;      // Resolution in azimuth direction (radians)
  double elevation_resolution_;    // Resolution in elevation direction (radians)
  int voxel_points_threshold_;     // Minimum points required per voxel
  double min_radius_;              // Minimum radius to consider
  double max_radius_;              // Maximum radius to consider

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

  /** \brief Filter using regular PointXYZ (compute polar coordinates) */
  void filterPointXYZ(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Filter using PointXYZIRCAEDT (use pre-computed polar coordinates) */
  void filterPointXYZIRCAEDT(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Convert Cartesian coordinates to polar voxel index */
  PolarVoxelIndex cartesianToPolarVoxel(double x, double y, double z) const;

  /** \brief Convert Cartesian point to polar coordinates */
  void cartesianToPolar(double x, double y, double z, double & radius, double & azimuth, double & elevation) const;

  /** \brief Convert pre-computed polar coordinates to voxel index */
  PolarVoxelIndex polarToPolarVoxel(double radius, double azimuth, double elevation) const;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolarVoxelOutlierFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__POLAR_VOXEL_OUTLIER_FILTER_NODE_HPP_ 