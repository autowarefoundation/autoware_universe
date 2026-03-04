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

#ifndef AUTOWARE__DETECTED_OBJECT_FEATURE_REMOVER__CONVERT_HPP_
#define AUTOWARE__DETECTED_OBJECT_FEATURE_REMOVER__CONVERT_HPP_

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace autoware::detected_object_feature_remover::convert
{

struct ConvertParams
{
  bool run_convex_hull_conversion;
};

void convertToDetectedObjects(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input,
  autoware_perception_msgs::msg::DetectedObjects & output, const ConvertParams & params);

void pclToConvexHull(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, autoware_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Pose & pose);

}  // namespace autoware::detected_object_feature_remover::convert

#endif  // AUTOWARE__DETECTED_OBJECT_FEATURE_REMOVER__CONVERT_HPP_
