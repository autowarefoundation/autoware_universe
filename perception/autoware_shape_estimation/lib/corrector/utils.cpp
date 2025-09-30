// Copyright 2021 TierIV. All rights reserved.
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

#define EIGEN_MPL2_ONLY

#include "autoware/shape_estimation/corrector/utils.hpp"

#include "autoware_utils/geometry/geometry.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <vector>

namespace autoware::shape_estimation
{

namespace corrector_utils
{
bool correctWithDefaultValue(
  const CorrectionBBParameters & param, autoware_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Pose & pose)
{
  Eigen::Translation<double, 2> trans =
    Eigen::Translation<double, 2>(pose.position.x, pose.position.y);
  Eigen::Rotation2Dd rotate(tf2::getYaw(pose.orientation));
  Eigen::Affine2d affine_mat;
  affine_mat = trans * rotate.toRotationMatrix();

  /*
   *         ^ x
   *         |
   *        (0)
   * y       |
   * <--(1)--|--(3)--
   *         |
   *        (2)
   *         |
   */
  std::vector<Eigen::Vector2d> v_point;
  v_point.push_back(Eigen::Vector2d(shape.dimensions.x / 2.0, 0.0));
  v_point.push_back(Eigen::Vector2d(0.0, shape.dimensions.y / 2.0));
  v_point.push_back(Eigen::Vector2d(-shape.dimensions.x / 2.0, 0.0));
  v_point.push_back(Eigen::Vector2d(0.0, -shape.dimensions.y / 2.0));

 // Calculate all distances and find indices in one pass
  std::vector<std::pair<double, size_t>> point_distances;
  for (size_t i = 0; i < v_point.size(); ++i) {
    point_distances.emplace_back((affine_mat * v_point.at(i)).norm(), i);
  }

  // Partial sort to get top 3
  std::partial_sort(point_distances.begin(), point_distances.begin() + 3,
                    point_distances.end(), std::greater<>());

  size_t first_most_distant_index = point_distances[0].second;
  size_t second_most_distant_index = point_distances[1].second;
  size_t third_most_distant_index = point_distances[2].second;

  // rule based correction
  Eigen::Vector2d correction_vector = Eigen::Vector2d::Zero();
  // Helper function to apply correction vector
  auto applyCorrectionVector = [](Eigen::Vector2d& correction_vector, double default_size) {
    if (correction_vector.x() == 0.0) {
      correction_vector.y() =
        std::max(std::abs(correction_vector.y()), default_size / 2.0) *
          (correction_vector.y() < 0.0 ? -1.0 : 1.0) -
        correction_vector.y();
    } else if (correction_vector.y() == 0.0) {
      correction_vector.x() =
        std::max(std::abs(correction_vector.x()), default_size / 2.0) *
          (correction_vector.x() < 0.0 ? -1.0 : 1.0) -
        correction_vector.x();
    }
  };

  // Check if first and second most distant points are on opposite edges (0,2 pair or 1,3 pair)
  bool are_opposite_edges = (static_cast<int>(std::abs(
    static_cast<int>(first_most_distant_index) - static_cast<int>(second_most_distant_index))) % 2 == 0);
  // Calculate distances for clearer condition checking
  double first_point_distance = (v_point.at(first_most_distant_index) * 2.0).norm();
  double second_point_distance = (v_point.at(second_most_distant_index) * 2.0).norm();
  double third_point_distance = (v_point.at(third_most_distant_index) * 2.0).norm();
  // Check if points are within parameter ranges
  bool first_in_width_range = (first_point_distance < param.max_width);
  bool first_in_length_range = (param.min_length < first_point_distance && first_point_distance < param.max_length);
  bool second_in_width_range = (second_point_distance < param.max_width);
  bool second_in_length_range = (param.min_length < second_point_distance && second_point_distance < param.max_length);
  bool third_in_width_range = (third_point_distance < param.max_width);
  bool third_in_length_range = (third_point_distance < param.max_length);


  // 1,3 pair or 0,2 pair is most far index
  if (are_opposite_edges) {
    // Case 1: First point fits width range, third point fits length range
    if (first_in_width_range && third_in_length_range) {
      correction_vector = v_point.at(third_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_length);
    }
    // Case 2: First point fits length range, third point fits width range  
    else if (first_in_length_range && third_in_width_range) {
      correction_vector = v_point.at(third_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_width);
    }
    else {
      return false;
    }
  }
  // Case: Adjacent edges (different scenarios based on which points fit ranges)
  else {
    // Case 3: Both first and second points are within width range
    if (first_in_width_range && second_in_width_range) {
      correction_vector = v_point.at(first_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_length);
    }
    // Case 4: Only first point is within width range
    else if (first_in_width_range) {
      correction_vector = v_point.at(second_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_length);
    }
    // Case 5: Only second point is within width range
    else if (second_in_width_range) {
      correction_vector = v_point.at(first_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_length);
    }
    // Case 6: First point is within length range, second point fits width constraint
    else if (first_in_length_range && (second_point_distance < param.max_width)) {
      correction_vector = v_point.at(second_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_width);
    }
    // Case 7: Second point is within length range, first point fits width constraint
    else if (second_in_length_range && (first_point_distance < param.max_width)) {
      correction_vector = v_point.at(first_most_distant_index);
      applyCorrectionVector(correction_vector, param.default_width);
    }
    else {
      return false;
    }
  }

  shape.dimensions.x += std::abs(correction_vector.x()) * 2.0;
  shape.dimensions.y += std::abs(correction_vector.y()) * 2.0;
  pose.position.x += (rotate.toRotationMatrix() * correction_vector).x();
  pose.position.y += (rotate.toRotationMatrix() * correction_vector).y();

  // correct to set long length is x, short length is y
  if (shape.dimensions.x < shape.dimensions.y) {
    geometry_msgs::msg::Vector3 rpy = autoware_utils::get_rpy(pose.orientation);
    rpy.z = rpy.z + M_PI_2;
    pose.orientation = autoware_utils::create_quaternion_from_rpy(rpy.x, rpy.y, rpy.z);
    double temp = shape.dimensions.x;
    shape.dimensions.x = shape.dimensions.y;
    shape.dimensions.y = temp;
  }

  return true;
}

bool correctWithReferenceYaw(
  const CorrectionBBParameters & param, autoware_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Pose & pose)
{
  // TODO(Taichi Higashide): refactor following code
  /*
    c1 is nearest point and other points are arranged like below
    c is center of bounding box
    width
    4---2
    |   |
    | c |length
    |   |
    3---1
   */

  Eigen::Vector3d c1, c2, c3, c4;

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose, base2obj_transform);

  std::vector<Eigen::Vector3d> v_point;
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(-shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));

  c1 = *(std::min_element(v_point.begin(), v_point.end(), [](const auto & a, const auto & b) {
    return a.norm() < b.norm();
  }));

  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_c1 = base2obj_transform.inverse() * c1;
  Eigen::Vector3d radiation_vec = c - local_c1;

  double ex = radiation_vec.x();
  double ey = radiation_vec.y();
  Eigen::Vector3d e1 = (Eigen::Vector3d(0, -ey, 0) - local_c1).normalized();
  Eigen::Vector3d e2 = (Eigen::Vector3d(-ex, 0, 0) - local_c1).normalized();
  double length = 0;
  if (param.min_length < shape.dimensions.x && shape.dimensions.x < param.max_length) {
    length = shape.dimensions.x;
  } else {
    length = param.default_length;
  }
  double width = 0;
  if (param.min_width < shape.dimensions.y && shape.dimensions.y < param.max_width) {
    width = shape.dimensions.y;
  } else {
    width = param.default_width;
  }

  c2 = c1 + base2obj_transform.rotation() * (e1 * length);
  c3 = c1 + base2obj_transform.rotation() * (e2 * width);
  c4 = c1 + (c2 - c1) + (c3 - c1);

  shape.dimensions.x = (c2 - c1).norm();
  shape.dimensions.y = (c3 - c1).norm();
  Eigen::Vector3d new_centroid = c1 + ((c4 - c1) * 0.5);
  pose.position.x = new_centroid.x();
  pose.position.y = new_centroid.y();
  pose.position.z = new_centroid.z();

  return true;
}

bool correctWithReferenceYawAndShapeSize(
  const ReferenceShapeSizeInfo & ref_shape_size_info, autoware_perception_msgs::msg::Shape & shape,
  geometry_msgs::msg::Pose & pose)
{
  /*
  c1 is nearest point and other points are arranged like below
  c is center of bounding box
         width
         4---2
         |   |
  length | c | → ey
         |   |
         3---1
           ↓
           ex
 */

  Eigen::Vector3d c1;

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose, base2obj_transform);

  std::vector<Eigen::Vector3d> v_point;
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(-shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform * Eigen::Vector3d(shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));

  c1 = *(std::min_element(v_point.begin(), v_point.end(), [](const auto & a, const auto & b) {
    return a.norm() < b.norm();
  }));

  Eigen::Vector3d local_c1 = base2obj_transform.inverse() * c1;
  Eigen::Vector3d ex = (Eigen::Vector3d(local_c1.x(), 0, 0)).normalized();
  Eigen::Vector3d ey = (Eigen::Vector3d(0, local_c1.y(), 0)).normalized();

  double length;
  if (
    ref_shape_size_info.mode == ReferenceShapeSizeInfo::Mode::Min &&
    ref_shape_size_info.shape.dimensions.x < shape.dimensions.x) {
    length = shape.dimensions.x;
  } else {
    length = ref_shape_size_info.shape.dimensions.x;
  }

  double width;
  if (
    ref_shape_size_info.mode == ReferenceShapeSizeInfo::Mode::Min &&
    ref_shape_size_info.shape.dimensions.y < shape.dimensions.y) {
    width = shape.dimensions.y;
  } else {
    width = ref_shape_size_info.shape.dimensions.y;
  }

  shape.dimensions.x = length;
  shape.dimensions.y = width;

  Eigen::Vector3d new_centroid =
    c1 - base2obj_transform.rotation() * (ex * length * 0.5 + ey * width * 0.5);
  pose.position.x = new_centroid.x();
  pose.position.y = new_centroid.y();
  pose.position.z = new_centroid.z();
  return true;
}

// use the reference object to correct the initial bounding box
bool correctWithReferenceShapeAndPose(
  const ReferenceShapeSizeInfo & ref_shape_size_info, const geometry_msgs::msg::Pose & ref_pose,
  autoware_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  /*
  c1 is farthest point from ref_pose and other points are arranged like below
  c is center of bounding box
         width
         4---2
         |   |
  length | c | → ey
         |   |
         3---1
           ↓
           ex
 */

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose, base2obj_transform);

  Eigen::Vector3d local_c1;
  Eigen::Vector3d ref_center = Eigen::Vector3d(ref_pose.position.x, ref_pose.position.y, 0.0);
  // local points
  std::vector<Eigen::Vector3d> v_points;
  v_points.push_back(Eigen::Vector3d(shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_points.push_back(Eigen::Vector3d(-shape.dimensions.x * 0.5, shape.dimensions.y * 0.5, 0.0));
  v_points.push_back(Eigen::Vector3d(shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));
  v_points.push_back(Eigen::Vector3d(-shape.dimensions.x * 0.5, -shape.dimensions.y * 0.5, 0.0));

  double max_dist = -1.0;
  // search the most distant index (c1) from the reference object's center
  for (std::size_t i = 0; i < v_points.size(); ++i) {
    const double tmp_dist = ((base2obj_transform * v_points[i]) - ref_center).squaredNorm();
    if (tmp_dist > max_dist) {
      local_c1 = v_points[i];
      max_dist = tmp_dist;
    }
  }

  Eigen::Vector3d ex = (Eigen::Vector3d(local_c1.x() / std::abs(local_c1.x()), 0, 0));
  Eigen::Vector3d ey = (Eigen::Vector3d(0, local_c1.y() / std::abs(local_c1.y()), 0));

  double length;
  if (
    ref_shape_size_info.mode == ReferenceShapeSizeInfo::Mode::Min &&
    ref_shape_size_info.shape.dimensions.x < shape.dimensions.x) {
    length = shape.dimensions.x;
  } else {
    length = ref_shape_size_info.shape.dimensions.x;
  }

  double width;
  if (
    ref_shape_size_info.mode == ReferenceShapeSizeInfo::Mode::Min &&
    ref_shape_size_info.shape.dimensions.y < shape.dimensions.y) {
    width = shape.dimensions.y;
  } else {
    width = ref_shape_size_info.shape.dimensions.y;
  }

  shape.dimensions.x = length;
  shape.dimensions.y = width;

  // compute a new center with correction vector
  Eigen::Vector3d new_centroid =
    (base2obj_transform * local_c1) -
    (base2obj_transform.rotation() * (ex * length * 0.5 + ey * width * 0.5));
  pose.position.x = new_centroid.x();
  pose.position.y = new_centroid.y();
  pose.position.z = new_centroid.z();

  return true;
}
}  // namespace corrector_utils

}  // namespace autoware::shape_estimation
