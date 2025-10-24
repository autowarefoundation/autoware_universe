// Copyright 2020 Tier IV, Inc.
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

/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: cropbox.cpp
 *
 */

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/transform_info.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <pcl/filters/crop_box.h>

#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct CropBox
{
  float min_x;
  float max_x;
  float min_y;
  float max_y;
  float min_z;
  float max_z;
};

/** \brief Return whether the point is inside the crop box.
 *
 * @param point The point to check, in the crop box frame.
 * @param box The crop box to check against.
 * @return Whether the point is inside the crop box.
 */
bool is_point_inside_crop_box(const Eigen::Vector4f & point, const CropBox & box);

/** \brief Return whether the line segment intersects the crop box.
 *
 * @param from_point The point from which the line segment starts, in the crop box frame.
 * @param to_point The point to which the line segment ends, in the crop box frame.
 * @param box The crop box to check against.
 * @return Whether the line segment intersects the crop box.
 */
bool does_line_segment_intersect_crop_box(
  const Eigen::Vector4f & from_point, const Eigen::Vector4f & to_point, const CropBox & box);

class CropBoxFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API
  void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info) override;

  void publish_crop_box_polygon();

private:
  struct CropBoxParam
  {
    CropBox box;
    bool negative{false};
    double processing_time_threshold_sec{0.0};
  } param_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr crop_box_polygon_pub_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);
  void publish_diagnostics(const std::vector<std::shared_ptr<const DiagnosticsBase>> & diagnostics);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit CropBoxFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
