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

#include "autoware/pointcloud_preprocessor/crop_box_filter/crop_box_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/diagnostics/crop_box_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/latency_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/diagnostics/pass_rate_diagnostics.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
bool is_point_inside_crop_box(const Eigen::Vector4f & point, const CropBox & box)
{
  return (point[0] > box.min_x && point[0] < box.max_x) &&
         (point[1] > box.min_y && point[1] < box.max_y) &&
         (point[2] > box.min_z && point[2] < box.max_z);
}

bool does_line_segment_intersect_crop_box(
  const Eigen::Vector4f & from_point, const Eigen::Vector4f & to_point, const CropBox & box)
{
  // The below algorithm is known as the Slab Method.
  // Further reading: https://tavianator.com/2022/ray_box_boundary.html

  Eigen::Vector3f ray_origin = from_point.head<3>();
  Eigen::Vector3f ray_direction = to_point.head<3>() - from_point.head<3>();
  Eigen::Vector3f ray_direction_inv = {
    1.0f / ray_direction.x(), 1.0f / ray_direction.y(), 1.0f / ray_direction.z()};

  Eigen::Vector3f box_min = {box.min_x, box.min_y, box.min_z};
  Eigen::Vector3f box_max = {box.max_x, box.max_y, box.max_z};

  // A line is represented as `l(t) = ray_origin + t * ray_direction`.
  // A line segment is represented as `l(t) = ray_origin + t * ray_direction`, where `t` is in [0,
  // 1], given that `ray_direction = (to_point - from_point)` (not normalized). We start with the
  // full line segment.
  float t_min = 0;  // from_point
  float t_max = 1;  // to_point

  // For each axis, we intersect the line segment with the min and max planes of the box,
  // keeping only the part of the line segment that is within the box.
  for (int axis = 0 /* x */; axis < 3 /* z */; ++axis) {
    float t1 = (box_min[axis] - ray_origin[axis]) * ray_direction_inv[axis];
    float t2 = (box_max[axis] - ray_origin[axis]) * ray_direction_inv[axis];

    t_min = std::max(t_min, std::min(t1, t2));
    t_max = std::min(t_max, std::max(t1, t2));
  }

  // If, after intersecting with all three pairs of planes, the line segment is still valid,
  // then the line segment intersects the box.
  return t_min < t_max;
}

CropBoxFilterComponent::CropBoxFilterComponent(const rclcpp::NodeOptions & options)
: Filter("CropBoxFilter", options)
{
  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    auto & p = param_;
    p.box.min_x = declare_parameter<double>("min_x");
    p.box.min_y = declare_parameter<double>("min_y");
    p.box.min_z = declare_parameter<double>("min_z");
    p.box.max_x = declare_parameter<double>("max_x");
    p.box.max_y = declare_parameter<double>("max_y");
    p.box.max_z = declare_parameter<double>("max_z");
    p.negative = declare_parameter<bool>("negative");
    p.use_ray_intersection = declare_parameter<bool>("use_ray_intersection");
    p.processing_time_threshold_sec = declare_parameter<double>("processing_time_threshold_sec");
    if (tf_input_frame_.empty()) {
      throw std::invalid_argument("Crop box requires non-empty input_frame");
    }
  }

  // Diagnostic
  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, this->get_fully_qualified_name());
  // set additional publishers
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    crop_box_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "~/crop_box_polygon", 10, pub_options);
  }

  // set parameter service callback
  {
    using std::placeholders::_1;
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&CropBoxFilterComponent::param_callback, this, _1));
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void CropBoxFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined above.
void CropBoxFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  if (indices) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Indices are not supported and will be ignored");
  }

  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  size_t output_size = 0;

  int skipped_count = 0;

  Eigen::Vector4f lidar_origin = {0, 0, 0, 1};
  if (transform_info.need_transform) {
    lidar_origin = transform_info.eigen_transform * lidar_origin;
  }

  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector4f point;
    std::memcpy(&point[0], &input->data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &input->data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &input->data[global_offset + z_offset], sizeof(float));
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      skipped_count++;
      continue;
    }

    if (transform_info.need_transform) {
      point = transform_info.eigen_transform * point;
    }

    bool point_is_inside{};
    if (param_.use_ray_intersection) {
      point_is_inside = does_line_segment_intersect_crop_box(lidar_origin, point, param_.box);
    } else {
      point_is_inside = is_point_inside_crop_box(point, param_.box);
    }

    if ((!param_.negative && point_is_inside) || (param_.negative && !point_is_inside)) {
      memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);

      if (transform_info.need_transform) {
        std::memcpy(&output.data[output_size + x_offset], &point[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point[2], sizeof(float));
      }

      output_size += input->point_step;
    }
  }

  if (skipped_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%d points contained NaN values and have been ignored",
      skipped_count);
  }

  output.data.resize(output_size);

  // Note that tf_input_orig_frame_ is the input frame, while tf_input_frame_ is the frame of the
  // crop box
  output.header.frame_id = tf_input_frame_;

  output.height = 1;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  publish_crop_box_polygon();

  const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
  const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const double pipeline_latency_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds((this->get_clock()->now() - input->header.stamp).nanoseconds()))
      .count();

  // Debug output
  if (debug_publisher_) {
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  auto latency_diagnostics = std::make_shared<LatencyDiagnostics>(
    input->header.stamp, processing_time_ms, pipeline_latency_ms,
    param_.processing_time_threshold_sec * 1000.0);
  auto pass_rate_diagnostics = std::make_shared<PassRateDiagnostics>(
    static_cast<int>(input->width * input->height), static_cast<int>(output.width * output.height));
  auto crop_box_diagnostics = std::make_shared<CropBoxDiagnostics>(skipped_count);

  publish_diagnostics({latency_diagnostics, pass_rate_diagnostics, crop_box_diagnostics});
}

void CropBoxFilterComponent::publish_diagnostics(
  const std::vector<std::shared_ptr<const DiagnosticsBase>> & diagnostics)
{
  diagnostics_interface_->clear();

  std::string message;
  int worst_level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  for (const auto & diag : diagnostics) {
    diag->add_to_interface(*diagnostics_interface_);
    if (const auto status = diag->evaluate_status(); status.has_value()) {
      worst_level = std::max(worst_level, status->first);
      if (!message.empty()) {
        message += " / ";
      }
      message += status->second;
    }
  }

  if (message.empty()) {
    message = "CropBoxFilter operating normally";
  }

  diagnostics_interface_->update_level_and_message(static_cast<int8_t>(worst_level), message);
  diagnostics_interface_->publish(this->get_clock()->now());
}

void CropBoxFilterComponent::publish_crop_box_polygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param_.box.max_x;
  const double x2 = param_.box.min_x;
  const double x3 = param_.box.min_x;
  const double x4 = param_.box.max_x;

  const double y1 = param_.box.max_y;
  const double y2 = param_.box.max_y;
  const double y3 = param_.box.min_y;
  const double y4 = param_.box.min_y;

  const double z1 = param_.box.min_z;
  const double z2 = param_.box.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = tf_input_frame_;
  polygon_msg.header.stamp = get_clock()->now();
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  crop_box_polygon_pub_->publish(polygon_msg);
}

rcl_interfaces::msg::SetParametersResult CropBoxFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  CropBoxParam new_param{};

  if (
    get_param(p, "min_x", new_param.box.min_x) && get_param(p, "min_y", new_param.box.min_y) &&
    get_param(p, "min_z", new_param.box.min_z) && get_param(p, "max_x", new_param.box.max_x) &&
    get_param(p, "max_y", new_param.box.max_y) && get_param(p, "max_z", new_param.box.max_z) &&
    get_param(p, "negative", new_param.negative) &&
    get_param(p, "use_ray_intersection", new_param.use_ray_intersection)) {
    if (
      param_.box.min_x != new_param.box.min_x || param_.box.max_x != new_param.box.max_x ||
      param_.box.min_y != new_param.box.min_y || param_.box.max_y != new_param.box.max_y ||
      param_.box.min_z != new_param.box.min_z || param_.box.max_z != new_param.box.max_z ||
      param_.negative != new_param.negative ||
      param_.use_ray_intersection != new_param.use_ray_intersection) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::param_callback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.box.min_x, new_param.box.min_y, new_param.box.min_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::param_callback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.box.max_x, new_param.box.max_y, new_param.box.max_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::param_callback] Setting the filter negative flag to: %s.", get_name(),
        new_param.negative ? "true" : "false");
      RCLCPP_DEBUG(
        get_logger(), "[%s::param_callback] Setting the use ray intersection flag to: %s.", get_name(),
        new_param.use_ray_intersection ? "true" : "false");
      param_ = new_param;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::CropBoxFilterComponent)
