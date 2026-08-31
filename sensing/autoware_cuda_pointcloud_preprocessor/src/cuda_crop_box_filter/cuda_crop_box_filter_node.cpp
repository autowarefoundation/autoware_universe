// Copyright 2026 NEWSLab, National Taiwan University
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_crop_box_filter/cuda_crop_box_filter_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::cuda_pointcloud_preprocessor
{

CudaCropBoxFilterNode::CudaCropBoxFilterNode(const rclcpp::NodeOptions & node_options)
: Node("cuda_crop_box_filter", node_options)
{
  CudaCropBoxFilter::BoxParams params{};
  params.min_x = static_cast<float>(declare_parameter<double>("min_x"));
  params.max_x = static_cast<float>(declare_parameter<double>("max_x"));
  params.min_y = static_cast<float>(declare_parameter<double>("min_y"));
  params.max_y = static_cast<float>(declare_parameter<double>("max_y"));
  params.min_z = static_cast<float>(declare_parameter<double>("min_z"));
  params.max_z = static_cast<float>(declare_parameter<double>("max_z"));
  params.negative = declare_parameter<bool>("negative", false);

  // The CPU component can crop in a different frame and looks up tf to do it.
  // This one cannot, so it is told which frame the box is expressed in and
  // refuses to guess: cropping the right box in the wrong frame removes the
  // wrong points and nothing downstream would report it.
  expected_frame_ = declare_parameter<std::string>("input_frame", "");

  if (params.min_x > params.max_x || params.min_y > params.max_y || params.min_z > params.max_z) {
    throw std::runtime_error(
      "cuda_crop_box_filter: an inverted bound was given (min greater than max); "
      "this would silently drop every point");
  }

  filter_ = std::make_unique<CudaCropBoxFilter>(params);

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&CudaCropBoxFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void CudaCropBoxFilterNode::pointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  if (!expected_frame_.empty() && msg->header.frame_id != expected_frame_) {
    if (!warned_about_frame_) {
      RCLCPP_ERROR(
        get_logger(),
        "Cloud is in frame '%s' but the crop box is configured for '%s'. This filter does not "
        "transform; dropping. Set input_frame to match, or crop upstream.",
        msg->header.frame_id.c_str(), expected_frame_.c_str());
      warned_about_frame_ = true;
    }
    return;
  }

  auto output = filter_->filter(*msg);
  if (!output) {
    if (!warned_about_layout_) {
      RCLCPP_ERROR(
        get_logger(), "Cloud has no float32 x/y/z fields; this filter cannot read it. Dropping.");
      warned_about_layout_ = true;
    }
    return;
  }

  pub_->publish(std::move(output));
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::cuda_pointcloud_preprocessor::CudaCropBoxFilterNode)
