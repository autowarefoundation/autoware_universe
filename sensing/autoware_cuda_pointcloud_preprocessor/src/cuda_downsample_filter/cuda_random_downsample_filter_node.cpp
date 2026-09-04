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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter_node.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::cuda_pointcloud_preprocessor
{

CudaRandomDownsampleFilterNode::CudaRandomDownsampleFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_random_downsample_filter", node_options)
{
  // No default: a point budget is a tuning decision per map and per LiDAR, and
  // a wrong one degrades localization quietly rather than failing. Better to
  // refuse to start than to invent one.
  const std::int64_t sample_num = declare_parameter<std::int64_t>("sample_num");

  if (sample_num <= 0) {
    throw std::runtime_error(
      "cuda_random_downsample_filter: sample_num must be positive, got " +
      std::to_string(sample_num) +
      "; a non-positive budget would publish empty clouds "
      "and stall NDT with no error anywhere downstream");
  }

  filter_ = std::make_unique<CudaRandomDownsampleFilter>(static_cast<std::size_t>(sample_num));

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&CudaRandomDownsampleFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void CudaRandomDownsampleFilterNode::pointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  // An empty cloud is republished rather than dropped: downstream timeout
  // diagnostics distinguish "the LiDAR stopped" from "the scan was empty" only
  // if the empty scan still arrives.
  pub_->publish(filter_->filter(*msg));
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaRandomDownsampleFilterNode)
