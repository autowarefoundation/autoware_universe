// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <boost/circular_buffer.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{

using sensor_msgs::msg::RegionOfInterest;

class Debugger
{
public:
  explicit Debugger(
    autoware::agnocast_wrapper::Node * node_ptr, const std::size_t image_num,
    const std::size_t image_buffer_size, std::vector<std::string> input_camera_topics);

  void publishImage(const std::size_t image_id, const rclcpp::Time & stamp);

  void clear();

  std::vector<RegionOfInterest> image_rois_;
  std::vector<RegionOfInterest> obstacle_rois_;
  std::vector<Eigen::Vector2d> obstacle_points_;
  std::vector<double> max_iou_for_image_rois_;

private:
  void imageCallback(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Image) input_image_msg,
    const std::size_t image_id);

  std::vector<std::string> input_camera_topics_;
  std::vector<AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::Image)> image_subs_;
  std::vector<AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::Image)> image_pubs_;
  std::vector<boost::circular_buffer<AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Image)>>
    image_buffers_;

  std::size_t image_buffer_size_;
};

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__DEBUGGER_HPP_
