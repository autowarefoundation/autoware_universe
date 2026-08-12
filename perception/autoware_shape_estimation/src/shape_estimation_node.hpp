// Copyright 2018 Autoware Foundation. All rights reserved.
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

#ifndef SHAPE_ESTIMATION_NODE_HPP_
#define SHAPE_ESTIMATION_NODE_HPP_

#include "autoware/shape_estimation/shape_estimator.hpp"

#ifdef USE_CUDA
#include "autoware/shape_estimation/tensorrt_shape_estimator.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#endif

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

namespace autoware::shape_estimation
{

using autoware_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
class ShapeEstimationNode : public autoware::agnocast_wrapper::Node
{
private:
  // ros
  AUTOWARE_PUBLISHER_PTR(DetectedObjectsWithFeature) pub_;
  AUTOWARE_SUBSCRIPTION_PTR(DetectedObjectsWithFeature) sub_;
  std::unique_ptr<autoware_utils::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;

  // debug publisher
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    processing_time_publisher_;

  void callback(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DetectedObjectsWithFeature) & input_msg);

  std::unique_ptr<ShapeEstimator> estimator_;
  bool use_vehicle_reference_yaw_;
  bool use_vehicle_reference_shape_size_;
  bool fix_filtered_objects_label_to_unknown_;

#ifdef USE_CUDA
  std::unique_ptr<TrtShapeEstimator> tensorrt_shape_estimator_;
#endif

  bool use_ml_shape_estimation_;
  size_t min_points_;

public:
  explicit ShapeEstimationNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace autoware::shape_estimation

#endif  // SHAPE_ESTIMATION_NODE_HPP_
