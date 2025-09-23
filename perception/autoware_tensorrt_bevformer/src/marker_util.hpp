// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// cspell:ignore BEVFORMER

#ifndef MARKER_UTIL_HPP_
#define MARKER_UTIL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::tensorrt_bevformer
{
/**
 * @brief Publishes visualization markers for BEVFormer debug mode
 *
 * Applies orientation corrections to detected objects and publishes a MarkerArray
 * for RViz visualization. This is used to visualize 3D bounding boxes detected
 * by BEVFormer when debug mode is enabled.
 *
 * @param logger The ROS logger for status messages
 * @param marker_pub The publisher used to publish the MarkerArray
 * @param objects The detected objects to be visualized (will be modified in-place)
 */
void publishDebugMarkers(
  const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> & marker_pub,
  const autoware_perception_msgs::msg::DetectedObjects & bevformer_objects);

}  // namespace autoware::tensorrt_bevformer
#endif  // MARKER_UTIL_HPP_
