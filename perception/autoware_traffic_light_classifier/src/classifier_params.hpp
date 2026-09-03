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

#ifndef CLASSIFIER_PARAMS_HPP_
#define CLASSIFIER_PARAMS_HPP_

// Node-side helpers that declare the classifier parameters on a node and return the plain config
// structs the (ROS-free) classifier cores consume. Lives at the node layer, not under classifier/,
// because declaring parameters is a Node concern; the cores themselves never touch rclcpp.

#include "classifier/color_classifier.hpp"

#include <rclcpp/rclcpp.hpp>

#if ENABLE_GPU
#include "classifier/cnn_classifier.hpp"
#include "classifier/cnn_lamp_recognizer.hpp"
#endif

namespace autoware::traffic_light
{
// Declare the 18 HSV threshold parameters on `node`, seeding each from the HSVConfig defaults, and
// return the resulting config.
HSVConfig declare_hsv_config(rclcpp::Node * node);

#if ENABLE_GPU
// Declare the CNN parameters on `node`, read the label file, and return the config. ROS params
// cannot load std::vector<float>, so mean/std are declared as std::vector<double> and narrowed
// here.
CNNConfig declare_cnn_config(rclcpp::Node * node);

// Declare the lamp recognizer parameters on `node` and return the config. The anchors and the
// float scalars (score / nms thresholds, scale_x_y) are narrowed double->float here (a ROS
// quirk); config-data invariants live in the core ctor, not here.
CnnLampRecognizerConfig declare_lamp_config(rclcpp::Node * node);
#endif
}  // namespace autoware::traffic_light

#endif  // CLASSIFIER_PARAMS_HPP_
