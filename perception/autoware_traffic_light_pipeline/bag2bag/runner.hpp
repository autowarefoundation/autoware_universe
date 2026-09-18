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

#ifndef PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__RUNNER_HPP_
#define PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__RUNNER_HPP_

#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <string>
#include <vector>

namespace autoware::traffic_light
{

// One camera's namespace and the input / output topic names derived from it.
struct CameraConfig
{
  std::string camera_namespace;
  std::string camera_info_topic;
  std::string compressed_image_topic;
  std::string traffic_signals_topic;
  std::string rois_topic;
};

// The whole run: N cameras sharing one set of models/thresholds, over one input bag.
struct Bag2BagConfig
{
  std::string input_bag_path;
  std::string output_bag_path;
  std::string lanelet2_map_path;
  std::string map_projector_info_path;

  std::vector<CameraConfig> cameras;
  TrafficLightRecognitionConfig recognition;
};

void run_bag2bag(const Bag2BagConfig & config);

}  // namespace autoware::traffic_light

#endif  // PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__RUNNER_HPP_
