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

#ifndef PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__CONFIG_BUILDER_HPP_
#define PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__CONFIG_BUILDER_HPP_

#include "pipeline_runner.hpp"

#include <string>
#include <vector>

namespace autoware::traffic_light
{

// The command line, parsed but not yet resolved into a run: see build_bag2bag_config().
struct CommandLineArgs
{
  std::string input_bag_path;
  std::string map_path;
  std::string output_bag_path;
  std::vector<int> camera_indices;
  std::string ml_model_path;
  std::string config_path;
};

// Resolves the command line into a run: reads the recognition parameter file, turns every relative
// model / label path into an absolute one, and derives each camera's topic names.
Bag2BagConfig build_bag2bag_config(const CommandLineArgs & args);

}  // namespace autoware::traffic_light

#endif  // PERCEPTION__AUTOWARE_TRAFFIC_LIGHT_PIPELINE__BAG2BAG__CONFIG_BUILDER_HPP_
