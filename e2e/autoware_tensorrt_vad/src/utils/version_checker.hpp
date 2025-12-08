// Copyright 2025 TIER IV.
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

#ifndef UTILS__VERSION_CHECKER_HPP_
#define UTILS__VERSION_CHECKER_HPP_

#include "utils/constants.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace autoware::tensorrt_vad::utils
{
using json = nlohmann::json;

/**
 * @brief Validates model version compatibility
 *
 * Checks if the model's major version matches the version supported by this node.
 * Models with different major versions have incompatible I/O or architecture changes.
 *
 * @param json_path Path to the model parameter JSON file
 * @throws std::runtime_error if version is incompatible or missing
 */
inline void check_model_version(const std::string & json_path)
{
  std::ifstream file(json_path);
  if (!file) {
    throw std::runtime_error("Could not open param.json file: " + json_path);
  }

  json j;
  file >> j;

  const std::string error_msg =
    "Please use the appropriate version of VAD model and vad-carla-tiny.param.json. "
    "Refer to README.md for more details.";

  if (!j.contains("major_version")) {
    throw std::runtime_error("Missing 'major_version' key in param.json. " + error_msg);
  }

  const int major_version = j["major_version"].get<int>();
  if (major_version != constants::SUPPORTED_MAJOR_VERSION) {
    throw std::runtime_error(
      "Unsupported major_version: " + std::to_string(major_version) +
      ". This node supports major_version " + std::to_string(constants::SUPPORTED_MAJOR_VERSION) +
      ". " + error_msg);
  }
}

}  // namespace autoware::tensorrt_vad::utils

#endif  // UTILS__VERSION_CHECKER_HPP_
