// Copyright 2025 TIER IV.
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

#ifndef UTILS__CLASSIFICATION_LOADER_HPP_
#define UTILS__CLASSIFICATION_LOADER_HPP_

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad::utils
{

struct ClassificationConfig
{
  std::vector<std::string> class_names;
  std::vector<double> thresholds;
  std::vector<std::string> * target_class_names;
  std::map<std::string, float> * target_thresholds;
  int32_t * num_classes;  // optional
  std::string validation_context;
};

inline bool load_classification_config(const ClassificationConfig & params)
{
  if (params.class_names.size() != params.thresholds.size()) {
    return false;
  }

  try {
    params.target_class_names->assign(params.class_names.begin(), params.class_names.end());
    if (params.num_classes != nullptr) {
      *params.num_classes = static_cast<int32_t>(params.class_names.size());
    }

    params.target_thresholds->clear();
    for (std::size_t i = 0; i < params.class_names.size(); ++i) {
      (*params.target_thresholds)[params.class_names[i]] = static_cast<float>(params.thresholds[i]);
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace autoware::tensorrt_vad::utils

#endif  // UTILS__CLASSIFICATION_LOADER_HPP_