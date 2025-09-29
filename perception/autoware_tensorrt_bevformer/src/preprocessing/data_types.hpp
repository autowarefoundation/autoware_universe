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

#ifndef PREPROCESSING__DATA_TYPES_HPP_
#define PREPROCESSING__DATA_TYPES_HPP_

#include <opencv2/opencv.hpp>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

class DataDict;

using DataValue = std::variant<
  bool, int, float, std::string, std::vector<float>, std::vector<int>, std::vector<std::string>,
  std::vector<cv::Mat>, std::vector<cv::Size>, std::shared_ptr<DataDict>>;

class DataDict : public std::map<std::string, DataValue>
{
public:
  DataDict() = default;

  // Check if a key corresponds to a nested dictionary
  bool isNestedDict(const std::string & key) const
  {
    auto it = find(key);
    if (it != end()) {
      return std::holds_alternative<std::shared_ptr<DataDict>>(it->second);
    }
    return false;
  }

  // Get a reference to a nested dictionary (const version)
  const DataDict & getNestedDict(const std::string & key) const
  {
    if (isNestedDict(key)) {
      return *std::get<std::shared_ptr<DataDict>>(at(key));
    }
    static const DataDict empty;
    return empty;
  }

  // Get a reference to a nested dictionary
  DataDict & getNestedDict(const std::string & key)
  {
    if (!isNestedDict(key)) {
      setNestedDict(key);
    }
    return *std::get<std::shared_ptr<DataDict>>((*this)[key]);
  }

  // Create a nested dictionary
  void setNestedDict(const std::string & key) { (*this)[key] = std::make_shared<DataDict>(); }
};

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__DATA_TYPES_HPP_
