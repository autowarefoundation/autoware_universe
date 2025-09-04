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

#include "transforms.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

Compose::Compose(const std::vector<std::shared_ptr<Transform>> & transforms)
: transforms_(transforms)
{
}

DataDict Compose::operator()(DataDict results) const
{
  // Apply each transform in sequence
  for (const auto & transform : transforms_) {
    if (transform) {
      results = (*transform)(results);
    }
  }
  return results;
}

std::string Compose::toString() const
{
  std::string str = "Compose([";
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (i > 0) {
      str += ", ";
    }
    if (transforms_[i]) {
      str += transforms_[i]->toString();
    } else {
      str += "null";
    }
  }
  str += "])";
  return str;
}

}  // namespace preprocessing
}  // namespace bevformer
