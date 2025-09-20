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

#ifndef COMMON__UTILS__MEMORY_HPP_
#define COMMON__UTILS__MEMORY_HPP_

#include <memory>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

template <typename T>
std::vector<T *> raws(const std::vector<std::unique_ptr<T>> & ptrs)
{
  std::vector<T *> result;
  for (const auto & ptr : ptrs) {
    result.push_back(ptr.get());
  }
  return result;
}

template <typename T, typename B>
std::vector<T *> filter(const std::vector<B *> & ptrs)
{
  std::vector<T *> result;
  for (const auto & ptr : ptrs) {
    if (T * t = dynamic_cast<T *>(ptr)) {
      result.push_back(t);
    }
  }
  return result;
}

template <typename T, typename S>
std::vector<std::unique_ptr<T>> filter(std::vector<std::unique_ptr<T>> && ptrs, const S & excludes)
{
  std::vector<std::unique_ptr<T>> result;
  for (auto & ptr : ptrs) {
    if (!excludes.count(ptr.get())) {
      result.push_back(std::move(ptr));
    }
  }
  return result;
}

template <typename T, typename S>
std::vector<T *> filter(const std::vector<T *> & ptrs, const S & excludes)
{
  std::vector<T *> result;
  for (auto & ptr : ptrs) {
    if (!excludes.count(ptr)) {
      result.push_back(std::move(ptr));
    }
  }
  return result;
}

template <typename T, typename B>
std::vector<std::unique_ptr<B>> release(std::vector<std::unique_ptr<B>> && ptrs)
{
  std::vector<std::unique_ptr<B>> result;
  for (auto & ptr : ptrs) {
    if (!dynamic_cast<T *>(ptr.get())) {
      result.push_back(std::move(ptr));
    }
  }
  return result;
}

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__UTILS__MEMORY_HPP_
