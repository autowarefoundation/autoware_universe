// Copyright 2025 TIER IV, Inc.
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

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp"

#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void FilterRegistry::registerFilter(
  const std::string & type_name, FilterFactory factory, FilterMetadataGetter metadata_getter)
{
  if (factories_.find(type_name) != factories_.end()) {
    throw std::runtime_error("Filter type '" + type_name + "' is already registered");
  }
  factories_[type_name] = std::move(factory);
  metadata_getters_[type_name] = std::move(metadata_getter);
}

std::unique_ptr<IFilter> FilterRegistry::createFilter(const std::string & type_name) const
{
  auto it = factories_.find(type_name);
  if (it == factories_.end()) {
    return nullptr;
  }
  return it->second();
}

FilterMetadata FilterRegistry::getMetadata(const std::string & type_name) const
{
  auto it = metadata_getters_.find(type_name);
  if (it == metadata_getters_.end()) {
    return FilterMetadata{};  // Return empty metadata
  }
  return it->second();
}

std::vector<std::string> FilterRegistry::listFilters() const
{
  std::vector<std::string> filters;
  filters.reserve(factories_.size());
  for (const auto & [name, _] : factories_) {
    filters.push_back(name);
  }
  return filters;
}

bool FilterRegistry::isRegistered(const std::string & type_name) const
{
  return factories_.find(type_name) != factories_.end();
}

// Global registry instance
namespace
{
FilterRegistry g_filter_registry;
}

FilterRegistry & getFilterRegistry()
{
  return g_filter_registry;
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
