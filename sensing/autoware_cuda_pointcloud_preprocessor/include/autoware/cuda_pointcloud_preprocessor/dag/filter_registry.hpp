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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_REGISTRY_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_REGISTRY_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Registry for managing filter types and their factories
 */
class FilterRegistry
{
public:
  using FilterFactory = std::function<std::unique_ptr<IFilter>()>;
  using FilterMetadataGetter = std::function<FilterMetadata()>;

  /**
   * @brief Register a filter type
   * @param type_name Unique name for the filter type
   * @param factory Factory function to create filter instances
   * @param metadata_getter Function to get filter metadata
   */
  void registerFilter(
    const std::string & type_name, FilterFactory factory, FilterMetadataGetter metadata_getter);

  /**
   * @brief Create a filter instance
   * @param type_name Name of the filter type
   * @return Unique pointer to the filter instance, or nullptr if not found
   */
  std::unique_ptr<IFilter> createFilter(const std::string & type_name) const;

  /**
   * @brief Get filter metadata
   * @param type_name Name of the filter type
   * @return Filter metadata, or empty metadata if not found
   */
  FilterMetadata getMetadata(const std::string & type_name) const;

  /**
   * @brief List all registered filter types
   * @return Vector of registered filter type names
   */
  std::vector<std::string> listFilters() const;

  /**
   * @brief Check if a filter type is registered
   * @param type_name Name of the filter type
   * @return true if registered, false otherwise
   */
  bool isRegistered(const std::string & type_name) const;

private:
  std::map<std::string, FilterFactory> factories_;
  std::map<std::string, FilterMetadataGetter> metadata_getters_;
};

/**
 * @brief Get the global filter registry instance
 * @return Reference to the global filter registry
 */
FilterRegistry & getFilterRegistry();

/**
 * @brief Template helper to register a filter type
 * @tparam FilterType The filter class to register
 * @param type_name Unique name for the filter type
 */
template <typename FilterType>
void registerFilterType(const std::string & type_name)
{
  auto & registry = getFilterRegistry();
  registry.registerFilter(
    type_name, []() -> std::unique_ptr<IFilter> { return std::make_unique<FilterType>(); },
    []() -> FilterMetadata {
      FilterType temp;
      return temp.getMetadata();
    });
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTER_REGISTRY_HPP_
