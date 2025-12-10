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

/**
 * @file filter_registrations.cpp
 * @brief Registers all filter implementations with the FilterRegistry
 *
 * This file is responsible for registering all concrete filter implementations
 * so they can be instantiated by the DAG executor based on configuration.
 *
 * NOTE: OrganizeFilter and FinalizeFilter are NOT included here because they
 * are internalized in the DAG node (automatic at entry/exit points).
 */

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/cropbox_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/distortion_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/ring_outlier_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/transform_filter.hpp"

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Static initialization function to register all filters
 *
 * This function is called automatically during static initialization to
 * register all filter types with the FilterRegistry.
 */
void registerAllFilters()
{
  // Register all user-configurable filter types
  registerFilterType<TransformFilter>("TransformFilter");
  registerFilterType<CropBoxFilter>("CropBoxFilter");
  registerFilterType<DistortionFilter>("DistortionFilter");
  registerFilterType<RingOutlierFilter>("RingOutlierFilter");
  registerFilterType<DownsampleFilter>("DownsampleFilter");
}

// Call registration function during static initialization
[[maybe_unused]] static const bool filters_registered = []() {
  registerAllFilters();
  return true;
}();

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
