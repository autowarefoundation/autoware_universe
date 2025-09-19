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

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/collector_matching_strategy.hpp"

template class autoware::pointcloud_preprocessor::NaiveMatchingStrategy<
  autoware::pointcloud_preprocessor::CudaPointCloud2Traits>;
template class autoware::pointcloud_preprocessor::AdvancedMatchingStrategy<
  autoware::pointcloud_preprocessor::CudaPointCloud2Traits>;
