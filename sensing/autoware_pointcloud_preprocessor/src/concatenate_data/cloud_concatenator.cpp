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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_concatenator.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

// The concatenator is templated on the point cloud message type (implementation in the .ipp); the
// PointCloud2 instantiation ships in concatenate_core so the bindings and tests link against one
// copy. The CUDA node instantiates its cuda_blackboard::CudaPointCloud2 variant in its own TU.
template class autoware::pointcloud_preprocessor::CloudConcatenator<sensor_msgs::msg::PointCloud2>;
