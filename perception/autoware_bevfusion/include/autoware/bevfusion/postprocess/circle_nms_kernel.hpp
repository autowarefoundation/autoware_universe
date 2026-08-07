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

#ifndef AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
#define AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_

#include "autoware/bevfusion/utils.hpp"

#include <thrust/device_vector.h>

#include <cstddef>

namespace autoware::bevfusion
{
// Distance-based non-maximum suppression (xy-plane distance instead of IoU) is
// implemented as a private PostprocessCuda method (see postprocess_kernel.hpp),
// reusing that class's pre-allocated scratch. The kernel launcher lives in
// circle_nms_kernel.cu.
}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
