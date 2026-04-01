// Copyright 2025 TIER IV.
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

#ifndef CORE__VAD_PIPELINE_HPP_
#define CORE__VAD_PIPELINE_HPP_

#include "core/vad_logger.hpp"
#include "data_types.hpp"
#include "vad_model.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <memory>
#include <optional>

namespace autoware::tensorrt_vad
{

class VadPipeline
{
public:
  VadPipeline(
    const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & backbone_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_no_prev_config,
    const std::shared_ptr<VadLogger> & logger);

  std::optional<VadOutputData> infer(const VadInputData & vad_input_data);

private:
  std::unique_ptr<VadModel<VadLogger>> model_;
};

}  // namespace autoware::tensorrt_vad

#endif  // CORE__VAD_PIPELINE_HPP_
