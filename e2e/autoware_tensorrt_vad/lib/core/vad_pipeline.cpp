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

#include "../src/core/vad_pipeline.hpp"

namespace autoware::tensorrt_vad
{

VadPipeline::VadPipeline(
  const VadConfig & vad_config,
  const autoware::tensorrt_common::TrtCommonConfig & backbone_config,
  const autoware::tensorrt_common::TrtCommonConfig & head_config,
  const autoware::tensorrt_common::TrtCommonConfig & head_no_prev_config,
  const std::shared_ptr<VadLogger> & logger)
{
  model_ = std::make_unique<VadModel<VadLogger>>(
    vad_config, backbone_config, head_config, head_no_prev_config, logger);
}

std::optional<VadOutputData> VadPipeline::infer(const VadInputData & vad_input_data)
{
  if (!model_) {
    return std::nullopt;
  }
  return model_->infer(vad_input_data);
}

}  // namespace autoware::tensorrt_vad
