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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__BACKBONE_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__BACKBONE_HPP_

#include "autoware/tensorrt_vad/networks/net.hpp"

namespace autoware::tensorrt_vad
{

class Backbone : public Net
{
public:
  Backbone(
    const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & trt_common_config,
    const std::string & plugins_path, std::shared_ptr<VadLogger> logger);

  std::vector<autoware::tensorrt_common::NetworkIO> setup_network_io(
    const VadConfig & vad_config) override;
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__BACKBONE_HPP_
