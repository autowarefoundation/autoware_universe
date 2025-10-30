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

#ifndef AUTOWARE__TENSORRT_VAD__CONVERTER_HPP_
#define AUTOWARE__TENSORRT_VAD__CONVERTER_HPP_

#include "coordinate_transformer.hpp"
#include "vad_interface_config.hpp"

namespace autoware::tensorrt_vad::vad_interface
{

/**
 * @brief Base class for all data converters
 */
class Converter
{
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing all necessary parameters
   */
  Converter(
    const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config);

  /**
   * @brief Virtual destructor to enable proper cleanup of derived classes
   */
  virtual ~Converter() = default;

protected:
  const CoordinateTransformer & coordinate_transformer_;
  const VadInterfaceConfig & config_;
};

}  // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE__TENSORRT_VAD__CONVERTER_HPP_
