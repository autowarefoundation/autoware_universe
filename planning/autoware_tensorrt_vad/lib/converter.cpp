#include "autoware/tensorrt_vad/converter.hpp"

namespace autoware::tensorrt_vad::vad_interface {

Converter::Converter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : coordinate_transformer_(coordinate_transformer), config_(config)
{
}

} // namespace autoware::tensorrt_vad::vad_interface
