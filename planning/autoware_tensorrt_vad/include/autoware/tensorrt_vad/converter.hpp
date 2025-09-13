#ifndef AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_

#include "coordinate_transformer.hpp"
#include "vad_interface_config.hpp"

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief Base class for all data converters
 */
class Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing all necessary parameters
   */
  Converter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);
  
  /**
   * @brief Virtual destructor to enable proper cleanup of derived classes
   */
  virtual ~Converter() = default;

protected:
  const CoordinateTransformer& coordinate_transformer_;
  const VadInterfaceConfig& config_;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_
