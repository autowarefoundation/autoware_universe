#ifndef AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_BEV_SHIFT_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_BEV_SHIFT_CONVERTER_HPP_

#include "autoware/tensorrt_vad/converter.hpp"
#include <vector>
#include <cmath>

namespace autoware::tensorrt_vad::vad_interface {

using ShiftData = std::vector<float>;
using CanBusData = std::vector<float>;

/**
 * @brief InputBEVShiftConverter handles BEV (Bird's Eye View) shift calculation
 * 
 * This class calculates the BEV coordinate shift based on vehicle movement:
 * - Position change analysis from consecutive CAN-Bus frames
 * - Grid-based coordinate transformation for BEV representation
 * - Ego vehicle angle consideration for proper shift direction
 * - Default shift handling for initial frames
 */
class InputBEVShiftConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer
   * @param config Reference to configuration containing BEV and detection range parameters
   */
  InputBEVShiftConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Calculate BEV shift from CAN-Bus data changes
   * @param can_bus Current frame's CAN-Bus data containing position and orientation
   * @param prev_can_bus Previous frame's CAN-Bus data for delta calculation
   * @return ShiftData 2-element vector containing [shift_x, shift_y] in normalized BEV coordinates
   */
  ShiftData process_shift(
    const CanBusData& can_bus,
    const CanBusData& prev_can_bus) const;

private:
  // The width and length of the BEV feature map in meters (corresponds to the real-world width and length covered by the BEV grid)
  float real_w_;
  float real_h_;
  
  // Default delta values when previous CAN-Bus data is not available
  float default_delta_x_;
  float default_delta_y_;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_BEV_SHIFT_CONVERTER_HPP_
