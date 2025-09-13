#ifndef AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_CAN_BUS_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_CAN_BUS_CONVERTER_HPP_

#include "autoware/tensorrt_vad/converter.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <vector>
#include <cmath>

namespace autoware::tensorrt_vad::vad_interface {

using CanBusData = std::vector<float>;

/**
 * @brief InputCanBusConverter handles CAN-Bus data processing and velocity calculations
 * 
 * This class converts ROS odometry and acceleration messages to VAD CAN-Bus format:
 * - Coordinate system transformation (Autoware to VAD coordinate system)
 * - Position, orientation, velocity, acceleration, and angular velocity processing
 * - Patch angle calculation from quaternion orientation
 * - Current longitudinal velocity calculation for trajectory generation
 */
class InputCanBusConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing default values
   */
  InputCanBusConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process odometry and acceleration data to generate CAN-Bus data
   * @param kinematic_state ROS Odometry message containing position, orientation, and velocity
   * @param acceleration ROS AccelWithCovarianceStamped message containing acceleration data
   * @param prev_can_bus Previous frame's CAN-Bus data for delta calculations
   * @return CanBusData 18-element vector containing processed CAN-Bus information
   */
  CanBusData process_can_bus(
    const nav_msgs::msg::Odometry::ConstSharedPtr& kinematic_state,
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& acceleration,
    const std::vector<float>& prev_can_bus) const;

private:
  // Default delta yaw value when previous CAN-Bus data is not available
  float default_delta_yaw_;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_CAN_BUS_CONVERTER_HPP_
