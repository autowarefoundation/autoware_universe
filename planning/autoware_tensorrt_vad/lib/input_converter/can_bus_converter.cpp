#include "autoware/tensorrt_vad/input_converter/can_bus_converter.hpp"
#include <Eigen/Dense>

namespace autoware::tensorrt_vad::vad_interface {

InputCanBusConverter::InputCanBusConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config),
    default_delta_yaw_(0.0f)
{
}

CanBusData InputCanBusConverter::process_can_bus(
  const nav_msgs::msg::Odometry::ConstSharedPtr& kinematic_state,
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& acceleration,
  const std::vector<float>& prev_can_bus) const
{
  CanBusData can_bus(18, 0.0f);

  // Apply Autoware to VAD base_link coordinate transformation to position
  auto [vad_x, vad_y, vad_z] =
      coordinate_transformer_.aw2vad_xyz(kinematic_state->pose.pose.position.x,
                             kinematic_state->pose.pose.position.y,
                             kinematic_state->pose.pose.position.z);

  // translation (0:3)
  can_bus[0] = vad_x;
  can_bus[1] = vad_y;
  can_bus[2] = vad_z;

  // Apply Autoware to VAD base_link coordinate transformation to orientation
  Eigen::Quaternionf q_aw(
      kinematic_state->pose.pose.orientation.w,
      kinematic_state->pose.pose.orientation.x,
      kinematic_state->pose.pose.orientation.y,
      kinematic_state->pose.pose.orientation.z);

  Eigen::Quaternionf q_vad = coordinate_transformer_.aw2vad_quaternion(q_aw);

  // rotation (3:7)
  can_bus[3] = q_vad.x();
  can_bus[4] = q_vad.y();
  can_bus[5] = q_vad.z();
  can_bus[6] = q_vad.w();

  // Apply Autoware to VAD base_link coordinate transformation to acceleration
  auto [vad_ax, vad_ay, vad_az] =
      coordinate_transformer_.aw2vad_xyz(acceleration->accel.accel.linear.x,
                             acceleration->accel.accel.linear.y,
                             acceleration->accel.accel.linear.z);

  // acceleration (7:10)
  can_bus[7] = vad_ax;
  can_bus[8] = vad_ay;
  can_bus[9] = vad_az;

  // Apply Autoware to VAD base_link coordinate transformation to angular velocity
  auto [vad_wx, vad_wy, vad_wz] =
      coordinate_transformer_.aw2vad_xyz(kinematic_state->twist.twist.angular.x,
                             kinematic_state->twist.twist.angular.y,
                             kinematic_state->twist.twist.angular.z);

  // angular velocity (10:13)
  can_bus[10] = vad_wx;
  can_bus[11] = vad_wy;
  can_bus[12] = vad_wz;

  // Apply Autoware to VAD base_link coordinate transformation to velocity
  auto [vad_vx, vad_vy, vad_vz] =
      coordinate_transformer_.aw2vad_xyz(kinematic_state->twist.twist.linear.x,
                             kinematic_state->twist.twist.linear.y,
                             0.0f); // Set z-direction velocity to 0

  // velocity (13:16)
  can_bus[13] = vad_vx;
  can_bus[14] = vad_vy;
  can_bus[15] = vad_vz;

  // Calculate patch_angle[rad] (16)
  // yaw = ArcTan(2 * (w * z + x * y) / (1 - 2 * (y ** 2 + z ** 2)))
  double yaw = std::atan2(
      2.0 * (can_bus[6] * can_bus[5] + can_bus[3] * can_bus[4]),
      1.0 - 2.0 * (can_bus[4] * can_bus[4] + can_bus[5] * can_bus[5]));
  if (yaw < 0)
    yaw += 2 * M_PI;
  can_bus[16] = static_cast<float>(yaw);

  // Calculate patch_angle[deg] (17)
  float delta_yaw = default_delta_yaw_;
  
  if (!prev_can_bus.empty()) {
    float prev_angle = prev_can_bus[16];
    delta_yaw = yaw - prev_angle;
  }
  
  can_bus[17] = delta_yaw * 180.0f / M_PI;

  return can_bus;
}
} // namespace autoware::tensorrt_vad::vad_interface
