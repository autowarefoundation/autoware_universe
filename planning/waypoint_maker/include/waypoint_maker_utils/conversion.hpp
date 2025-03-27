#ifndef WAYPOINT_MAKER_UTILS__CONVERSION_HPP_
#define WAYPOINT_MAKER_UTILS__CONVERSION_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace waypoint_maker
{
/**
 * @brief current velocity kmph to mps
 * @param [in] velocity_kmph current velocity kmph
 */
constexpr double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000.0) / 3600.0;
}
/**
 * @brief current velocity mps to kmps
 * @param [in] velocity_mps current velocity mps
 */
constexpr double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 3600.0) / 1000.0;
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

}  // namespace waypoint_maker
#endif  // WAYPOINT_MAKER_UTILS__CONVERSION_HPP_
