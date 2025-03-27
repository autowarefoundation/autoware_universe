#ifndef WAYPOINT_MAKER_UTILS__DUBUG_MAKER_HPP_
#define WAYPOINT_MAKER_UTILS__DUBUG_MAKER_HPP_

#include "waypoint_maker_utils/utils.hpp"

namespace waypoint_maker
{
visualization_msgs::msg::Marker createTrajectoryMarkerArray(
  const autoware_planning_msgs::msg::Trajectory & tra)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  marker.id = 0L;
  marker.ns = "trajectory_debug";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.7;

  for (unsigned i = 0; i < tra.points.size(); i++) {
    marker.points.push_back(autoware::universe_utils::createPoint(
      tra.points.at(i).pose.position.x, tra.points.at(i).pose.position.y,
      tra.points.at(i).pose.position.z));
  }

  return marker;
}

visualization_msgs::msg::MarkerArray createBoundMarkerArray(
  const std::array<std::vector<geometry_msgs::msg::Point>, 2> & bound)
{
  visualization_msgs::msg::MarkerArray bound_pub;

  for (unsigned int i = 0; i < bound.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    marker.id = i;
    marker.ns = "bound_debug";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.7;

    for (unsigned int j = 0; j < bound.at(i).size(); j++) {
      marker.points.push_back(bound.at(i).at(j));
    }
    bound_pub.markers.push_back(marker);
  }

  return bound_pub;
}
}  // namespace waypoint_maker

#endif  // WAYPOINT_MAKER_UTILS__DUBUG_MAKER_HPP_
