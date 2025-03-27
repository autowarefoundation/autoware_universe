#ifndef WAYPOINT_SAVER__WAYPOINT_SAVER_H_
#define WAYPOINT_SAVER__WAYPOINT_SAVER_H_

#include "waypoint_maker_utils/utils.hpp"

// ros
#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

// autoware
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <tier4_planning_msgs/srv/save_waypoint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class WaypointSaver : public rclcpp::Node
{
public:
  explicit WaypointSaver(const rclcpp::NodeOptions & node_options);
  ~WaypointSaver();

private:
  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;

  rclcpp::Service<tier4_planning_msgs::srv::SaveWaypoint>::SharedPtr srv_record_;

  // tf
  autoware::universe_utils::SelfPoseListener self_pose_listener_;

  // Parameters
  std::string output_file_str_;
  std::string output_file_str_backup_;
  std::string map_frame_;
  std::string base_link_frame_;
  bool record_;
  float interval_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr_;

  /**
   * @brief callback on current pose
   * @param [in] msg pose message
   */
  void poseCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void onRecordService(
    const tier4_planning_msgs::srv::SaveWaypoint::Request::SharedPtr request,
    const tier4_planning_msgs::srv::SaveWaypoint::Response::SharedPtr response);

  /**
   * @brief writing current pose and velocity
   * @param [in] current_pose
   * @param [in] velocity
   */
  void outputProcessing(
    const geometry_msgs::msg::Pose & current_pose, const double & velocity) const;
};

#endif  // WAYPOINT_SAVER__WAYPOINT_SAVER_H_
