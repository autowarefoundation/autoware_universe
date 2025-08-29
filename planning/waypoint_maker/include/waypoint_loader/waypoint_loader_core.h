#ifndef WAYPOINT_LOADER__WAYPOINT_LOADER_CORE_H_
#define WAYPOINT_LOADER__WAYPOINT_LOADER_CORE_H_

#include "waypoint_maker_utils/dubug_maker.hpp"

// ROS
#include "tf2/utils.h"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/optional.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>

class WaypointLoaderNode : public rclcpp::Node
{
public:
  WaypointLoaderNode();

private:
  // publisher & subscriber
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_visualization_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr file_location_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_following_state_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr file_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr sub_scenario_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // variables
  const autoware::vehicle_info_utils::VehicleInfo vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  std::string lane_csv_position_;
  std::string csv_file_;
  double max_velocity_;
  double bound_margin_;

  double distance_threshold_;
  double angle_threshold_;
  double stop_duration_;
  boost::optional<rclcpp::Time> stop_time_;

  double update_rate_;
  std::string base_link_frame_;
  std::string map_frame_;

  signed short state_;
  signed short id_flip_;

  std::string current_scenario_;

  tier4_planning_msgs::msg::RouteState route_state_;
  autoware_planning_msgs::msg::Trajectory basic_trajectory_;
  autoware_planning_msgs::msg::Trajectory ouput_trajectory_;
  geometry_msgs::msg::PoseStamped start_pose_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;

  // Callback function
  void csvFileCallback(std_msgs::msg::String::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onScenario(const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr msg);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // functions
  void run();
  autoware_planning_msgs::msg::Trajectory generateOutputTrajectory(
    autoware_planning_msgs::msg::Trajectory input_trajectory,
    geometry_msgs::msg::PoseStamped start_pose, double distance_threshold, double angle_threshold);
  autoware_planning_msgs::msg::Path convertTrajectoryToPath(
    const autoware_planning_msgs::msg::Trajectory & trajectory);
  void generateDrivableArea(
    autoware_planning_msgs::msg::Path & path, const double vehicle_length, const double offset,
    const bool is_driving_forward);
  void readCsvFile();
  template <class T>
  T waitForParam(
    rclcpp::Node * node, const std::string & remote_node_name, const std::string & param_name);
  bool getEgoVehiclePose(geometry_msgs::msg::PoseStamped * ego_vehicle_pose);
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped * output_pose, const std::string target_frame);
  bool isArrival(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Pose & point);
  bool isVehicleStopped(const geometry_msgs::msg::TwistStamped & twist);
};

#endif  // WAYPOINT_LOADER__WAYPOINT_LOADER_CORE_H_
