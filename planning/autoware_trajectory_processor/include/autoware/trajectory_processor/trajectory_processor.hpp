// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_HPP_

#include "autoware/trajectory_processor/trajectory_processor_context.hpp"
#include "autoware/trajectory_processor/trajectory_processor_data.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <autoware_trajectory_processor/trajectory_processor_param.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_processor
{

/// @brief Runs the complete ordered modifier and optimizer plugin pipeline.
class TrajectoryProcessor : public rclcpp::Node
{
public:
  /// @brief Construct the processor node and load its configured plugin pipeline.
  explicit TrajectoryProcessor(const rclcpp::NodeOptions & options);

private:
  using CandidateTrajectories = autoware_internal_planning_msgs::msg::CandidateTrajectories;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using Odometry = nav_msgs::msg::Odometry;
  using Acceleration = geometry_msgs::msg::AccelWithCovarianceStamped;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Plugin = plugin::TrajectoryProcessorPluginBase;

  /// @brief Process every candidate through the configured plugin sequence.
  void on_trajectories(const CandidateTrajectories::ConstSharedPtr msg);
  /// @brief Store the latest lanelet map used by map-dependent plugins.
  void on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  /// @brief Take one snapshot of all polling inputs for the current callback.
  tl::expected<TrajectoryProcessorData, std::string> make_input_data();
  /// @brief Load every configured plugin in pipeline order.
  void load_plugins();
  /// @brief Create and initialize one plugin instance.
  void load_plugin(const std::string & class_name, std::size_t pipeline_index);
  /// @brief Apply changed parameters and reload the pipeline if its order changed.
  void update_params();
  /// @brief Publish the total callback processing duration.
  void publish_processing_time(double processing_time_ms);

  std::unique_ptr<trajectory_processor_params::ParamListener> param_listener_;
  TrajectoryProcessorParams params_;

  pluginlib::ClassLoader<Plugin> plugin_loader_;
  std::vector<std::shared_ptr<Plugin>> plugins_;
  std::shared_ptr<TrajectoryProcessorContext> context_;

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr trajectories_sub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr trajectories_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Acceleration> sub_current_acceleration_{
    this, "~/input/acceleration"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PointCloud2> sub_pointcloud_{
    this, "~/input/pointcloud", autoware_utils_rclcpp::single_depth_sensor_qos()};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_lights_{this, "~/input/traffic_signals"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware_utils_rclcpp::polling_policy::Latest>
    sub_route_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  std::shared_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_HPP_
