// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware::pure_pursuit
{
namespace
{
void publish_processing_time(
  const rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr & publisher,
  autoware_utils::StopWatch<std::chrono::milliseconds> & stop_watch, rclcpp::Clock & clock)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time;
  processing_time.stamp = clock.now();
  processing_time.data = stop_watch.toc();
  publisher->publish(processing_time);
}
}  // namespace

PurePursuitLateralControllerNode::PurePursuitLateralControllerNode(rclcpp::Node & node)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  writer_(logger_, clock_),
  controller_(std::make_unique<PurePursuitLateralController>(createConfig(node), writer_))
{
  // Debug Publishers
  pub_debug_values_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
      "~/debug/ld_outputs", rclcpp::QoS{1});

  // Publish predicted trajectory
  pub_predicted_trajectory_ = node.create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/predicted_trajectory", 1);

  pub_processing_time_ms_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/lateral/debug/processing_time_ms", 1);
}

PurePursuitLateralControllerConfig PurePursuitLateralControllerNode::createConfig(
  rclcpp::Node & node)
{
  PurePursuitLateralControllerConfig config{};

  // Vehicle Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  config.wheel_base = vehicle_info.wheel_base_m;
  config.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  config.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  config.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  config.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  config.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  config.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  config.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  config.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  config.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
  config.prediction_ds = node.declare_parameter<double>("prediction_ds");
  config.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  config.resampling_ds = node.declare_parameter<double>("resampling_ds");
  config.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  config.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  config.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");
  return config;
}

tl::expected<LateralOutput, std::string> PurePursuitLateralControllerNode::run(
  const InputData & input_data, const rclcpp::Time & stamp)
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  const auto result = controller_->runDetailed(input_data, stamp);

  if (result.lookahead_debug_values) {
    pub_debug_values_->publish(*result.lookahead_debug_values);
  }
  if (result.predicted_trajectory) {
    pub_predicted_trajectory_->publish(*result.predicted_trajectory);
  }

  publish_processing_time(pub_processing_time_ms_, stop_watch, *clock_);

  return result.output;
}
}  // namespace autoware::pure_pursuit
