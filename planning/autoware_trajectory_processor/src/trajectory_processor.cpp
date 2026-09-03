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

#include "autoware/trajectory_processor/trajectory_processor.hpp"

#include "autoware/trajectory_processor/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware::trajectory_processor
{

TrajectoryProcessor::TrajectoryProcessor(const rclcpp::NodeOptions & options)
: Node{"trajectory_processor", options},
  param_listener_{
    std::make_unique<trajectory_processor_params::ParamListener>(get_node_parameters_interface())},
  params_{param_listener_->get_params()},
  plugin_loader_{
    "autoware_trajectory_processor",
    "autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase"},
  context_{std::make_shared<TrajectoryProcessorContext>(this)}
{
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryProcessor::on_map, this, std::placeholders::_1));
  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryProcessor::on_trajectories, this, std::placeholders::_1));
  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);

  sub_current_odometry_ =
    namespace_polling::create_polling_subscriber<Odometry>(this, "~/input/odometry");
  sub_current_acceleration_ =
    namespace_polling::create_polling_subscriber<Acceleration>(this, "~/input/acceleration");
  sub_objects_ =
    namespace_polling::create_polling_subscriber<PredictedObjects>(this, "~/input/objects");
  sub_pointcloud_ = namespace_polling::create_polling_subscriber<PointCloud2>(
    this, "~/input/pointcloud", autoware_utils_rclcpp::single_depth_sensor_qos());
  sub_traffic_lights_ = namespace_polling::create_polling_subscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>(this, "~/input/traffic_signals");
  sub_route_ =
    namespace_polling::create_polling_subscriber<autoware_planning_msgs::msg::LaneletRoute>(
      this, "~/input/route", rclcpp::QoS{1}.transient_local());

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  debug_publisher_ =
    std::make_shared<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>(
      this, "~/debug");
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  load_plugins();
  RCLCPP_INFO(get_logger(), "TrajectoryProcessor initialized with %zu plugins", plugins_.size());
}

void TrajectoryProcessor::on_map(
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void TrajectoryProcessor::load_plugins()
{
  plugins_.clear();
  for (std::size_t index = 0; index < params_.plugin_names.size(); ++index) {
    const auto & class_name = params_.plugin_names.at(index);
    if (!class_name.empty()) {
      load_plugin(class_name, index);
    }
  }
}

void TrajectoryProcessor::load_plugin(
  const std::string & class_name, const std::size_t pipeline_index)
{
  try {
    auto processor_plugin = plugin_loader_.createSharedInstance(class_name);
    const auto instance_name = class_name + "#" + std::to_string(pipeline_index);
    processor_plugin->initialize(class_name, instance_name, this, time_keeper_, context_, params_);
    plugins_.push_back(std::move(processor_plugin));
    RCLCPP_INFO(
      get_logger(), "Loaded trajectory processor plugin '%s' as '%s'", class_name.c_str(),
      instance_name.c_str());
  } catch (const pluginlib::PluginlibException & error) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load trajectory processor plugin '%s': %s", class_name.c_str(),
      error.what());
  }
}

void TrajectoryProcessor::update_params()
{
  try {
    auto updated_params = param_listener_->get_params();
    const bool pipeline_changed = updated_params.plugin_names != params_.plugin_names;
    params_ = std::move(updated_params);
    if (pipeline_changed) {
      load_plugins();
      return;
    }
    for (auto & processor_plugin : plugins_) {
      processor_plugin->update_params(params_);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "Failed to update parameters: %s", error.what());
  }
}

tl::expected<TrajectoryProcessorData, std::string> TrajectoryProcessor::make_input_data()
{
  TrajectoryProcessorData data;
  data.current_odometry = sub_current_odometry_->take_data();
  data.current_acceleration = sub_current_acceleration_->take_data();
  data.predicted_objects = sub_objects_->take_data();
  data.obstacle_pointcloud = sub_pointcloud_->take_data();
  data.route = sub_route_->take_data();
  data.traffic_light_signals = sub_traffic_lights_->take_data();
  data.lanelet_map = lanelet_map_ptr_;

  if (!data.current_odometry || !data.current_acceleration) {
    return tl::make_unexpected("Data is not ready: odometry or acceleration is not set");
  }
  if (!data.predicted_objects) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: predicted_objects is not set");
  }
  if (!data.obstacle_pointcloud) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: obstacle_pointcloud is not set");
  }
  if (!data.route) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: route is not set");
  }
  if (!data.traffic_light_signals) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: traffic_light_signals is not set");
  }
  if (!data.lanelet_map) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: lanelet_map is not set");
  }
  return data;
}

void TrajectoryProcessor::publish_processing_time(const double processing_time_ms)
{
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "processing_time_ms", processing_time_ms);
}

void TrajectoryProcessor::on_trajectories(AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories)
                                            msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(__func__);

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  const auto input = make_input_data();
  if (!input) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", input.error().c_str());
    publish_processing_time(stop_watch.toc(__func__));
    return;
  }

  CandidateTrajectories output = *msg;
  std::string modified_instances;
  for (std::size_t candidate_index = 0; candidate_index < output.candidate_trajectories.size();
       ++candidate_index) {
    auto & candidate = output.candidate_trajectories.at(candidate_index);
    auto data = input.value();
    for (auto & processor_plugin : plugins_) {
      const auto result = processor_plugin->process(candidate.points, data);
      processor_plugin->publish_debug_data("trajectory_" + std::to_string(candidate_index));
      if (result != plugin::ProcessingResult::Modified) continue;
      processor_plugin->publish_planning_factor();
      if (!modified_instances.empty()) {
        modified_instances += ", ";
      }
      modified_instances += processor_plugin->get_short_name();
    }
    if (candidate.points.size() < 3) {
      candidate.points =
        autoware::trajectory_processor::utils::generate_three_point_stopped_trajectory(
          candidate.points, *data.current_odometry);
    }
  }

  if (!modified_instances.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "Trajectory was modified by %s",
      modified_instances.c_str());
  }

  trajectories_pub_->publish(output);
  if (output.candidate_trajectories.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received an empty candidate trajectory array; skipping selected trajectory output");
    publish_processing_time(stop_watch.toc(__func__));
    return;
  }

  Trajectory selected;
  selected.header = output.candidate_trajectories.front().header;
  selected.points = output.candidate_trajectories.front().points;
  trajectory_pub_->publish(selected);
  publish_processing_time(stop_watch.toc(__func__));
}

}  // namespace autoware::trajectory_processor

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_processor::TrajectoryProcessor)
