// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/simpl_node.hpp"

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/exception.hpp"
#include "autoware/simpl/conversion/tracked_object.hpp"

#include <autoware/tensorrt_common/utils.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::simpl
{
// TODO(ktro2828): use a parameter
using TrackedObject = SimplNode::TrackedObject;

SimplNode::SimplNode(const rclcpp::NodeOptions & options) : rclcpp::Node("simpl_node", options)
{
  {  // Subscriptions and publisher
    using std::placeholders::_1;

    objects_subscription_ = create_subscription<TrackedObjects>(
      "~/input/objects", rclcpp::QoS{1}, std::bind(&SimplNode::callback, this, _1));

    lanelet_subscription_ = create_subscription<LaneletMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&SimplNode::on_map, this, _1));

    objects_publisher_ = create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});
  }

  {
    // Pre-processor
    const auto label_names = declare_parameter<std::vector<std::string>>("processing.labels");
    const auto label_ids = archetype::to_label_ids(label_names);
    const auto max_num_agent = declare_parameter<int>("processing.max_num_agent");
    num_past_ = declare_parameter<int>("processing.num_past");
    const auto max_num_polyline = declare_parameter<int>("processing.max_num_polyline");
    const auto max_num_point = declare_parameter<int>("processing.max_num_point");
    const auto break_distance = declare_parameter<double>("processing.polyline_break_distance");

    preprocessor_ = std::make_unique<processing::PreProcessor>(
      label_ids, max_num_agent, num_past_, max_num_polyline, max_num_point, break_distance);

    // Post-processor
    const auto num_mode = declare_parameter<int>("processing.num_mode");
    const auto num_future = declare_parameter<int>("processing.num_future");
    postprocessor_ = std::make_unique<processing::PostProcessor>(num_mode, num_future);

    time_threshold_ = declare_parameter<double>("processing.time_threshold");
    polyline_distance_threshold_ =
      declare_parameter<double>("processing.polyline_distance_threshold");
  }

  {
    // Detector
    const auto onnx_path = declare_parameter<std::string>("detector.onnx_path");
    tensorrt_common::TrtCommonConfig config(onnx_path, "fp32");
    detector_ = std::make_unique<TrtSimpl>(config);
  }

  if (declare_parameter<bool>("build_only")) {
    RCLCPP_INFO(get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void SimplNode::callback(const TrackedObjects::ConstSharedPtr objects_msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Subscribe: " << objects_msg->header.frame_id);

  // 1. Retrieve the latest ego state.
  const auto current_ego = subscribe_ego();
  if (!current_ego) {
    RCLCPP_WARN(get_logger(), "Failed to subscribe ego vehicle state.");
    return;
  }

  // 2. Convert lanelet to points extracting points closer than threshold from ego vehicle.
  const auto map_points =
    lanelet_converter_ptr_->convert(current_ego.value(), polyline_distance_threshold_);
  if (!map_points) {
    RCLCPP_WARN(get_logger(), "No map points.");
    return;
  }

  const auto current_time = rclcpp::Time(objects_msg->header.stamp).seconds();
  remove_ancient_history(current_time, objects_msg);
  update_history(current_time, objects_msg);

  std::vector<archetype::AgentHistory> histories;
  std::vector<archetype::AgentState> current_states;
  for (const auto & [agent_id, history] : history_map_) {
    histories.emplace_back(history);
    current_states.emplace_back(history.current());
  }

  // Execute preprocessing
  const auto & [agent_tensor, map_tensor, rpe_tensor] =
    preprocessor_->process(histories, map_points.value(), current_ego.value());

  // Inference
  std::vector<float> scores, trajectories;
  try {
    auto [scores, trajectories] =
      detector_->do_inference(agent_tensor, map_tensor, rpe_tensor).unwrap();
  } catch (const archetype::SimplException & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Inference failed: " << e.what());
    return;
  }

  // Execute postprocessing
  const auto predictions = postprocessor_->process(scores, trajectories, current_states);
}

void SimplNode::on_map(const LaneletMapBin::ConstSharedPtr map_msg)
{
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);

  lanelet_converter_ptr_ = std::make_unique<conversion::LaneletConverter>(lanelet_map_ptr_);
}

std::optional<archetype::AgentState> SimplNode::subscribe_ego()
{
  const auto odometry_msg = odometry_subscription_->take_data();
  if (!odometry_msg) {
    return std::nullopt;
  }
  return conversion::to_state(*odometry_msg, true);
}

void SimplNode::remove_ancient_history(
  double current_time, const TrackedObjects::ConstSharedPtr objects_msg) noexcept
{
  for (const auto & object : objects_msg->objects) {
    const auto agent_id = autoware_utils::to_hex_string(object.object_id);
    if (history_map_.count(agent_id) == 0) {
      continue;
    }

    const auto & history = history_map_.at(agent_id);
    if (history.is_ancient(current_time, time_threshold_)) {
      history_map_.erase(agent_id);
    }
  }
}

void SimplNode::update_history(
  double current_time, const TrackedObjects::ConstSharedPtr objects_msg) noexcept
{
  std::vector<std::string> observed_ids;

  constexpr bool is_valid = true;
  // Update agent history
  {
    for (const auto & object : objects_msg->objects) {
      const auto agent_id = autoware_utils::to_hex_string(object.object_id);
      observed_ids.emplace_back(agent_id);

      const auto state = conversion::to_state(object, is_valid);
      if (history_map_.count(agent_id) == 0) {
        archetype::AgentHistory history(agent_id, num_past_, current_time, state);
        history_map_.emplace(agent_id, history);
      } else {
        history_map_.at(agent_id).update(current_time, state);
      }
    }
  }

  // Update unobserved histories with empty
  for (auto & [agent_id, history] : history_map_) {
    if (std::find(observed_ids.begin(), observed_ids.end(), agent_id) != observed_ids.end()) {
      continue;
    }
    history.update();
  }
}
}  // namespace autoware::simpl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::simpl::SimplNode);
