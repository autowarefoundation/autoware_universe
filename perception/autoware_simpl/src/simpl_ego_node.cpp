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

#include "autoware/simpl/simpl_ego_node.hpp"

#include "autoware/simpl/conversion/tracked_object.hpp"
#include "autoware/simpl/debug/marker.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/time.hpp>

#include <glog/logging.h>
#include <lanelet2_core/LaneletMap.h>

#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

namespace autoware::simpl
{
namespace
{
/**
 * @brief Convert `Odometry` to `TrackedObject`.
 *
 * @param odometry Odometry.
 */
autoware_perception_msgs::msg::TrackedObject to_tracked_object(
  const nav_msgs::msg::Odometry & odometry)
{
  autoware_perception_msgs::msg::TrackedObject object;

  object.existence_probability = 1.0;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  object.classification.emplace_back(classification);

  object.kinematics.pose_with_covariance = odometry.pose;
  object.kinematics.twist_with_covariance = odometry.twist;

  object.shape.dimensions.x = 0.5;
  object.shape.dimensions.y = 0.5;
  object.shape.dimensions.z = 0.5;

  return object;
}
}  // namespace

SimplEgoNode::SimplEgoNode(const rclcpp::NodeOptions & options) : rclcpp::Node("simpl_ego", options)
{
  google::InitGoogleLogging(get_name());
  google::InstallFailureSignalHandler();

  {
    // Subscriptions and publisher
    using std::placeholders::_1;

    lanelet_subscription_ = create_subscription<LaneletMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&SimplEgoNode::on_map, this, _1));

    objects_publisher_ = create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});
  }

  {
    // Lanelet converter
    lanelet_converter_ptr_ = std::make_unique<conversion::LaneletConverter>();
  }

  {
    // Pre-processor
    const auto label_names = declare_parameter<std::vector<std::string>>("preprocess.labels");
    const auto label_ids = archetype::to_label_ids(label_names);
    const auto max_num_agent = declare_parameter<int>("preprocess.max_num_agent");
    num_past_ = declare_parameter<int>("preprocess.num_past");
    const auto max_num_polyline = declare_parameter<int>("preprocess.max_num_polyline");
    const auto max_num_point = declare_parameter<int>("preprocess.max_num_point");
    const auto polyline_range_distance =
      declare_parameter<double>("preprocess.polyline_range_distance");
    const auto polyline_break_distance =
      declare_parameter<double>("preprocess.polyline_break_distance");

    preprocessor_ = std::make_unique<processing::PreProcessor>(
      label_ids, max_num_agent, num_past_, max_num_polyline, max_num_point, polyline_range_distance,
      polyline_break_distance);
  }

  {
    // Post-processor
    const auto num_mode = declare_parameter<int>("postprocess.num_mode");
    const auto num_future = declare_parameter<int>("postprocess.num_future");
    const auto score_threshold = declare_parameter<double>("postprocess.score_threshold");
    postprocessor_ =
      std::make_unique<processing::PostProcessor>(num_mode, num_future, score_threshold);
  }

  {
    // Detector
    const auto onnx_path = declare_parameter<std::string>("detector.onnx_path");
    const auto precision = declare_parameter<std::string>("detector.precision");
    const auto engine_path = declare_parameter<std::string>("detector.engine_path");
    tensorrt_common::TrtCommonConfig config(onnx_path, precision, engine_path, 1UL << 60U);
    detector_ = std::make_unique<TrtSimpl>(config);
  }

  if (declare_parameter<bool>("build_only")) {
    RCLCPP_INFO(get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }

  {
    using std::chrono_literals::operator""ms;
    timer_ = this->create_wall_timer(100ms, std::bind(&SimplEgoNode::callback, this));
  }

  {
    // processing time
    stopwatch_ptr_ =
      std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
    stopwatch_ptr_->tic("cyclic_time");
    stopwatch_ptr_->tic("processing_time");
    processing_time_publisher_ =
      std::make_unique<autoware_utils_debug::DebugPublisher>(this, get_name());
  }

  {
    // Debug marker publisher
    history_marker_publisher_ =
      this->create_publisher<MarkerArray>("~/debug/histories", rclcpp::QoS{1});
    polyline_marker_publisher_ =
      this->create_publisher<MarkerArray>("~/debug/map_points", rclcpp::QoS{1});
    processed_map_marker_publisher_ =
      this->create_publisher<MarkerArray>("~/debug/processed_map", rclcpp::QoS{1});
  }
}

void SimplEgoNode::callback()
{
  stopwatch_ptr_->toc("processing_time", true);

  const auto current_ego_opt = subscribe_ego();
  if (!current_ego_opt) {
    RCLCPP_WARN(get_logger(), "Failed to subscribe ego vehicle state.");
    return;
  }
  const auto & current_ego = current_ego_opt.value();

  const auto polylines_opt = lanelet_converter_ptr_->polylines();
  if (!polylines_opt) {
    RCLCPP_WARN(get_logger(), "No map points.");
    return;
  }
  const auto & polylines = polylines_opt.value();

  std::vector<archetype::AgentHistory> histories;
  for (const auto & [_, history] : history_map_) {
    histories.emplace_back(history);
  }

  const auto [agent_metadata, map_metadata, rpe_tensor] =
    preprocessor_->process(histories, polylines, current_ego);

  try {
    const auto [scores, trajectories] =
      detector_->do_inference(agent_metadata.tensor, map_metadata.tensor, rpe_tensor).unwrap();

    const auto predicted_objects = postprocessor_->process(
      scores, trajectories, agent_metadata.agent_ids, *current_header_, tracked_object_map_);

    objects_publisher_->publish(predicted_objects);
  } catch (const archetype::SimplException & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Inference failed: " << e.what());
  }

  ////////////// DEBUG //////////////
  {
    // processing time
    const auto cyclic_time_ms = stopwatch_ptr_->toc("cyclic_time", true);
    const auto processing_time_ms = stopwatch_ptr_->toc("processing_time", true);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }

  {
    // debug marker
    const auto history_marker_array =
      debug::create_history_marker_array(history_map_, *current_header_);
    const auto polyline_marker_array =
      debug::create_polyline_marker_array(polylines, *current_header_);
    const auto processed_map_marker_array = debug::create_processed_map_marker_array(
      map_metadata.tensor.polylines, map_metadata.centers, map_metadata.vectors, *current_header_);
    history_marker_publisher_->publish(history_marker_array);
    polyline_marker_publisher_->publish(polyline_marker_array);
    processed_map_marker_publisher_->publish(processed_map_marker_array);
  }
}

void SimplEgoNode::on_map(const LaneletMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);

  lanelet_converter_ptr_->convert(lanelet_map_ptr_);
}

std::optional<archetype::AgentState> SimplEgoNode::subscribe_ego()
{
  const auto odometry_msg = odometry_subscription_.take_data();
  if (!odometry_msg) {
    return std::nullopt;
  }
  // update history
  const auto current_ego = conversion::to_agent_state(*odometry_msg);
  update_history(current_ego);

  // update current header
  current_header_ = odometry_msg->header;

  // update tracked object information
  const auto tracked_object = to_tracked_object(*odometry_msg);
  tracked_object_map_.insert_or_assign(ego_id, tracked_object);

  return current_ego;
}

void SimplEgoNode::update_history(const archetype::AgentState & current_ego) noexcept
{
  if (history_map_.count(ego_id) == 0) {
    archetype::AgentHistory history(ego_id, num_past_, current_ego);
    history_map_.emplace(ego_id, history);
  } else {
    history_map_.at(ego_id).update(current_ego);
  }
}
}  // namespace autoware::simpl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::simpl::SimplEgoNode);
