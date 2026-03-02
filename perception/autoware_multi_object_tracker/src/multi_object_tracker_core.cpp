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

#include "multi_object_tracker_core.hpp"

#include <tf2_ros/create_timer_ros.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

MultiObjectTrackerInternalState::MultiObjectTrackerInternalState()
: last_published_time(0, 0, RCL_ROS_TIME), last_updated_time(0, 0, RCL_ROS_TIME)
{
}

void MultiObjectTrackerInternalState::init(
  const MultiObjectTrackerParameters & params, rclcpp::Node & node,
  const std::function<void()> & trigger_function)
{
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    node.get_node_base_interface(), node.get_node_timers_interface());
  tf_buffer->setCreateTimerInterface(cti);

  odometry = std::make_shared<Odometry>(
    node.get_logger(), node.get_clock(), tf_buffer, params.world_frame_id, params.ego_frame_id,
    params.enable_odometry_uncertainty);

  // Initialize input manager
  input_manager = std::make_unique<InputManager>(odometry, node.get_logger(), node.get_clock());
  input_manager->init(params.input_channels_config);
  input_manager->setTriggerFunction(trigger_function);

  // Initialize processor
  processor = std::make_unique<TrackerProcessor>(
    params.processor_config, params.associator_config, params.input_channels_config);

  last_published_time = node.now();
  last_updated_time = node.now();
}

namespace core
{

std::optional<ObjectsList> get_objects(
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state)
{
  ObjectsList objects_list;
  const bool is_objects_ready = state.input_manager->getObjects(current_time, objects_list);
  if (!is_objects_ready) {
    return std::nullopt;
  }
  return objects_list;
}

void process_parameters(MultiObjectTrackerParameters & params)
{
  using Label = autoware_perception_msgs::msg::ObjectClassification;

  // convert string to TrackerType
  static const std::unordered_map<std::string, TrackerType> TRACKER_TYPE_MAP = {
    {"multi_vehicle_tracker", TrackerType::MULTIPLE_VEHICLE},
    {"pedestrian_and_bicycle_tracker", TrackerType::PEDESTRIAN_AND_BICYCLE},
    {"normal_vehicle_tracker", TrackerType::NORMAL_VEHICLE},
    {"pedestrian_tracker", TrackerType::PEDESTRIAN},
    {"big_vehicle_tracker", TrackerType::BIG_VEHICLE},
    {"bicycle_tracker", TrackerType::BICYCLE},
    {"pass_through_tracker", TrackerType::PASS_THROUGH}};

  auto getTrackerType = [&params](const std::string & tracker_key) -> TrackerType {
    auto tracker_name_it = params.tracker_type_map.find(tracker_key);
    if (tracker_name_it == params.tracker_type_map.end()) {
      return TrackerType::UNKNOWN;
    }
    const std::string & tracker_name = tracker_name_it->second;
    auto it = TRACKER_TYPE_MAP.find(tracker_name);
    return it != TRACKER_TYPE_MAP.end() ? it->second : TrackerType::UNKNOWN;
  };

  // Set the tracker map for processor config
  params.processor_config.tracker_map = {
    {Label::CAR, getTrackerType("car_tracker")},
    {Label::TRUCK, getTrackerType("truck_tracker")},
    {Label::BUS, getTrackerType("bus_tracker")},
    {Label::TRAILER, getTrackerType("trailer_tracker")},
    {Label::PEDESTRIAN, getTrackerType("pedestrian_tracker")},
    {Label::BICYCLE, getTrackerType("bicycle_tracker")},
    {Label::MOTORCYCLE, getTrackerType("motorcycle_tracker")},
    {Label::UNKNOWN, TrackerType::UNKNOWN}};
  // Set the pruning thresholds for processor config
  for (size_t i = 0; i < params.pruning_giou_thresholds.size(); ++i) {
    const auto label = static_cast<LabelType>(i);
    params.processor_config.pruning_giou_thresholds[label] = params.pruning_giou_thresholds.at(i);
  }
  for (size_t i = 0; i < params.pruning_distance_thresholds.size(); ++i) {
    const auto label = static_cast<LabelType>(i);
    params.processor_config.pruning_distance_thresholds[label] =
      params.pruning_distance_thresholds.at(i);
  }

  // Initialize association matrices
  auto initializeMatrixInt = [](const std::vector<int64_t> & vector) {
    const int label_num = types::NUM_LABELS;
    if (vector.size() != label_num * label_num) {
      throw std::runtime_error("Invalid can_assign_matrix size");
    }
    std::vector<int> converted_vector(vector.begin(), vector.end());
    // Use row-major mapping to match the YAML layout
    using RowMajorMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<RowMajorMatrixXi> matrix_tmp(converted_vector.data(), label_num, label_num);

    // Convert to column-major (Eigen's default) for consistency
    return Eigen::MatrixXi(matrix_tmp);
  };
  auto initializeMatrixDouble = [](const std::vector<double> & vector) {
    const int label_num = types::NUM_LABELS;
    if (vector.size() != label_num * label_num) {
      throw std::runtime_error("Invalid association matrix configuration size");
    }
    // Use row-major mapping to match the YAML layout
    using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<const RowMajorMatrixXd> matrix_tmp(vector.data(), label_num, label_num);

    // Convert to column-major (Eigen's default) for consistency
    return Eigen::MatrixXd(matrix_tmp);
  };
  Eigen::MatrixXi can_assign_matrix = initializeMatrixInt(params.can_assign_matrix);
  params.associator_config.max_dist_matrix = initializeMatrixDouble(params.max_dist_matrix);
  params.associator_config.max_area_matrix = initializeMatrixDouble(params.max_area_matrix);
  params.associator_config.min_area_matrix = initializeMatrixDouble(params.min_area_matrix);
  params.associator_config.min_iou_matrix = initializeMatrixDouble(params.min_iou_matrix);

  // pre-process
  const int label_num = params.associator_config.max_dist_matrix.rows();
  for (int i = 0; i < label_num; i++) {
    for (int j = 0; j < label_num; j++) {
      params.associator_config.max_dist_matrix(i, j) =
        params.associator_config.max_dist_matrix(i, j) *
        params.associator_config.max_dist_matrix(i, j);
    }
  }

  // Set the tracker map for associator config
  params.associator_config.can_assign_map.clear();
  for (const auto & [label, tracker_type] : params.processor_config.tracker_map) {
    params.associator_config.can_assign_map[tracker_type].fill(false);
  }
  // can_assign_map : tracker_type that can be assigned to each measurement label
  // relationship is given by tracker_map and can_assign_matrix
  for (int i = 0; i < can_assign_matrix.rows(); ++i) {
    for (int j = 0; j < can_assign_matrix.cols(); ++j) {
      if (can_assign_matrix(i, j) == 1) {
        const auto tracker_type = params.processor_config.tracker_map.at(i);
        params.associator_config.can_assign_map[tracker_type][j] = true;
      }
    }
  }
}

void process_objects(
  const types::DynamicObjectList & objects, const rclcpp::Time & current_time,
  [[maybe_unused]] const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state, TrackerDebugger & debugger,
  const rclcpp::Logger & logger)
{
  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    rclcpp::Time(objects.header.stamp, current_time.get_clock_type());

  std::optional<geometry_msgs::msg::Pose> ego_pose;
  if (const auto odometry_info = state.odometry->getOdometryFromTf(measurement_time)) {
    ego_pose = odometry_info->pose.pose;
  }

  if (!ego_pose) {
    RCLCPP_WARN(
      logger, "No odometry information available at the measurement time: %.9f",
      measurement_time.seconds());
  }

  /* predict trackers to the measurement time */
  state.processor->predict(measurement_time, ego_pose);

  /* object association */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  state.processor->associate(objects, direct_assignment, reverse_assignment);

  // Collect debug information - tracker list, existence probabilities, association results
  debugger.collectObjectInfo(
    measurement_time, state.processor->getListTracker(), objects, direct_assignment,
    reverse_assignment);

  /* tracker update */
  state.processor->update(objects, direct_assignment);

  /* tracker pruning */
  state.processor->prune(measurement_time);

  /* spawn new tracker */
  state.processor->spawn(objects, reverse_assignment);

  state.last_updated_time = current_time;
}

bool should_publish(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state)
{
  if (state.last_updated_time.nanoseconds() == 0) {
    state.last_updated_time = current_time;
  }

  // ensure minimum interval: room for the next process(prediction)
  static constexpr double minimum_publish_interval_ratio = 0.85;
  static constexpr double maximum_publish_interval_ratio = 1.05;

  const double publisher_period = 1.0 / params.publish_rate;
  const double minimum_publish_interval = publisher_period * minimum_publish_interval_ratio;
  const auto elapsed_time = (current_time - state.last_published_time).seconds();

  if (elapsed_time < minimum_publish_interval) {
    return false;
  }

  // if there was update after publishing, publish new messages
  bool should_publish = state.last_published_time < state.last_updated_time;

  // if there was no update, publish if the elapsed time is longer than the maximum publish latency
  // in this case, it will perform extrapolate/remove old objects
  const double maximum_publish_interval = publisher_period * maximum_publish_interval_ratio;
  should_publish = should_publish || elapsed_time > maximum_publish_interval;

  return should_publish;
}

void prune_objects(const rclcpp::Time & time, MultiObjectTrackerInternalState & state)
{
  state.processor->prune(time);
}

autoware_perception_msgs::msg::TrackedObjects get_tracked_objects(
  const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params, MultiObjectTrackerInternalState & state)
{
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  tracked_objects.header.frame_id = params.world_frame_id;
  const rclcpp::Time object_time = params.enable_delay_compensation ? current_time : publish_time;
  state.processor->getTrackedObjects(object_time, tracked_objects);

  state.last_published_time = current_time;

  return tracked_objects;
}

std::optional<autoware_perception_msgs::msg::DetectedObjects> get_merged_objects(
  const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params, MultiObjectTrackerInternalState & state,
  const rclcpp::Logger & logger)
{
  if (!params.publish_merged_objects) {
    return std::nullopt;
  }

  const rclcpp::Time object_time = params.enable_delay_compensation ? current_time : publish_time;
  const auto tf_base_to_world = state.odometry->getTransform(publish_time);
  if (tf_base_to_world) {
    autoware_perception_msgs::msg::DetectedObjects merged_output_msg;
    state.processor->getMergedObjects(object_time, *tf_base_to_world, merged_output_msg);
    merged_output_msg.header.frame_id = params.ego_frame_id;
    return merged_output_msg;
  }

  RCLCPP_WARN(
    logger, "No odometry information available at the publishing time: %.9f",
    publish_time.seconds());
  return std::nullopt;
}

}  // namespace core
}  // namespace autoware::multi_object_tracker
