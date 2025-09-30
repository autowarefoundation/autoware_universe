// Copyright 2025 TIER IV, inc.
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
#include "test_bench.hpp"

#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Configuration creation functions
autoware::multi_object_tracker::TrackerProcessorConfig createProcessorConfig()
{
  autoware::multi_object_tracker::TrackerProcessorConfig config;
  using autoware::multi_object_tracker::TrackerType;
  using autoware_perception_msgs::msg::ObjectClassification;

  // Set tracker types for different object classes
  config.tracker_map = {
    {ObjectClassification::UNKNOWN, TrackerType::UNKNOWN},
    {ObjectClassification::CAR, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::TRUCK, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::BUS, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::TRAILER, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::PEDESTRIAN, TrackerType::PEDESTRIAN_AND_BICYCLE},
    {ObjectClassification::BICYCLE, TrackerType::PEDESTRIAN_AND_BICYCLE},
    {ObjectClassification::MOTORCYCLE, TrackerType::PEDESTRIAN_AND_BICYCLE}};

  // Set tracker lifetime and removal thresholds (from multi_object_tracker_node.param.yaml)
  config.tracker_lifetime = 1.0;                  // [s]
  config.min_known_object_removal_iou = 0.1;      // [ratio]
  config.min_unknown_object_removal_iou = 0.001;  // [ratio]

  // Set confident count thresholds for different object classes (from
  // multi_object_tracker_node.param.yaml)
  std::map<std::string, ObjectClassification::_label_type> class_name_to_label = {
    {"UNKNOWN", ObjectClassification::UNKNOWN}, {"CAR", ObjectClassification::CAR},
    {"TRUCK", ObjectClassification::TRUCK},     {"BUS", ObjectClassification::BUS},
    {"TRAILER", ObjectClassification::TRAILER}, {"MOTORBIKE", ObjectClassification::MOTORCYCLE},
    {"BICYCLE", ObjectClassification::BICYCLE}, {"PEDESTRIAN", ObjectClassification::PEDESTRIAN}};

  // Generalized IoU threshold for each class
  config.pruning_giou_thresholds = {
    {ObjectClassification::UNKNOWN, -0.3}, {ObjectClassification::CAR, -0.4},
    {ObjectClassification::TRUCK, -0.6},   {ObjectClassification::BUS, -0.6},
    {ObjectClassification::TRAILER, -0.6}, {ObjectClassification::MOTORCYCLE, -0.1},
    {ObjectClassification::BICYCLE, -0.1}, {ObjectClassification::PEDESTRIAN, -0.1}};

  config.pruning_moving_object_speed = 5.5;   // [m/s]
  config.pruning_static_object_speed = 1.38;  // [m/s]
  config.pruning_static_iou_threshold = 0.0;  // [ratio]
  // overlap distance threshold for each class
  config.pruning_distance_thresholds = {
    {ObjectClassification::UNKNOWN, 9.0}, {ObjectClassification::CAR, 5.0},
    {ObjectClassification::TRUCK, 9.0},   {ObjectClassification::BUS, 9.0},
    {ObjectClassification::TRAILER, 9.0}, {ObjectClassification::MOTORCYCLE, 4.0},
    {ObjectClassification::BICYCLE, 3.0}, {ObjectClassification::PEDESTRIAN, 2.0}};

  return config;
}

autoware::multi_object_tracker::AssociatorConfig createAssociatorConfig()
{
  autoware::multi_object_tracker::AssociatorConfig config;
  constexpr int label_num =
    autoware::multi_object_tracker::types::NUM_LABELS;  // Number of object classes
  using autoware::multi_object_tracker::TrackerType;
  using autoware_perception_msgs::msg::ObjectClassification;

  // Initialize matrices with values from data_association_matrix.param.yaml
  // For a 8x8 matrix (8 object classes: UNKNOWN, CAR, TRUCK, BUS, TRAILER, MOTORCYCLE, BICYCLE,
  // PEDESTRIAN)
  std::map<ObjectClassification::_label_type, TrackerType> tracker_map = {
    {ObjectClassification::UNKNOWN, TrackerType::UNKNOWN},
    {ObjectClassification::CAR, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::TRUCK, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::BUS, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::TRAILER, TrackerType::MULTIPLE_VEHICLE},
    {ObjectClassification::PEDESTRIAN, TrackerType::PEDESTRIAN_AND_BICYCLE},
    {ObjectClassification::BICYCLE, TrackerType::PEDESTRIAN_AND_BICYCLE},
    {ObjectClassification::MOTORCYCLE, TrackerType::PEDESTRIAN_AND_BICYCLE}};

  // Initialize can_assign_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXi can_assign_matrix(label_num, label_num);
  // 8x8 matrix for can_assign relationships
  can_assign_matrix << 1, 0, 0, 0, 0, 0, 0, 0,  // UNKNOWN
    0, 1, 1, 1, 1, 0, 0, 0,                     // CAR
    0, 1, 1, 1, 1, 0, 0, 0,                     // TRUCK
    0, 1, 1, 1, 1, 0, 0, 0,                     // BUS
    0, 1, 1, 1, 1, 0, 0, 0,                     // TRAILER
    0, 0, 0, 0, 0, 1, 1, 1,                     // MOTORBIKE
    0, 0, 0, 0, 0, 1, 1, 1,                     // BICYCLE
    0, 0, 0, 0, 0, 1, 1, 1;                     // PEDESTRIAN

  config.can_assign_map.clear();
  for (const auto & [label, tracker_type] : tracker_map) {
    config.can_assign_map[tracker_type].fill(false);
  }

  // can_assign_map : tracker_type that can be assigned to each measurement label
  // relationship is given by tracker_map and can_assign_matrix
  for (int i = 0; i < can_assign_matrix.rows(); ++i) {
    for (int j = 0; j < can_assign_matrix.cols(); ++j) {
      if (can_assign_matrix(i, j) == 1) {
        const auto tracker_type = tracker_map.at(i);
        config.can_assign_map[tracker_type][j] = true;
      }
    }
  }

  // Initialize max_dist_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXd max_dist_matrix(label_num, label_num);
  max_dist_matrix << 4.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  // UNKNOWN
    4.0, 2.5, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0,                   // CAR
    4.0, 2.5, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0,                   // TRUCK
    4.0, 2.5, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0,                   // BUS
    4.0, 2.5, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0,                   // TRAILER
    3.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 2.0,                   // MOTORCYCLE
    3.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 2.0,                   // BICYCLE
    2.0, 1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 2.0;                   // PEDESTRIAN
  config.max_dist_matrix = max_dist_matrix;

  // Initialize max_area_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXd max_area_matrix(label_num, label_num);
  max_area_matrix << 100.00, 100.00, 100.00, 100.00, 100.00, 100.00, 100.00, 100.00, 12.10, 12.10,
    36.00, 60.00, 60.00, 10000.00, 10000.00, 10000.00, 36.00, 12.10, 36.00, 60.00, 60.00, 10000.00,
    10000.00, 10000.00, 60.00, 12.10, 36.00, 60.00, 60.00, 10000.00, 10000.00, 10000.00, 60.00,
    12.10, 36.00, 60.00, 60.00, 10000.00, 10000.00, 10000.00, 2.50, 10000.00, 10000.00, 10000.00,
    10000.00, 2.50, 2.50, 2.50, 2.50, 10000.00, 10000.00, 10000.00, 10000.00, 2.50, 2.50, 2.50,
    2.00, 10000.00, 10000.00, 10000.00, 10000.00, 2.00, 2.00, 2.00;
  config.max_area_matrix = max_area_matrix;

  // Initialize min_area_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXd min_area_matrix(label_num, label_num);
  min_area_matrix << 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 3.600, 3.600, 4.200,
    10.000, 10.000, 0.000, 0.000, 0.000, 4.200, 3.600, 4.200, 10.000, 10.000, 0.000, 0.000, 0.000,
    10.000, 3.600, 4.200, 10.000, 10.000, 0.000, 0.000, 0.000, 10.000, 3.600, 4.200, 10.000, 10.000,
    0.000, 0.000, 0.000, 0.001, 0.000, 0.000, 0.000, 0.000, 0.100, 0.100, 0.100, 0.001, 0.000,
    0.000, 0.000, 0.000, 0.100, 0.100, 0.100, 0.001, 0.000, 0.000, 0.000, 0.000, 0.100, 0.100,
    0.100;
  config.min_area_matrix = min_area_matrix;

  // Initialize max_rad_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXd max_rad_matrix(label_num, label_num);
  max_rad_matrix << 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 1.047, 1.047,
    1.047, 1.047, 3.150, 3.150, 3.150, 3.150, 1.047, 1.047, 1.047, 1.047, 3.150, 3.150, 3.150,
    3.150, 1.047, 1.047, 1.047, 1.047, 3.150, 3.150, 3.150, 3.150, 1.047, 1.047, 1.047, 1.047,
    3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150,
    3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150, 3.150,
    3.150;
  config.max_rad_matrix = max_rad_matrix;

  // Initialize min_iou_matrix (8x8) from data_association_matrix.param.yaml
  Eigen::MatrixXd min_iou_matrix(label_num, label_num);
  min_iou_matrix << 0.0001, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1, 0.1,
    0.1, 0.1, 0.2, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1, 0.1, 0.2, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1, 0.1, 0.2,
    0.3, 0.3, 0.3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0001;
  config.min_iou_matrix = min_iou_matrix;

  // Pre-process matrices
  config.max_rad_matrix = config.max_rad_matrix.cwiseAbs();
  config.max_dist_matrix = config.max_dist_matrix.array().square();

  config.unknown_association_giou_threshold =
    -0.8;  // Default GIoU threshold for unknown-unknown association

  return config;
}

std::vector<autoware::multi_object_tracker::types::InputChannel> createInputChannelsConfig()
{
  // Create input channel configuration based on input_channels.param.yaml
  // Using lidar_centerpoint as the primary input channel
  autoware::multi_object_tracker::types::InputChannel input_channel_config;
  input_channel_config.index = 0;
  input_channel_config.input_topic = "/perception/object_recognition/detection/centerpoint/objects";
  input_channel_config.long_name = "centerpoint";
  input_channel_config.short_name = "Lcp";
  input_channel_config.is_spawn_enabled = true;
  input_channel_config.trust_existence_probability = true;
  input_channel_config.trust_extension = true;
  input_channel_config.trust_classification = true;
  input_channel_config.trust_orientation = true;

  return {input_channel_config};
}

bool isOverlapping(const ObjectState & obj1, const ObjectState & obj2)
{
  const float obj1_half_x = obj1.shape.x / 2.0f;
  const float obj1_half_y = obj1.shape.y / 2.0f;
  const float obj2_half_x = obj2.shape.x / 2.0f;
  const float obj2_half_y = obj2.shape.y / 2.0f;

  const float obj1_left = obj1.pose.position.x - obj1_half_x;
  const float obj1_right = obj1.pose.position.x + obj1_half_x;
  const float obj1_top = obj1.pose.position.y + obj1_half_y;
  const float obj1_bottom = obj1.pose.position.y - obj1_half_y;

  const float obj2_left = obj2.pose.position.x - obj2_half_x;
  const float obj2_right = obj2.pose.position.x + obj2_half_x;
  const float obj2_top = obj2.pose.position.y + obj2_half_y;
  const float obj2_bottom = obj2.pose.position.y - obj2_half_y;

  return (obj1_right > obj2_left) && (obj1_left < obj2_right) && (obj1_top > obj2_bottom) &&
         (obj1_bottom < obj2_top);
}

uint64_t TestBench::getGridKey(float x, float y) const
{
  return (static_cast<uint64_t>(x / GRID_SIZE) << 32) | static_cast<uint32_t>(y / GRID_SIZE);
}

void TestBench::updateGrid(const std::string & id, bool remove_first)
{
  // Map to store the previous cell key for each object ID
  static std::unordered_map<std::string, uint64_t> previous_cell_keys;
  // Retrieve the current car state associated with the given ID.
  auto & car = car_states_[id];
  // Compute the grid key based on the car's (x, y) position.
  auto key = getGridKey(car.pose.position.x, car.pose.position.y);

  if (remove_first) {
    // Remove the ID from the previous cell directly
    if (previous_cell_keys.count(id)) {
      auto prev_key = previous_cell_keys[id];
      auto & ids = spatial_grid_[prev_key];
      ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
    }
  }
  // Add the ID to the list of objects in the computed grid cell.
  spatial_grid_[key].push_back(id);
  // Update the previous cell key
  previous_cell_keys[id] = key;
}

bool TestBench::checkCollisions(const std::string & id)
{
  auto & car = car_states_[id];
  auto center_key = getGridKey(car.pose.position.x, car.pose.position.y);

  // Check 3x3 grid around current position
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      uint64_t key = center_key + (static_cast<uint64_t>(dx) << 32) + static_cast<uint32_t>(dy);
      for (const auto & other_id : spatial_grid_[key]) {
        if (other_id != id && isOverlapping(car, car_states_[other_id])) {
          return true;
        }
      }
    }
  }
  return false;
}

bool isConvex(const std::vector<geometry_msgs::msg::Point> & polygon)
{
  // A polygon must have at least 3 vertices to be convex
  constexpr size_t min_polygon_vertices = 3;
  if (polygon.size() < min_polygon_vertices) return false;

  bool sign = false;
  size_t n = polygon.size();

  for (size_t i = 0; i < n; ++i) {
    const auto & p0 = polygon[i];
    const auto & p1 = polygon[(i + 1) % n];
    const auto & p2 = polygon[(i + 2) % n];
    // Calculate the cross product to determine the orientation
    double cross = (p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x);

    if (i == 0) {
      sign = cross > 0;
    } else if ((cross > 0) != sign) {
      return false;
    }
  }
  return true;
}

void TestBench::initializeDetectionHeader(
  autoware::multi_object_tracker::types::DynamicObjectList & detections, const rclcpp::Time & stamp)
{
  detections.header.stamp = stamp;
  detections.header.frame_id = "map";
  detections.channel_index = 0;
}

// Helper methods
void TestBench::initializeCarObject(
  autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
  const rclcpp::Time & stamp, const ObjectState & state)
{
  obj.uuid.uuid = stringToUUID(id);
  obj.time = stamp;
  obj.classification.emplace_back();
  obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  obj.shape.dimensions.x = state.shape.x;
  obj.shape.dimensions.y = state.shape.y;
  obj.shape.dimensions.z = 1.5;
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  initializeKinematics(obj);
  obj.pose = state.pose;
  obj.twist.linear.x = std::hypot(state.twist.linear.x, state.twist.linear.y);
  obj.twist.linear.y = 0.0;
  obj.existence_probability = 0.95;
  obj.channel_index = 0;
  obj.area = state.shape.x * state.shape.y;
}

void TestBench::initializePedestrianObject(
  autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
  const rclcpp::Time & stamp, const ObjectState & state)
{
  obj.uuid.uuid = stringToUUID(id);
  obj.time = stamp;
  obj.classification.emplace_back();
  obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  obj.shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
  obj.shape.dimensions.x = 0.4;
  obj.shape.dimensions.y = 0.4;
  obj.shape.dimensions.z = 1.5;
  initializeKinematics(obj);
  obj.pose = state.pose;
  obj.twist.linear.x = std::hypot(state.twist.linear.x, state.twist.linear.y);
  obj.twist.linear.y = 0.0;
  obj.existence_probability = 0.9;
  obj.channel_index = 0;
  obj.area = state.shape.x * state.shape.y;
}

void TestBench::initializeUnknownObject(
  autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
  const rclcpp::Time & stamp, const UnknownObjectState & state)
{
  obj.uuid.uuid = stringToUUID(id);
  obj.time = stamp;
  obj.classification.resize(1);
  obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  obj.classification[0].probability = 1.0;

  // Shape configuration
  obj.shape.type = state.shape_type;
  if (obj.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    obj.shape.dimensions.x = state.base_size;
    obj.shape.dimensions.y = state.base_size / 2.0;
    obj.shape.dimensions.z = state.z_dimension;
  } else {
    obj.shape.footprint.points.clear();
    for (const auto & p : state.current_footprint) {
      geometry_msgs::msg::Point32 point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.0;
      obj.shape.footprint.points.push_back(point);
    }
    obj.shape.dimensions.x = 0.0;
    obj.shape.dimensions.y = 0.0;
    obj.shape.dimensions.z = state.z_dimension;
  }

  initializeKinematics(obj);
  obj.pose = state.pose;
  obj.twist.linear.x = std::hypot(state.twist.linear.x, state.twist.linear.y);
  obj.twist.linear.y = 0.0;
  obj.existence_probability = autoware::multi_object_tracker::types::default_existence_probability;
  obj.channel_index = 0;
  obj.area = obj.shape.dimensions.x * obj.shape.dimensions.y;
}

void TestBench::initializeKinematics(autoware::multi_object_tracker::types::DynamicObject & obj)
{
  obj.kinematics.has_position_covariance = false;
  obj.kinematics.has_twist = false;
  obj.kinematics.has_twist_covariance = false;
  obj.pose_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  obj.twist_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void TestBench::addNoiseAndOrientation(
  autoware::multi_object_tracker::types::DynamicObject & obj, const ObjectState & state)
{
  obj.pose.position.x += pos_noise_(rng_);
  obj.pose.position.y += pos_noise_(rng_);
  setOrientationFromVelocity(state.twist, obj.pose);
}

void TestBench::addNoiseAndOrientation(
  autoware::multi_object_tracker::types::DynamicObject & obj, const UnknownObjectState & state)
{
  obj.pose.position.x += pos_noise_(rng_);
  obj.pose.position.y += pos_noise_(rng_);
  setOrientationFromVelocity(state.twist, obj.pose);
}

void TestBench::updateUnknownState(UnknownObjectState & state, double dt)
{
  if (state.is_moving) {
    state.pose.position.x += state.twist.linear.x * dt;
    state.pose.position.y += state.twist.linear.y * dt;
  }
  // Random shape evolution (30% chance of significant change)
  if (shape_change_dist_(rng_)) {
    updateUnknownShape(state);
  } else {
    auto new_footprint = state.current_footprint;
    // Minor shape variations
    for (auto & point : new_footprint) {
      point.x += shape_evolution_noise_(rng_);
      point.y += shape_evolution_noise_(rng_);
    }
    if (isConvex(new_footprint)) {
      state.current_footprint = new_footprint;
    }
  }
}

autoware::multi_object_tracker::types::DynamicObjectList TestBench::generateDetections(
  const rclcpp::Time & stamp)
{
  const double dt = (stamp - last_stamp_).seconds();
  last_stamp_ = stamp;

  autoware::multi_object_tracker::types::DynamicObjectList detections;
  initializeDetectionHeader(detections, stamp);

  // Update and generate car detections
  updateCarStates(dt);
  for (auto & [id, state] : car_states_) {
    if (dropout_dist_(rng_)) continue;
    // Add noise and create detection
    autoware::multi_object_tracker::types::DynamicObject obj;
    initializeCarObject(obj, id, stamp, state);
    addNoiseAndOrientation(obj, state);
    detections.objects.push_back(obj);
  }
  // Update and generate pedestrian detections
  for (auto & [id, state] : pedestrian_states_) {
    if (dropout_dist_(rng_)) continue;

    // Update state (pedestrians move randomly)
    state.pose.position.x += state.twist.linear.x * dt;
    state.pose.position.y += state.twist.linear.y * dt;

    // Change direction occasionally
    if (dir_change_dist_(rng_)) {
      state.twist.linear.x = pedestrian_speed_dist_(rng_) * cos_dist_(rng_);
      state.twist.linear.y = pedestrian_speed_dist_(rng_) * sin_dist_(rng_);
    }

    autoware::multi_object_tracker::types::DynamicObject obj;
    initializePedestrianObject(obj, id, stamp, state);
    addNoiseAndOrientation(obj, state);
    detections.objects.push_back(obj);
  }

  // Update and generate unknown object detections
  for (auto & [id, state] : unknown_states_) {
    if (dropout_dist_(rng_)) continue;

    // Move if it's a moving unknown object
    updateUnknownState(state, dt);

    // Create detection
    autoware::multi_object_tracker::types::DynamicObject obj;
    initializeUnknownObject(obj, id, stamp, state);
    addNoiseAndOrientation(obj, state);
    detections.objects.push_back(obj);
  }

  // Add new objects occasionally
  if (new_obj_dist_(rng_)) {
    addNewPedestrian("ped_" + std::to_string(object_counter_++), 0, 0);
  }
  return detections;
}

void TestBench::updateCarStates(float dt)
{
  for (auto & [id, state] : car_states_) {
    // Update state
    auto old_pos = state.pose.position;
    updateGrid(id, true);  // Remove from old grid position

    // Predict movement
    state.pose.position.x += state.twist.linear.x * dt;
    state.pose.position.y += state.twist.linear.y * dt;

    // Check for collisions
    if (checkCollisions(id)) {
      state.pose.position = old_pos;  // Revert
      state.twist.linear.x *= 0.9f;   // Reduce speed
      state.twist.linear.y *= 0.9f;   // Reduce speed
    }

    updateGrid(id);  // Add to new grid position

    // state.pose.position.x += state.twist.linear.x * dt;
    state.pose.position.y += lateral_drift_(rng_) * dt;
    state.pose.position.x += lateral_drift_(rng_) * dt;
  }
}
void TestBench::initializeObjects()
{
  // Initialize cars
  for (int lane = 0; lane < params_.num_lanes; ++lane) {
    const float y = lane * params_.lane_width;
    float x = static_cast<float>(-params_.cars_per_lane * params_.car_spacing_mean);

    for (int i = 0; i < params_.cars_per_lane; ++i) {
      std::string id = "car_l" + std::to_string(lane) + "_" + std::to_string(i);
      // Rotate initial position
      float x_rot = x * params_.lane_angle_cos - y * params_.lane_angle_sin;
      float y_rot = x * params_.lane_angle_sin + y * params_.lane_angle_cos;
      float speed = car_speed_dist_(rng_);
      float speed_x = speed * params_.lane_angle_cos;
      float speed_y = speed * params_.lane_angle_sin;
      addNewCar(id, x_rot, y_rot, speed_x, speed_y);
      x += car_spacing_dist_(rng_) + car_length_dist_(rng_);
    }
  }

  // Initialize pedestrians
  const float y_spacing =
    params_.pedestrian_cluster_spacing;  // Use 80% of road width for pedestrian areas
  const int y_clusters = std::max(1, static_cast<int>(std::sqrt(params_.pedestrian_clusters)));
  const float cluster_x_offset = (y_clusters - 1) * params_.pedestrian_cluster_spacing / 2.0f;
  const float cluster_y_offset = (y_clusters - 1) * y_spacing + params_.lane_width * 0.5f + 30.0f;

  // Initialize pedestrians
  for (int cluster = 0; cluster < params_.pedestrian_clusters; ++cluster) {
    // Calculate 2D grid positions for clusters
    const int x_idx = cluster % y_clusters;
    const int y_idx = cluster / y_clusters;

    const float center_x = x_idx * params_.pedestrian_cluster_spacing - cluster_x_offset;
    const float center_y = y_idx * y_spacing - cluster_y_offset;

    for (int j = 0; j < params_.pedestrians_per_cluster; ++j) {
      std::string id = "ped_c" + std::to_string(cluster) + "_" + std::to_string(j);
      addNewPedestrian(id, center_x + pos_noise_(rng_), center_y + pedestrian_y_dist_(rng_));
    }
  }
  // Initialize unknown objects
  // Start unknown objects after the last car's x position
  float unknown_start_x = -params_.unknown_objects * 3.0f;  // 6.0f/2.0f
  float unknown_start_y =
    (params_.num_lanes + 1) * params_.lane_width + 25.0f;  // Start above the road

  for (int i = 0; i < params_.unknown_objects; ++i) {
    std::string id = "unk_" + std::to_string(i);
    // Wide scatter: uniform distribution in ±50m
    float x = unknown_start_x + unknown_pos_dist_(rng_) + i * 6.0f;
    float y = unknown_start_y + unknown_pos_dist_(rng_);
    addNewUnknown(id, x, y);
  }
}
void TestBench::setOrientationFromVelocity(
  const geometry_msgs::msg::Twist & twist, geometry_msgs::msg::Pose & pose)
{
  const double velocity_magnitude =
    std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);

  if (velocity_magnitude > 0.1) {
    const double yaw = std::atan2(twist.linear.y, twist.linear.x);
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
  } else {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
  }
}
void TestBench::addNewCar(const std::string & id, float x, float y, float speed_x, float speed_y)
{
  ObjectState state;
  state.twist.linear.x = speed_x;
  state.twist.linear.y = speed_y;
  state.shape.x = car_length_dist_(rng_);
  state.shape.y = car_width_dist_(rng_);
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.position.z = 0.5;
  car_states_[id] = state;
  updateGrid(id);
}

void TestBench::addNewPedestrian(const std::string & id, float x, float y)
{
  ObjectState state;
  state.twist.linear.x = pedestrian_speed_dist_(rng_) * cos_dist_(rng_);
  state.twist.linear.y = pedestrian_speed_dist_(rng_) * sin_dist_(rng_);
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.position.z = 0.5;
  state.shape.x = 0.4;
  state.shape.y = 0.4;
  pedestrian_states_[id] = state;
}

void TestBench::addNewUnknown(const std::string & id, float x, float y)
{
  UnknownObjectState state;
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.position.z = z_pos_noise_(rng_);
  state.pose.orientation.w = 1.0;

  // Movement properties
  state.is_moving = movement_chance_dist_(rng_);

  if (state.is_moving) {
    float speed = moving_unknown_speed_dist_(rng_);
    state.twist.linear.x = speed * cos_dist_(rng_);
    state.twist.linear.y = speed * sin_dist_(rng_);
  }
  // Shape properties
  state.z_dimension = z_size_noise_(rng_);
  state.base_size = base_size_dist_(rng_);

  // Initial shape
  updateUnknownShape(state);
  unknown_states_[id] = state;
}

void TestBench::generateClusterFootprint(
  float base_size, std::vector<geometry_msgs::msg::Point> & footprint)
{
  const int num_points = point_count_dist_(rng_);
  footprint.resize(num_points);

  float radius = base_size * footprint_radius_scale_dist_(rng_);
  for (int i = 0; i < num_points; ++i) {
    float angle = 2.0f * M_PI * i / num_points;
    footprint[i].x = radius * cos(angle);
    footprint[i].y = radius * sin(angle);
    footprint[i].z = 0.0f;
  }
}

void TestBench::updateUnknownShape(UnknownObjectState & state)
{
  state.previous_footprint = state.current_footprint;

  // Randomly decide shape type (70% polygon, 30% bounding box)
  if (shape_type_dist_(rng_)) {
    state.shape_type = autoware_perception_msgs::msg::Shape::POLYGON;
    generateClusterFootprint(state.base_size, state.current_footprint);
  } else {
    state.shape_type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    state.current_footprint.clear();
  }
}
