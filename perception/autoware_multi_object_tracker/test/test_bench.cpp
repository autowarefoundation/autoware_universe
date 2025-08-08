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

uint64_t TrackingTestBench::getGridKey(float x, float y) const
{
  return (static_cast<uint64_t>(x / GRID_SIZE) << 32) | static_cast<uint32_t>(y / GRID_SIZE);
}

void TrackingTestBench::updateGrid(const std::string & id, bool remove_first)
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

bool TrackingTestBench::checkCollisions(const std::string & id)
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

autoware::multi_object_tracker::types::DynamicObjectList TrackingTestBench::generateDetections(
  const rclcpp::Time & stamp)
{
  const double dt = (stamp - last_stamp_).seconds();
  last_stamp_ = stamp;

  autoware::multi_object_tracker::types::DynamicObjectList detections;
  detections.header.stamp = stamp;
  detections.header.frame_id = "map";
  detections.channel_index = 0;

  // Update and generate car detections
  for (auto & [id, state] : car_states_) {
    if (dropout_dist_(rng_)) continue;
    // Update state
    auto old_pos = state.pose.position;
    updateGrid(id, true);  // Remove from old grid position

    // Predict movement
    state.pose.position.x += state.twist.linear.x * dt;

    // Check for collisions
    if (checkCollisions(id)) {
      state.pose.position = old_pos;  // Revert
      state.twist.linear.x *= 0.9f;   // Reduce speed
    }

    updateGrid(id);  // Add to new grid position

    // state.pose.position.x += state.twist.linear.x * dt;
    state.pose.position.y += lateral_drift_(rng_) * dt;

    // Add noise and create detection
    autoware::multi_object_tracker::types::DynamicObject obj;
    obj.uuid.uuid = stringToUUID(id);
    obj.time = stamp;
    obj.classification.emplace_back();
    obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    obj.shape.dimensions.x = state.shape.x;
    obj.shape.dimensions.y = state.shape.y;
    obj.shape.dimensions.z = 1.5;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    // Kinematics
    obj.kinematics.has_position_covariance = false;
    obj.kinematics.has_twist = false;
    obj.kinematics.has_twist_covariance = false;
    obj.pose_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    obj.twist_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    obj.pose = state.pose;
    obj.pose.position.x += pos_noise_(rng_);
    obj.pose.position.y += pos_noise_(rng_);
    // Set orientation based on velocity
    setOrientationFromVelocity(state.twist, obj.pose);
    obj.twist = state.twist;
    obj.twist.linear.x += vel_noise_(rng_);
    obj.existence_probability = 0.95;
    obj.channel_index = 0;
    obj.area = state.shape.x * state.shape.y;
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
    obj.uuid.uuid = stringToUUID(id);
    obj.time = stamp;
    obj.classification.emplace_back();
    obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    obj.shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
    obj.shape.dimensions.x = 0.4;
    obj.shape.dimensions.y = 0.4;
    obj.shape.dimensions.z = 1.5;

    // Kinematics
    obj.kinematics.has_position_covariance = false;
    obj.kinematics.has_twist = false;
    obj.kinematics.has_twist_covariance = false;
    obj.pose_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    obj.twist_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    obj.pose = state.pose;
    obj.pose.position.x += pos_noise_(rng_);
    obj.pose.position.y += pos_noise_(rng_);
    // Set orientation based on velocity
    setOrientationFromVelocity(state.twist, obj.pose);
    obj.twist = state.twist;
    obj.existence_probability = 0.9;
    obj.channel_index = 0;
    obj.area = state.shape.x * state.shape.y;
    detections.objects.push_back(obj);
  }
  // Add new objects occasionally
  if (new_obj_dist_(rng_)) {
    addNewCar("car_" + std::to_string(object_counter_++), 0, 0);
  }
  if (new_obj_dist_(rng_)) {
    addNewPedestrian("ped_" + std::to_string(object_counter_++), 0, 0);
  }
  return detections;
}

void TrackingTestBench::initializeObjects(const TrackingScenarioConfig & params)
{
  // Initialize cars
  for (int lane = 0; lane < params.num_lanes; ++lane) {
    auto y = lane * params.lane_width;
    auto x = static_cast<float>(-params.cars_per_lane * params.car_spacing_mean);

    for (int i = 0; i < params.cars_per_lane; ++i) {
      std::string id = "car_l" + std::to_string(lane) + "_" + std::to_string(i);
      addNewCar(id, x, y);
      x += car_spacing_dist_(rng_) + car_length_dist_(rng_);
    }
  }

  // Initialize pedestrians
  const float y_spacing =
    params.pedestrian_cluster_spacing;  // Use 80% of road width for pedestrian areas
  const int y_clusters = std::max(1, static_cast<int>(std::sqrt(params.pedestrian_clusters)));

  // Initialize pedestrians
  for (int cluster = 0; cluster < params.pedestrian_clusters; ++cluster) {
    // Calculate 2D grid positions for clusters
    const int x_idx = cluster % y_clusters;
    const int y_idx = cluster / y_clusters;

    auto center_x = static_cast<float>(
      x_idx * params.pedestrian_cluster_spacing -
      (y_clusters - 1) * params.pedestrian_cluster_spacing / 2.0f);

    auto center_y = static_cast<float>(
      (y_idx)*y_spacing - (y_clusters - 1) * y_spacing -
      params.lane_width * 0.5f);  // Offset from road center

    for (int j = 0; j < params.pedestrians_per_cluster; ++j) {
      std::string id = "ped_c" + std::to_string(cluster) + "_" + std::to_string(j);
      addNewPedestrian(id, center_x + pos_noise_(rng_), center_y + pedestrian_y_dist_(rng_));
    }
  }
}
void TrackingTestBench::setOrientationFromVelocity(
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
void TrackingTestBench::addNewCar(const std::string & id, float x, float y)
{
  ObjectState state;
  state.twist.linear.x = car_speed_dist_(rng_);
  state.shape.x = car_length_dist_(rng_);
  state.shape.y = car_width_dist_(rng_);
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.position.z = 0.5;
  car_states_[id] = state;
  updateGrid(id);
}

void TrackingTestBench::addNewPedestrian(const std::string & id, float x, float y)
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
