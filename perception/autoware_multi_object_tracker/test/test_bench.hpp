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
#ifndef TEST_BENCH_HPP_
#define TEST_BENCH_HPP_
#include "../src/multi_object_tracker_node.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"
#include "test_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct ObjectState
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  rclcpp::Time last_update;
  struct Shape
  {
    float x;
    float y;
  } shape;
};

struct UnknownObjectState
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
  std::vector<geometry_msgs::msg::Point> current_footprint;
  std::vector<geometry_msgs::msg::Point> previous_footprint;
  float z_dimension;
  float base_size;
  rclcpp::Time last_update;
  uint8_t shape_type;
  bool is_moving;
};

struct UnknownObjectParams
{
  // Size parameters
  float min_size = 0.5f;
  float max_size = 5.0f;
  float min_z = 0.2f;
  float max_z = 1.5f;

  // Movement parameters
  float stationary_probability = 0.7f;  // 70% chance of being stationary
  float min_speed = 0.0f;
  float max_speed = 15.0f;

  // Shape parameters
  float polygon_probability = 0.7f;  // 70% chance of polygon shape
  int min_points = 3;
  int max_points = 8;

  // Evolution parameters
  float shape_change_prob = 0.4f;
  float max_evolution_noise = 0.2f;
};

struct ScenarioParams
{
  int num_lanes = 8;                           // 5 lanes
  int cars_per_lane = 20;                      // 20 cars per lane
  float lane_width = 3.5f;                     // 3.5m width per lane
  double lane_angle_cos = std::cos(M_PI / 6);  // 30 degrees angle for lanes
  double lane_angle_sin = std::sin(M_PI / 6);  // 30 degrees angle for lanes

  int pedestrian_clusters = 16;              // 16 pedestrian clusters
  int pedestrians_per_cluster = 6;           // 6 pedestrians per cluster
  float pedestrian_cluster_spacing = 10.0f;  // 10m spacing between pedestrian clusters

  double car_spacing_mean = 10.0;
  double lateral_drift_min = 0.0;
  double lateral_drift_max = 0.3;
  double car_speed_min = 7.0;
  double car_speed_max = 14.0;
  double car_length_min = 4.2;
  double car_length_max = 5.5;
  double car_width_min = 1.7;
  double car_width_max = 2.0;

  double pedestrian_speed_mean = 1.2;
  double pedestrian_speed_stddev = 0.4;
  double pedestrian_height_min = 1.5;
  double pedestrian_height_max = 1.9;
  double pedestrian_y_min = -4.0;
  double pedestrian_y_max = -2.5;

  // Unknown objects configuration
  int unknown_objects = 20;            // Number of unknown objects
  UnknownObjectParams unknown_params;  // Parameters for unknown objects

  double dropout_rate = 0.05;
  double pos_noise_min = 0.0;
  double pos_noise_max = 0.2;
  double vel_noise_min = 0.0;
  double vel_noise_max = 0.5;
};

// Configuration creation functions
autoware::multi_object_tracker::TrackerProcessorConfig createProcessorConfig();
autoware::multi_object_tracker::AssociatorConfig createAssociatorConfig();
std::vector<autoware::multi_object_tracker::types::InputChannel> createInputChannelsConfig();

class TestBench
{
protected:
  const float GRID_SIZE = 15.0f;  // Larger than max car length
  std::unordered_map<uint64_t, std::vector<std::string>> spatial_grid_;
  UnknownObjectParams unknown_params_;
  ScenarioParams params_;  // store params for later use

  uint64_t getGridKey(float x, float y) const;
  void updateGrid(const std::string & id, bool remove_first = false);
  bool checkCollisions(const std::string & id);

public:
  explicit TestBench(const ScenarioParams & params)
  : unknown_params_(params.unknown_params),
    params_(params),
    base_time_(rclcpp::Clock().now(), RCL_ROS_TIME),
    object_counter_(0),
    rng_(0),  // To reproduce the same results, we need to set the seed to 0
    // Known object distributions
    car_spacing_dist_(params.car_spacing_mean),
    lateral_drift_(params.lateral_drift_min, params.lateral_drift_max),
    car_speed_dist_(params.car_speed_min, params.car_speed_max),
    car_length_dist_(params.car_length_min, params.car_length_max),
    car_width_dist_(params.car_width_min, params.car_width_max),
    pedestrian_speed_dist_(params.pedestrian_speed_mean, params.pedestrian_speed_stddev),
    pedestrian_height_dist_(params.pedestrian_height_min, params.pedestrian_height_max),
    pedestrian_y_dist_(params.pedestrian_y_min, params.pedestrian_y_max),
    dropout_dist_(params.dropout_rate),
    pos_noise_(params.pos_noise_min, params.pos_noise_max),
    vel_noise_(params.vel_noise_min, params.vel_noise_max),
    // Unknown object distributions
    base_size_dist_(params.unknown_params.min_size, params.unknown_params.max_size),
    z_size_noise_(params.unknown_params.min_z, params.unknown_params.max_z),
    z_pos_noise_(params.unknown_params.min_z, params.unknown_params.max_z),
    shape_type_dist_(0.7f),
    point_count_dist_(params.unknown_params.min_points, params.unknown_params.max_points),
    shape_evolution_noise_(0.0f, params.unknown_params.max_evolution_noise),
    movement_chance_dist_(1.0f - params.unknown_params.stationary_probability),
    moving_unknown_speed_dist_(params.unknown_params.min_speed, params.unknown_params.max_speed),
    shape_change_dist_(params.unknown_params.shape_change_prob),
    footprint_radius_scale_dist_(0.7f, 1.2f)
  {
  }

  virtual void initializeObjects();
  virtual autoware::multi_object_tracker::types::DynamicObjectList generateDetections(
    const rclcpp::Time & stamp);

protected:
  void setOrientationFromVelocity(
    const geometry_msgs::msg::Twist & twist, geometry_msgs::msg::Pose & pose);
  // Functions to add new objects
  virtual void initializeDetectionHeader(
    autoware::multi_object_tracker::types::DynamicObjectList & detections,
    const rclcpp::Time & stamp);
  virtual void initializeKinematics(autoware::multi_object_tracker::types::DynamicObject & obj);
  virtual void initializeCarObject(
    autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
    const rclcpp::Time & stamp, const ObjectState & state);
  virtual void initializePedestrianObject(
    autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
    const rclcpp::Time & stamp, const ObjectState & state);
  virtual void initializeUnknownObject(
    autoware::multi_object_tracker::types::DynamicObject & obj, const std::string & id,
    const rclcpp::Time & stamp, const UnknownObjectState & state);
  virtual void addNoiseAndOrientation(
    autoware::multi_object_tracker::types::DynamicObject & obj, const ObjectState & state);
  virtual void addNoiseAndOrientation(
    autoware::multi_object_tracker::types::DynamicObject & obj, const UnknownObjectState & state);

  virtual void addNewCar(
    const std::string & id, float x, float y, float speed_x = 0.0f, float speed_y = 0.0f);
  virtual void addNewPedestrian(const std::string & id, float x, float y);
  virtual void addNewUnknown(const std::string & id, float x, float y);
  virtual void updateCarStates(float dt);
  virtual void updateUnknownState(UnknownObjectState & state, double dt);
  // Functions to generate random shapes for unknown objects
  virtual void generateClusterFootprint(
    float base_size, std::vector<geometry_msgs::msg::Point> & footprint);
  virtual void updateUnknownShape(UnknownObjectState & state);
  struct Shape
  {
    float x;
    float y;
  };

  rclcpp::Time base_time_;
  rclcpp::Time last_stamp_ = base_time_;
  int object_counter_;
  std::mt19937 rng_;

  // Distributions for known objects
  std::normal_distribution<float> car_spacing_dist_;
  std::uniform_real_distribution<float> lateral_drift_;
  std::uniform_real_distribution<float> car_speed_dist_;
  std::uniform_real_distribution<double> car_length_dist_;
  std::uniform_real_distribution<double> car_width_dist_;
  std::uniform_real_distribution<float> pedestrian_speed_dist_;
  std::uniform_real_distribution<float> pedestrian_height_dist_;
  std::uniform_real_distribution<double> pedestrian_y_dist_;
  std::bernoulli_distribution dropout_dist_;
  std::normal_distribution<float> pos_noise_;
  std::normal_distribution<float> vel_noise_;
  std::bernoulli_distribution dir_change_dist_{0.1};
  std::uniform_real_distribution<float> cos_dist_{-1.0, 1.0};
  std::uniform_real_distribution<float> sin_dist_{-1.0, 1.0};
  std::bernoulli_distribution new_obj_dist_{0.0};

  // Distributions for unknown objects
  std::uniform_real_distribution<float> base_size_dist_;
  std::uniform_real_distribution<float> z_size_noise_;
  std::uniform_real_distribution<float> z_pos_noise_;
  std::bernoulli_distribution shape_type_dist_;
  std::uniform_int_distribution<int> point_count_dist_;
  std::normal_distribution<float> shape_evolution_noise_;  // renamed from cluster_evolution_noise_
  std::bernoulli_distribution movement_chance_dist_;
  std::uniform_real_distribution<float> moving_unknown_speed_dist_;
  std::bernoulli_distribution shape_change_dist_;
  std::uniform_real_distribution<float> footprint_radius_scale_dist_;
  std::uniform_real_distribution<float> unknown_pos_dist_{-20.0f, 20.0f};

  // Object states
  std::unordered_map<std::string, ObjectState> car_states_;
  std::unordered_map<std::string, ObjectState> pedestrian_states_;
  std::unordered_map<std::string, UnknownObjectState> unknown_states_;
};

#endif  // TEST_BENCH_HPP_
