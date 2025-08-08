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

struct TrackingScenarioConfig
{
  int num_lanes = 8;        // 5 lanes
  int cars_per_lane = 20;   // 20 cars per lane
  float lane_width = 3.5f;  // 3.5m width per lane

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

class TrackingTestBench
{
  const float GRID_SIZE = 15.0f;  // Larger than max car length
  std::unordered_map<uint64_t, std::vector<std::string>> spatial_grid_;

  uint64_t getGridKey(float x, float y) const;

  void updateGrid(const std::string & id, bool remove_first = false);

  bool checkCollisions(const std::string & id);

public:
  explicit TrackingTestBench(const TrackingScenarioConfig & params)
  : base_time_(rclcpp::Clock().now(), RCL_ROS_TIME),
    object_counter_(0),
    rng_(0),  // To reproduce the same results, we need to set the seed to 0
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
    vel_noise_(params.vel_noise_min, params.vel_noise_max)
  {
    initializeObjects(params);
  }

  autoware::multi_object_tracker::types::DynamicObjectList generateDetections(
    const rclcpp::Time & stamp);

private:
  void initializeObjects(const TrackingScenarioConfig & params);
  void setOrientationFromVelocity(
    const geometry_msgs::msg::Twist & twist, geometry_msgs::msg::Pose & pose);
  void addNewCar(const std::string & id, float x, float y);

  void addNewPedestrian(const std::string & id, float x, float y);
  struct Shape
  {
    float x;
    float y;
  };

  rclcpp::Time base_time_;
  rclcpp::Time last_stamp_ = base_time_;
  int object_counter_;
  std::mt19937 rng_;

  // Distributions
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

  // Object states
  std::unordered_map<std::string, ObjectState> car_states_;
  std::unordered_map<std::string, ObjectState> pedestrian_states_;
};
#endif  // TEST_BENCH_HPP_
