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
#include "../src/multi_object_tracker_node.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/odometry.hpp"
#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"
#include "test_bench.hpp"
#include "test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detail/detected_objects__struct.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Using-declarations for chrono literals
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using Clock = std::chrono::high_resolution_clock;

FunctionTimings runIterations(
  int num_iterations, const TrackingScenarioConfig & config, bool print_frame_stats = false,
  bool write_bag = false)
{
  RosbagWriterHelper writer(write_bag);  // Initialize rosbag writer if enabled

  using Clock = std::chrono::high_resolution_clock;

  autoware::multi_object_tracker::TrackerProcessorConfig processor_config = createProcessorConfig();
  autoware::multi_object_tracker::AssociatorConfig associator_config = createAssociatorConfig();
  processor_config.max_dist_matrix = associator_config.max_dist_matrix;
  std::vector<autoware::multi_object_tracker::types::InputChannel> input_channels_config =
    createInputChannelsConfig();

  auto processor = std::make_unique<autoware::multi_object_tracker::TrackerProcessor>(
    processor_config, associator_config, input_channels_config);
  TrackingTestBench simulator(config);
  // Performance tracking for individual functions
  FunctionTimings timings;

  rclcpp::Clock clock;
  rclcpp::Time current_time = rclcpp::Time(clock.now(), RCL_ROS_TIME);

  for (int i = 0; i < num_iterations; ++i) {
    // Advance simulation time (10Hz)
    current_time += 100ms;
    auto detections = simulator.generateDetections(current_time);
    detections = autoware::multi_object_tracker::uncertainty::modelUncertainty(detections);

    auto total_start = Clock::now();

    // Individual function timing
    auto predict_start = Clock::now();
    processor->predict(current_time, std::nullopt);
    auto predict_end = Clock::now();
    auto predict_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(predict_end - predict_start).count() /
      1000.0;
    timings.predict.times.push_back(predict_duration);

    std::unordered_map<int, int> direct_assignment, reverse_assignment;

    auto associate_start = Clock::now();
    processor->associate(detections, direct_assignment, reverse_assignment);
    auto associate_end = Clock::now();
    auto associate_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(associate_end - associate_start)
        .count() /
      1000.0;
    timings.associate.times.push_back(associate_duration);

    // std::cout << "DEBUG: Starting update step..." << std::endl;
    auto update_start = Clock::now();
    processor->update(detections, direct_assignment);
    auto update_end = Clock::now();
    auto update_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(update_end - update_start).count() /
      1000.0;
    timings.update.times.push_back(update_duration);
    // std::cout << "DEBUG: Update step completed successfully" << std::endl;

    // std::cout << "DEBUG: Starting prune step..." << std::endl;
    auto prune_start = Clock::now();
    processor->prune(current_time);
    auto prune_end = Clock::now();
    auto prune_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(prune_end - prune_start).count() /
      1000.0;
    timings.prune.times.push_back(prune_duration);
    // std::cout << "DEBUG: Prune step completed successfully" << std::endl;

    // std::cout << "DEBUG: Starting spawn step..." << std::endl;
    auto spawn_start = Clock::now();
    processor->spawn(detections, reverse_assignment);
    auto spawn_end = Clock::now();
    auto spawn_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(spawn_end - spawn_start).count() /
      1000.0;
    timings.spawn.times.push_back(spawn_duration);
    // std::cout << "DEBUG: Spawn step completed successfully" << std::endl;

    auto total_end = Clock::now();
    auto total_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() /
      1000.0;
    timings.total.times.push_back(total_duration);
    // std::cout << "DEBUG: All processing steps completed successfully" << std::endl;

    if (i % 10 == 0 && print_frame_stats) {
      printFrameStats(i, detections, timings);
    }

    autoware_perception_msgs::msg::TrackedObjects latest_tracked_objects;
    processor->getTrackedObjects(current_time, latest_tracked_objects);
    latest_tracked_objects.header.frame_id = "map";

    writer.write(
      toDetectedObjectsMsg(detections), "/perception/object_recognition/detection/objects",
      current_time);
    writer.write(
      latest_tracked_objects, "/perception/object_recognition/tracking/objects", current_time);
  }
  return timings;
}

void runPerformanceTest()
{
  TrackingScenarioConfig params;
  FunctionTimings timings = runIterations(500, params, true, false);

  // Calculate all statistics
  timings.predict.calculate();
  timings.associate.calculate();
  timings.update.calculate();
  timings.prune.calculate();
  timings.spawn.calculate();
  timings.total.calculate();

  // Print summary statistics
  std::cout << "\n=== Overall Performance Statistics ===" << std::endl;
  printPerformanceStats("Total", timings.total);

  std::cout << "\n=== Individual Function Performance ===" << std::endl;
  printPerformanceStats("Predict", timings.predict);
  printPerformanceStats("Associate", timings.associate);
  printPerformanceStats("Update", timings.update);
  printPerformanceStats("Prune", timings.prune);
  printPerformanceStats("Spawn", timings.spawn);
}

void runPerformanceTestWithRosbag(const std::string & rosbag_path)
{
  // === Setup ===
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("multi_object_tracker_test_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
  RosbagWriterHelper writer(true);

  std::string world_frame_id = "map";      // Assuming map is the world frame ID
  std::string ego_frame_id = "base_link";  // Assuming base_link is the ego vehicle frame ID
  auto odometry = std::make_shared<autoware::multi_object_tracker::Odometry>(
    *node, world_frame_id, ego_frame_id, true);

  auto processor_config = createProcessorConfig();
  auto associator_config = createAssociatorConfig();
  processor_config.max_dist_matrix = associator_config.max_dist_matrix;
  auto input_channels_config = createInputChannelsConfig();

  auto processor = std::make_unique<autoware::multi_object_tracker::TrackerProcessor>(
    processor_config, associator_config, input_channels_config);

  // === Open Rosbag ===
  rosbag2_cpp::Reader reader;
  reader.open(rosbag_path);

  // Check if the bag is opened successfully
  if (!reader.has_next()) {
    std::cerr << "No messages found in the bag file." << std::endl;
    return;
  }
  // Create serialization objects
  rclcpp::Serialization<autoware_perception_msgs::msg::DetectedObjects> detection_serialization;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (bag_message->topic_name == "/tf" || bag_message->topic_name == "/tf_static") {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);  // wrap raw buffer

      tf2_msgs::msg::TFMessage tf_msg;
      tf_serialization.deserialize_message(&serialized_msg, &tf_msg);
      for (auto & tf : tf_msg.transforms) {
        tf_buffer->setTransform(tf, "authority1");
      }

      writer.write(tf_msg, "/tf", rclcpp::Time(tf_msg.transforms.front().header.stamp));
    } else if (bag_message->topic_name == "/perception/object_recognition/detection/objects") {
      auto tf_transform = tf_buffer->lookupTransform(world_frame_id, ego_frame_id, rclcpp::Time(0));

      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg =
        std::make_shared<autoware_perception_msgs::msg::DetectedObjects>();
      detection_serialization.deserialize_message(&serialized_msg, msg.get());
      msg->header.frame_id = "base_link";  // Set frame_id to base_link for processing

      // Convert to DynamicObjectList
      auto dynamic_objects = autoware::multi_object_tracker::types::toDynamicObjectList(*msg, 0);
      dynamic_objects =
        autoware::multi_object_tracker::uncertainty::modelUncertainty(dynamic_objects);
      for (auto & object : dynamic_objects.objects) {
        auto & pose = object.pose;
        auto & pose_cov = object.pose_covariance;
        tf2::Transform tf_target2objects;
        tf2::Transform tf_objects_world2objects;
        tf2::fromMsg(tf_transform.transform, tf_target2objects);
        tf2::fromMsg(pose, tf_objects_world2objects);
        tf_target2objects = tf_target2objects * tf_objects_world2objects;
        // transform pose, frame difference and object pose
        tf2::toMsg(tf_target2objects, pose);
        // transform covariance, only the frame difference
        pose_cov = tf2::transformCovariance(pose_cov, tf_target2objects);
      }
      // Process through tracker
      processor->predict(msg->header.stamp, std::nullopt);

      std::unordered_map<int, int> direct_assignment;
      std::unordered_map<int, int> reverse_assignment;
      processor->associate(dynamic_objects, direct_assignment, reverse_assignment);
      processor->update(dynamic_objects, direct_assignment);
      processor->prune(msg->header.stamp);
      processor->spawn(dynamic_objects, reverse_assignment);

      // Get and output results
      rclcpp::Time current_time(msg->header.stamp);
      autoware_perception_msgs::msg::TrackedObjects latest_tracked_objects;
      autoware_perception_msgs::msg::DetectedObjects latest_detected_objects;
      processor->getTrackedObjects(current_time, latest_tracked_objects);
      latest_detected_objects = toDetectedObjectsMsg(dynamic_objects);
      auto stamp = rclcpp::Time(msg->header.stamp);
      std::time_t time_sec = static_cast<std::time_t>(stamp.seconds());
      char time_str[32];
      std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&time_sec));
      std::cout << "Processed frame at " << time_str << " with "
                << latest_tracked_objects.objects.size() << " tracked objects" << std::endl;
      latest_tracked_objects.header.frame_id = "map";
      latest_detected_objects.header.frame_id = "map";

      writer.write(
        latest_detected_objects, "/perception/object_recognition/detection/objects", current_time);
      writer.write(
        latest_tracked_objects, "/perception/object_recognition/tracking/objects", current_time);
    }
  }
  rclcpp::shutdown();
}

void profilePerformanceVsCarCount()
{
  // Test configuration
  const int min_cars = 1;
  const int max_cars = 1000;
  const int step = 5;
  const int iterations_per_count = 5;      // Number of runs per car count
  const float simulation_duration = 5.0f;  // Seconds per test
  std::cout << "\n=== Performance vs Car Count (1-" << max_cars << " cars) ===" << std::endl;
  std::cout << std::left << std::setw(10) << "CarCount" << "," << std::setw(12) << "TotalTime"
            << "," << std::setw(12) << "PredictTime" << "," << std::setw(14) << "AssociateTime"
            << "," << std::setw(12) << "UpdateTime" << "," << std::setw(12) << "PruneTime" << ","
            << std::setw(12) << "SpawnTime" << std::endl;

  // Test different car counts
  for (int target_cars = min_cars; target_cars <= max_cars; target_cars += step) {
    TrackingScenarioConfig params;
    // Disable pedestrians
    params.pedestrian_clusters = 0;
    params.pedestrians_per_cluster = 0;

    // Configure lanes and cars per lane
    params.num_lanes = std::max(1, target_cars / 20);  // At least 1 lane
    params.cars_per_lane = std::max(1, target_cars / params.num_lanes);

    // Adjust car spacing based on density
    params.car_spacing_mean = 10.0f * (1.0f + target_cars / 500.0f);

    FunctionTimings total_timings;
    for (int i = 0; i < iterations_per_count; ++i) {
      int num_iterations = static_cast<int>(simulation_duration * 10.0f);
      FunctionTimings iteration_timings = runIterations(num_iterations, params);
      total_timings.accumulate(iteration_timings);
    }
    // Calculate statistics for this car count
    total_timings.calculate();
    std::cout << std::left << std::fixed << std::setprecision(3) << std::setw(10) << target_cars
              << "," << std::setw(12) << total_timings.total.avg << "," << std::setw(12)
              << total_timings.predict.avg << "," << std::setw(14) << total_timings.associate.avg
              << "," << std::setw(12) << total_timings.update.avg << "," << std::setw(12)
              << total_timings.prune.avg << "," << std::setw(12) << total_timings.spawn.avg
              << std::endl;
  }
}

void profilePerformanceVsPedestrianCount()
{
  // Test configuration
  const int min_peds = 1;
  const int max_peds = 1000;
  const int step = 5;
  const int iterations_per_count = 5;      // Number of runs per pedestrian count
  const float simulation_duration = 5.0f;  // Seconds per test

  std::cout << "\n=== Performance vs Pedestrian Count (1-" << max_peds
            << " pedestrians) ===" << std::endl;
  std::cout << std::left << std::setw(10) << "PedCount" << "," << std::setw(12) << "TotalTime"
            << "," << std::setw(12) << "PredictTime" << "," << std::setw(14) << "AssociateTime"
            << "," << std::setw(12) << "UpdateTime" << "," << std::setw(12) << "PruneTime" << ","
            << std::setw(12) << "SpawnTime" << std::endl;

  // Test different pedestrian counts
  for (int target_peds = min_peds; target_peds <= max_peds; target_peds += step) {
    TrackingScenarioConfig config;
    // Disable cars
    config.num_lanes = 0;
    config.cars_per_lane = 0;

    // Configure pedestrian clusters and pedestrians per cluster
    config.pedestrian_clusters = std::max(1, target_peds / 5);  // At least 1 cluster
    config.pedestrians_per_cluster = std::max(1, target_peds / config.pedestrian_clusters);

    // Adjust cluster spacing based on density
    config.pedestrian_cluster_spacing = 30.0f * (1.0f + target_peds / 200.0f);

    FunctionTimings total_timings;
    for (int i = 0; i < iterations_per_count; ++i) {
      int num_iterations = static_cast<int>(simulation_duration * 10.0f);
      FunctionTimings iteration_timings = runIterations(num_iterations, config);
      total_timings.accumulate(iteration_timings);
    }

    // Calculate statistics for this pedestrian count
    total_timings.calculate();

    std::cout << std::left << std::fixed << std::setprecision(3) << std::setw(10) << target_peds
              << "," << std::setw(12) << total_timings.total.avg << "," << std::setw(12)
              << total_timings.predict.avg << "," << std::setw(14) << total_timings.associate.avg
              << "," << std::setw(12) << total_timings.update.avg << "," << std::setw(12)
              << total_timings.prune.avg << "," << std::setw(12) << total_timings.spawn.avg
              << std::endl;
  }
}
class MultiObjectTrackerTest : public ::testing::Test
{
public:
  MultiObjectTrackerTest() = default;
  ~MultiObjectTrackerTest() override = default;
};

TEST_F(MultiObjectTrackerTest, test_multi_object_tracker)
{
  runPerformanceTest();
}
