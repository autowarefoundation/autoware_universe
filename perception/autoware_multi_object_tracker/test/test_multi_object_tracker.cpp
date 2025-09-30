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
#include "test_bench_association.hpp"
#include "test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
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

template <typename Func>
double measureTimeMs(Func && func)
{
  const auto start = Clock::now();
  func();
  const auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count();
}

FunctionTimings runIterationsAssociation(
  int num_iterations, const ScenarioParams & config, bool print_frame_stats = false,
  bool write_bag = false)
{
  RosbagWriterHelper writer(write_bag);

  auto processor_config = createProcessorConfig();
  const auto associator_config = createAssociatorConfig();
  const auto input_channels_config = createInputChannelsConfig();

  auto processor = std::make_unique<autoware::multi_object_tracker::TrackerProcessor>(
    processor_config, associator_config, input_channels_config);
  // TestBenchAssociation by default.
  // Or use TestBenchAssociationLemniscate for more complex association scenarios
  TestBenchAssociation simulator(config);
  simulator.initializeObjects();
  // Performance tracking for individual functions
  FunctionTimings timings;
  rclcpp::Clock clock;
  rclcpp::Time current_time = rclcpp::Time(clock.now(), RCL_ROS_TIME);
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  if (print_frame_stats) {
    printFrameStatsHeader();
  }
  tf2_msgs::msg::TFMessage static_tf_msg;
  if (write_bag) {
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;

    // Publish a default map frame transform (identity transform)
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = current_time;
    map_transform.header.frame_id = "map";
    map_transform.child_frame_id = "base_link";
    map_transform.transform.translation.x = 0.0;
    map_transform.transform.translation.y = 0.0;
    map_transform.transform.translation.z = 0.0;
    map_transform.transform.rotation.x = 0.0;
    map_transform.transform.rotation.y = 0.0;
    map_transform.transform.rotation.z = 0.0;
    map_transform.transform.rotation.w = 1.0;
    static_transforms.push_back(map_transform);
    // Create TF message for /tf_static topic
    ;
    static_tf_msg.transforms = static_transforms;
  }

  for (int i = 0; i < num_iterations; ++i) {
    direct_assignment.clear();
    reverse_assignment.clear();
    // Advance simulation time (10Hz)
    current_time += 100ms;
    auto detections = simulator.generateDetections(current_time);
    detections = autoware::multi_object_tracker::uncertainty::modelUncertainty(detections);

    const auto total_start = Clock::now();

    // Individual function timing
    timings.predict.times.push_back(
      measureTimeMs([&]() { processor->predict(current_time, std::nullopt); }));
    timings.associate.times.push_back(measureTimeMs(
      [&]() { processor->associate(detections, direct_assignment, reverse_assignment); }));
    timings.update.times.push_back(
      measureTimeMs([&]() { processor->update(detections, direct_assignment); }));
    timings.prune.times.push_back(measureTimeMs([&]() { processor->prune(current_time); }));
    timings.spawn.times.push_back(
      measureTimeMs([&]() { processor->spawn(detections, reverse_assignment); }));

    const auto total_end = Clock::now();
    auto total_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() /
      1000.0;
    timings.total.times.push_back(total_duration);

    autoware_perception_msgs::msg::TrackedObjects latest_tracked_objects;
    processor->getTrackedObjects(current_time, latest_tracked_objects);

    latest_tracked_objects.header.frame_id = "map";
    writer.write(static_tf_msg, "/tf", current_time);
    writer.write(
      toDetectedObjectsMsg(detections), "/perception/object_recognition/detection/objects",
      current_time);
    writer.write(
      latest_tracked_objects, "/perception/object_recognition/tracking/objects", current_time);
  }
  return timings;
}

FunctionTimings runIterations(
  int num_iterations, const ScenarioParams & config, bool print_frame_stats = false,
  bool write_bag = false)
{
  RosbagWriterHelper writer(write_bag);

  auto processor_config = createProcessorConfig();
  const auto associator_config = createAssociatorConfig();
  const auto input_channels_config = createInputChannelsConfig();

  auto processor = std::make_unique<autoware::multi_object_tracker::TrackerProcessor>(
    processor_config, associator_config, input_channels_config);
  TestBench simulator(config);
  simulator.initializeObjects();
  // Performance tracking for individual functions
  FunctionTimings timings;

  rclcpp::Clock clock;
  rclcpp::Time current_time = rclcpp::Time(clock.now(), RCL_ROS_TIME);
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  if (print_frame_stats) {
    printFrameStatsHeader();
  }
  for (int i = 0; i < num_iterations; ++i) {
    direct_assignment.clear();
    reverse_assignment.clear();
    // Advance simulation time (10Hz)
    current_time += 100ms;
    auto detections = simulator.generateDetections(current_time);
    detections = autoware::multi_object_tracker::uncertainty::modelUncertainty(detections);

    const auto total_start = Clock::now();

    // Individual function timing
    timings.predict.times.push_back(
      measureTimeMs([&]() { processor->predict(current_time, std::nullopt); }));
    timings.associate.times.push_back(measureTimeMs(
      [&]() { processor->associate(detections, direct_assignment, reverse_assignment); }));
    timings.update.times.push_back(
      measureTimeMs([&]() { processor->update(detections, direct_assignment); }));
    int num_trackers0 = processor->getListTracker().size();
    timings.prune.times.push_back(measureTimeMs([&]() { processor->prune(current_time); }));
    int num_trackers1 = processor->getListTracker().size();
    timings.spawn.times.push_back(
      measureTimeMs([&]() { processor->spawn(detections, reverse_assignment); }));
    int num_trackers2 = processor->getListTracker().size();

    int num_pruned = num_trackers0 - num_trackers1;
    int num_spawned = num_trackers2 - num_trackers1;
    const auto total_end = Clock::now();
    auto total_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(total_end - total_start).count() /
      1000.0;
    timings.total.times.push_back(total_duration);

    if (i % 10 == 0 && print_frame_stats) {
      printFrameStats(
        i, detections.objects.size(), num_trackers0, num_trackers2, num_pruned, num_spawned,
        timings);
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

void runAssociationTest()
{
  ScenarioParams params;
  params.num_lanes = 0;
  params.pedestrian_clusters = 0;
  params.cars_per_lane = 0;
  params.unknown_objects = 0;
  params.dropout_rate = 0.0f;                        // No dropout
  params.unknown_params.shape_change_prob = 0.0f;    // No shape change
  params.unknown_params.max_evolution_noise = 0.0f;  // No evolution noise

  FunctionTimings timings = runIterationsAssociation(500, params, true, true);
  std::cout << "Total time for all iterations: "
            << std::accumulate(timings.total.times.begin(), timings.total.times.end(), 0.0) << " ms"
            << std::endl;
}

void runPerformanceTest()
{
  const ScenarioParams params;
  FunctionTimings timings = runIterations(50, params, true, false);
  std::cout << "Total time for all iterations: "
            << std::accumulate(timings.total.times.begin(), timings.total.times.end(), 0.0) << " ms"
            << std::endl;

  timings.calculate();
  timings.printSummary();
}

void runPerformanceTestWithRosbag(const std::string & rosbag_path, bool write_bag = false)
{
  // === Setup ===
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<rclcpp::Node>("multi_object_tracker_test_node");
  const auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  const auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
  RosbagWriterHelper writer(write_bag);
  RosbagReaderHelper reader(rosbag_path);

  const std::string world_frame_id = "map";      // Assuming map is the world frame ID
  const std::string ego_frame_id = "base_link";  // Assuming base_link is the ego vehicle frame ID
  const auto odometry = std::make_shared<autoware::multi_object_tracker::Odometry>(
    *node, world_frame_id, ego_frame_id, true);

  auto processor_config = createProcessorConfig();
  const auto associator_config = createAssociatorConfig();
  auto input_channels_config = createInputChannelsConfig();

  auto processor = std::make_unique<autoware::multi_object_tracker::TrackerProcessor>(
    processor_config, associator_config, input_channels_config);

  // Create serialization objects
  rclcpp::Serialization<autoware_perception_msgs::msg::DetectedObjects> detection_serialization;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

  while (reader.hasNext()) {
    auto bag_message = reader.readNext();
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
        tf2::Transform tf_target2objects;
        tf2::fromMsg(tf_transform.transform, tf_target2objects);

        tf2::Transform tf_objects_world2objects;
        auto & pose = object.pose;
        auto & pose_cov = object.pose_covariance;
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
      // Print every 10 frames to reduce output frequency
      static int frame_count = 0;
      if (frame_count++ % 10 == 1) {
        std::cout << "Processed frame at " << time_str << " with "
                  << latest_tracked_objects.objects.size() << " tracked objects" << std::endl;
      }
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

// Configuration structure for profiling parameters
struct ProfileConfig
{
  std::string profile_name;
  int min_count;
  int max_count;
  int step;
  int iterations_per_count;
  float simulation_duration;
  std::function<void(ScenarioParams &, int)> config_updater;
};

// Common profiling function template
void profilePerformance(const ProfileConfig & config)
{
  const std::vector<std::string> performance_columns = {
    "TotalTime", "PredictTime", "AssociateTime", "UpdateTime", "PruneTime", "SpawnTime"};

  std::cout << "\n=== Performance (ms) vs " << config.profile_name << " Count (" << config.min_count
            << "-" << config.max_count << ") ===\n";

  // Print header
  std::cout << std::left << std::setw(15) << config.profile_name;
  for (const auto & name : performance_columns) {
    std::cout << "," << std::setw(12) << name;
  }
  std::cout << "\n";

  // Pre-calculate constants
  const int total_iterations = static_cast<int>(config.simulation_duration * 10.0f);
  const int expected_timings_size = config.iterations_per_count * total_iterations;

  // Main profiling loop
  for (int target_count = config.min_count; target_count <= config.max_count;
       target_count += config.step) {
    ScenarioParams params;
    config.config_updater(params, target_count);

    FunctionTimings total_timings;
    total_timings.reserve(expected_timings_size);

    // Timing accumulation
    for (int i = 0; i < config.iterations_per_count; ++i) {
      total_timings.accumulate(runIterations(total_iterations, params));
    }

    total_timings.calculate();

    std::cout << std::left << std::fixed << std::setprecision(3) << std::setw(15) << target_count;
    std::cout << "," << std::setw(12) << total_timings.total.avg << "," << std::setw(12)
              << total_timings.predict.avg << "," << std::setw(14) << total_timings.associate.avg
              << "," << std::setw(12) << total_timings.update.avg << "," << std::setw(12)
              << total_timings.prune.avg << "," << std::setw(12) << total_timings.spawn.avg << "\n";
  }
}

void profilePerformanceVsCarCount()
{
  profilePerformance(
    ProfileConfig{
      "Car",
      1,     // min_count
      1000,  // max_count
      5,     // step
      5,     // iterations_per_count
      5.0f,  // simulation_duration
      [](ScenarioParams & params, int target_count) {
        params.pedestrian_clusters = 0;  // No pedestrians in this profile
        params.pedestrians_per_cluster = 0;
        params.num_lanes = std::max(1, target_count / 20);
        params.cars_per_lane = std::max(1, target_count / params.num_lanes);
        params.car_spacing_mean = 10.0f * (1.0f + target_count / 500.0f);
        params.unknown_objects = 0;  // No unknown objects in this profile
      }});
}

void profilePerformanceVsPedestrianCount()
{
  profilePerformance(
    ProfileConfig{
      "Pedestrian",
      1,     // min_count
      1000,  // max_count
      5,     // step
      5,     // iterations_per_count
      5.0f,  // simulation_duration
      [](ScenarioParams & params, int target_count) {
        params.num_lanes = 0;  // No cars in this profile
        params.cars_per_lane = 0;
        params.pedestrian_clusters = std::max(1, target_count / 5);
        params.pedestrians_per_cluster = std::max(1, target_count / params.pedestrian_clusters);
        params.pedestrian_cluster_spacing = 30.0f * (1.0f + target_count / 200.0f);
        params.unknown_objects = 0;  // No unknown objects in this profile
      }});
}

void profilePerformanceVsUnknownObjectCount()
{
  profilePerformance(
    ProfileConfig{
      "Unknown",
      1,     // min_count
      1000,  // max_count
      5,     // step
      5,     // iterations_per_count
      5.0f,  // simulation_duration
      [](ScenarioParams & params, int target_count) {
        params.num_lanes = 0;  // No cars in this profile
        params.cars_per_lane = 0;
        params.pedestrian_clusters = 0;  // No pedestrians in this profile
        params.pedestrians_per_cluster = 0;
        params.unknown_objects = target_count;
      }});
}

class MultiObjectTrackerTest : public ::testing::Test
{
public:
  MultiObjectTrackerTest() = default;
  ~MultiObjectTrackerTest() override = default;
};

TEST_F(MultiObjectTrackerTest, DISABLED_PerformanceVsCarCount)
{
  // This test runs performance analysis with varying car counts
  profilePerformanceVsCarCount();
}

TEST_F(MultiObjectTrackerTest, DISABLED_PerformanceVsPedestrianCount)
{
  // This test runs performance analysis with varying pedestrian counts
  profilePerformanceVsPedestrianCount();
}

TEST_F(MultiObjectTrackerTest, DISABLED_PerformanceVsUnknownObjectCount)
{
  // This test runs performance analysis with varying unknown object counts
  profilePerformanceVsUnknownObjectCount();
}

TEST_F(MultiObjectTrackerTest, DISABLED_AssociationTest)  // NOLINT
{
  // This test checks the merging of unknown objects with existing cars
  // By default, this test enables recording into a rosbag; it is intended for local verification
  // only.
  runAssociationTest();
}

TEST_F(MultiObjectTrackerTest, SimulatedDataPerformanceTest)
{
  // This test runs performance analysis using simulated tracking data
  runPerformanceTest();
}

TEST_F(MultiObjectTrackerTest, RealDataRosbagPerformanceTest)
{
  // This test runs the tracker using a real rosbag for evaluation
  std::filesystem::path bag_root_dir = _SRC_RESOURCES_DIR_PATH;  // defined in CMakeLists.txt
  std::filesystem::path bag_dir = bag_root_dir / "test_data1";
  std::filesystem::path db3_file;

  for (const auto & entry : std::filesystem::directory_iterator(bag_dir)) {
    if (entry.path().extension() == ".db3") {
      db3_file = entry.path();
      break;
    }
  }

  ASSERT_FALSE(db3_file.empty()) << "No .db3 file found in " << bag_dir;

  runPerformanceTestWithRosbag(db3_file.string());
}
