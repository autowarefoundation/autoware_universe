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
// This file contains utility functions and structures for testing the Autoware multi-object
// tracker.
#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <autoware/multi_object_tracker/object_model/types.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
// Convert string ID to 16-byte UUID
std::array<uint8_t, 16> stringToUUID(const std::string & id);

struct PerformanceStats
{
  std::vector<double> times;
  double total = 0.0;
  double avg = 0.0;
  double min = 0.0;
  double max = 0.0;

  void calculate()
  {
    if (times.empty()) return;
    total = std::accumulate(times.begin(), times.end(), 0.0);
    avg = total / times.size();
    min = *std::min_element(times.begin(), times.end());
    max = *std::max_element(times.begin(), times.end());
  }
};

struct FunctionTimings
{
  PerformanceStats predict;
  PerformanceStats associate;
  PerformanceStats update;
  PerformanceStats prune;
  PerformanceStats spawn;
  PerformanceStats total;
  void accumulate(const FunctionTimings & other)
  {
    // Accumulate timings from another FunctionTimings object
    predict.times.insert(
      predict.times.end(), other.predict.times.begin(), other.predict.times.end());
    associate.times.insert(
      associate.times.end(), other.associate.times.begin(), other.associate.times.end());
    update.times.insert(update.times.end(), other.update.times.begin(), other.update.times.end());
    prune.times.insert(prune.times.end(), other.prune.times.begin(), other.prune.times.end());
    spawn.times.insert(spawn.times.end(), other.spawn.times.begin(), other.spawn.times.end());
    total.times.insert(total.times.end(), other.total.times.begin(), other.total.times.end());
  }
  void calculate()
  {
    predict.calculate();
    associate.calculate();
    update.calculate();
    prune.calculate();
    spawn.calculate();
    total.calculate();
  }
};

void printPerformanceStats(const std::string & name, const PerformanceStats & stats);

void printFrameStats(
  int frame, const autoware::multi_object_tracker::types::DynamicObjectList & detections,
  const FunctionTimings & timings);

autoware_perception_msgs::msg::DetectedObjects toDetectedObjectsMsg(
  const autoware::multi_object_tracker::types::DynamicObjectList & dyn_objects);

class RosbagWriterHelper
{
public:
  explicit RosbagWriterHelper(bool enabled, const std::string & storage_format = "sqlite3");
  ~RosbagWriterHelper();

  template <typename T>
  void write(const T & msg, const std::string & topic_name, const rclcpp::Time & time)
  {
    if (enabled_ && writer_) {
      writer_->write(msg, topic_name, time);
    }
  }

private:
  bool enabled_;
  std::string bag_name_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

class RosbagReaderHelper
{
public:
  explicit RosbagReaderHelper(const std::string & path);

  bool hasNext();
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> readNext();

private:
  rosbag2_cpp::Reader reader_;
};

#endif  // TEST_UTILS_HPP_
