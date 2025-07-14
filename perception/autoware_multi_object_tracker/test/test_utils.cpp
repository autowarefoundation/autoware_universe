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
#include "test_utils.hpp"

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <array>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
namespace
{

constexpr size_t uuid_size = 16;
constexpr size_t hash_size = sizeof(size_t);

}  // namespace
// Convert string ID to 16-byte UUID
std::array<uint8_t, uuid_size> stringToUUID(const std::string & id)
{
  std::array<uint8_t, uuid_size> uuid{};
  std::size_t hash = std::hash<std::string>{}(id);
  // Distribute hash across UUID bytes
  std::memcpy(uuid.data(), &hash, hash_size);
  std::memcpy(uuid.data() + hash_size, &hash, hash_size);  // Repeat hash
  return uuid;
}

void printPerformanceStats(const std::string & name, const PerformanceStats & stats)
{
  std::cout << std::left << std::setw(10) << name << " - Avg: " << std::setw(8) << stats.avg
            << " ms" << " Min: " << std::setw(8) << stats.min << " ms" << " Max: " << std::setw(8)
            << stats.max << " ms" << std::endl;
}

void printFrameStats(
  int frame, const autoware::multi_object_tracker::types::DynamicObjectList & detections,
  const FunctionTimings & timings)
{
  std::cout << std::left << "[Frame " << std::setw(5) << frame << "] "
            << "Objects: " << std::setw(3) << detections.objects.size()
            << " | Total: " << std::setw(6) << timings.total.times.back() << " ms"
            << " | Predict: " << std::setw(6) << timings.predict.times.back() << " ms"
            << " | Associate: " << std::setw(6) << timings.associate.times.back() << " ms"
            << " | Update: " << std::setw(6) << timings.update.times.back() << " ms"
            << " | Prune: " << std::setw(6) << timings.prune.times.back() << " ms"
            << " | Spawn: " << std::setw(6) << timings.spawn.times.back() << " ms" << std::endl;
}

autoware_perception_msgs::msg::DetectedObject toDetectedObject(
  const autoware::multi_object_tracker::types::DynamicObject & dyn_object)
{
  autoware_perception_msgs::msg::DetectedObject det_object;

  // classification
  det_object.classification = dyn_object.classification;

  // pose and covariance
  det_object.kinematics.pose_with_covariance.pose = dyn_object.pose;
  det_object.kinematics.pose_with_covariance.covariance = dyn_object.pose_covariance;

  // twist and covariance
  det_object.kinematics.twist_with_covariance.twist = dyn_object.twist;
  det_object.kinematics.twist_with_covariance.covariance = dyn_object.twist_covariance;

  det_object.kinematics.has_position_covariance = dyn_object.kinematics.has_position_covariance;

  // orientation availability
  using DetKinematics = autoware_perception_msgs::msg::DetectedObjectKinematics;
  using OrientationAvailability = autoware::multi_object_tracker::types::OrientationAvailability;
  switch (dyn_object.kinematics.orientation_availability) {
    case OrientationAvailability::UNAVAILABLE:
      det_object.kinematics.orientation_availability = DetKinematics::UNAVAILABLE;
      break;
    case OrientationAvailability::SIGN_UNKNOWN:
      det_object.kinematics.orientation_availability = DetKinematics::SIGN_UNKNOWN;
      break;
    case OrientationAvailability::AVAILABLE:
      det_object.kinematics.orientation_availability = DetKinematics::AVAILABLE;
      break;
    default:
      det_object.kinematics.orientation_availability = DetKinematics::UNAVAILABLE;
      break;
  }

  det_object.kinematics.has_twist = dyn_object.kinematics.has_twist;
  det_object.kinematics.has_twist_covariance = dyn_object.kinematics.has_twist_covariance;

  // shape
  det_object.shape = dyn_object.shape;

  // existence probability
  det_object.existence_probability = dyn_object.existence_probability;

  return det_object;
}

// Conversion from DynamicObjectList to TrackedObjects message
autoware_perception_msgs::msg::DetectedObjects toDetectedObjectsMsg(
  const autoware::multi_object_tracker::types::DynamicObjectList & dyn_objects)
{
  autoware_perception_msgs::msg::DetectedObjects detected_objects;
  detected_objects.header = dyn_objects.header;

  detected_objects.objects.reserve(dyn_objects.objects.size());
  for (const auto & dyn_object : dyn_objects.objects) {
    detected_objects.objects.emplace_back(toDetectedObject(dyn_object));
  }

  return detected_objects;
}

// ==========================
// RosbagWriterHelper
// ==========================

RosbagWriterHelper::RosbagWriterHelper(bool enabled) : enabled_(enabled)
{
  if (!enabled_) return;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Generate timestamped bag name
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "tracking_results_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  bag_name_ = ss.str();
  std::cout << "Writing results to rosbag: " << bag_name_ << std::endl;
  writer_->open(bag_name_);

  std::cout << "Rosbag opened successfully." << std::endl;
  // Register topics
  const auto serialization_format = rmw_get_serialization_format();
  writer_->create_topic(
    {"/perception/object_recognition/detection/objects",
     "autoware_perception_msgs/msg/DetectedObjects", serialization_format, ""});
  writer_->create_topic(
    {"/perception/object_recognition/tracking/objects",
     "autoware_perception_msgs/msg/TrackedObjects", serialization_format, ""});
  std::cout << "Topics registered successfully." << std::endl;
}

RosbagWriterHelper::~RosbagWriterHelper()
{
  if (enabled_ && writer_) {
    writer_->close();
    const auto absolute_path = std::filesystem::absolute(bag_name_);
    std::cout << "Run rosbag by:\nros2 bag play " << absolute_path << std::endl;
  }
}
