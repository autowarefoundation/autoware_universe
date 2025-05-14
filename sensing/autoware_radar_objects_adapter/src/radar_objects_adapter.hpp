// Copyright 2025 The Autoware Contributors
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

#ifndef RADAR_OBJECTS_ADAPTER_HPP_
#define RADAR_OBJECTS_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_sensing_msgs/msg/radar_info.hpp>
#include <autoware_sensing_msgs/msg/radar_objects.hpp>

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
class RadarObjectsAdapter : public rclcpp::Node
{
public:
  explicit RadarObjectsAdapter(const rclcpp::NodeOptions & options);

private:
  void radar_cov_to_detection_pose_cov(
    const std::array<float, 6> & radar_pose_cov, const double orientation_std,
    std::array<double, 36> & pose_cov);

  void radar_cov_to_detection_twist_cov(
    const std::array<float, 6> & radar_twist_cov, const float yaw, const float yaw_rate_std,
    std::array<double, 36> & twist_cov);

  void radar_cov_to_detection_acceleration_cov(
    const std::array<float, 6> & radar_acceleration_cov, const float yaw,
    std::array<double, 36> & acceleration_cov);

  template <typename ObjectType>
  void populate_common_fields(
    const autoware_sensing_msgs::msg::RadarObject & input_object, ObjectType & output_object,
    const float yaw);

  void populate_classifications(
    const std::vector<autoware_sensing_msgs::msg::RadarClassification> & input_classifications,
    std::vector<autoware_perception_msgs::msg::ObjectClassification> & output_classifications);

  void objects_callback(const autoware_sensing_msgs::msg::RadarObjects & objects_msg);
  void parse_as_detections(const autoware_sensing_msgs::msg::RadarObjects & input_msg);
  void parse_as_tracks(const autoware_sensing_msgs::msg::RadarObjects & input_msg);

  void radar_info_callback(const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg);

  rclcpp::Subscription<autoware_sensing_msgs::msg::RadarObjects>::SharedPtr radar_objects_sub_;
  rclcpp::Subscription<autoware_sensing_msgs::msg::RadarInfo>::SharedPtr radar_info_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr detections_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr tracks_pub_;

  std::unordered_map<std::string, autoware_sensing_msgs::msg::RadarFieldInfo> field_info_map_;

  bool valid_radar_info_{false};
  std::array<std::uint8_t, sizeof(std::size_t)> topic_hash_code_;

  std::vector<std::string> required_attributes_;
  float default_position_z_;
  float default_velocity_z_;
  float default_acceleration_z_;
  float default_size_x_;
  float default_size_y_;
  float default_size_z_;

  bool position_z_available_;
  bool velocity_z_available_;
  bool acceleration_z_available_;
  bool size_x_available_;
  bool size_y_available_;
  bool size_z_available_;

  bool orientation_std_available_;
  bool orientation_rate_available_;
  bool orientation_rate_std_available_;
};
}  // namespace autoware

#endif  // RADAR_OBJECTS_ADAPTER_HPP_
