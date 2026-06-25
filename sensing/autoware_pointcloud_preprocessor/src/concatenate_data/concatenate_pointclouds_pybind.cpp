// Copyright 2025 TIER IV, Inc.
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

// Python bindings for the (ROS-runtime-free) point cloud concatenation core. ROS messages cross the
// boundary as CDR-serialized bytes (rclpy::serialize_message on the Python side), which keeps the
// binding free of any C++/Python message-conversion machinery and fully deterministic.

#include "autoware/pointcloud_preprocessor/concatenate_data/collector_info.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace
{
using autoware::pointcloud_preprocessor::AdvancedCollectorInfo;
using autoware::pointcloud_preprocessor::AdvancedMatchingPolicy;
using autoware::pointcloud_preprocessor::CandidateCollectorState;
using autoware::pointcloud_preprocessor::CollectorInfoBase;
using autoware::pointcloud_preprocessor::CollectorReference;
using autoware::pointcloud_preprocessor::CombineCloudHandler;
using autoware::pointcloud_preprocessor::ConcatenatedCloudResult;
using autoware::pointcloud_preprocessor::IncomingCloudInfo;
using autoware::pointcloud_preprocessor::MatchingPolicy;
using autoware::pointcloud_preprocessor::MotionCompensationStatus;
using autoware::pointcloud_preprocessor::NaiveCollectorInfo;
using autoware::pointcloud_preprocessor::NaiveMatchingPolicy;

// Copyable wrapper around the polymorphic matching core so it can be returned by value from the
// bound factory functions and held by the Python Concatenator.
class MatchingPolicyWrapper
{
public:
  static MatchingPolicyWrapper make_naive()
  {
    MatchingPolicyWrapper wrapper;
    wrapper.impl_ = std::make_shared<NaiveMatchingPolicy>();
    return wrapper;
  }

  static MatchingPolicyWrapper make_advanced(
    const std::vector<std::string> & input_topics,
    const std::vector<double> & lidar_timestamp_offsets,
    const std::vector<double> & lidar_timestamp_noise_window)
  {
    MatchingPolicyWrapper wrapper;
    wrapper.impl_ = std::make_shared<AdvancedMatchingPolicy>(
      input_topics, lidar_timestamp_offsets, lidar_timestamp_noise_window);
    return wrapper;
  }

  [[nodiscard]] std::optional<std::size_t> match(
    const std::vector<CandidateCollectorState> & collectors, const IncomingCloudInfo & params) const
  {
    return impl_->match(collectors, params);
  }

  [[nodiscard]] CollectorReference reference_for(const IncomingCloudInfo & params) const
  {
    return impl_->reference_for(params);
  }

private:
  std::shared_ptr<MatchingPolicy> impl_;
};

template <typename MsgT>
MsgT deserialize(const py::bytes & data)
{
  const auto buffer = static_cast<std::string>(data);
  rclcpp::SerializedMessage serialized(buffer.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, buffer.data(), buffer.size());
  rcl_msg.buffer_length = buffer.size();

  MsgT msg;
  rclcpp::Serialization<MsgT>().deserialize_message(&serialized, &msg);
  return msg;
}

template <typename MsgT>
py::bytes serialize(const MsgT & msg)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<MsgT>().serialize_message(&msg, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return py::bytes(reinterpret_cast<const char *>(rcl_msg.buffer), rcl_msg.buffer_length);
}

// Holds the ROS-free combine handler and converts serialized messages to/from the core's C++ types.
// The collection/matching/timeout orchestration is done by the Python Concatenator on top of this.
class CombineCloudHandlerWrapper
{
public:
  CombineCloudHandlerWrapper(
    const std::vector<std::string> & input_topics, const std::string & output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud, const std::string & matching_strategy_name)
  : handler_(
      input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
      keep_input_frame_in_synchronized_pointcloud, matching_strategy_name)
  {
  }

  void set_transform(const py::bytes & sensor_to_output_frame)
  {
    handler_.set_transform(
      deserialize<geometry_msgs::msg::TransformStamped>(sensor_to_output_frame));
  }

  void process_twist(const py::bytes & twist)
  {
    handler_.process_twist(
      std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(
        deserialize<geometry_msgs::msg::TwistWithCovarianceStamped>(twist)));
  }

  void process_odometry(const py::bytes & odometry)
  {
    handler_.process_odometry(
      std::make_shared<nav_msgs::msg::Odometry>(deserialize<nav_msgs::msg::Odometry>(odometry)));
  }

  // Combine a group of clouds (topic -> CDR bytes). When reference_timestamp is provided, an
  // AdvancedCollectorInfo is attached so the strategy window is recorded in the info message.
  // Returns (concatenated_cloud_bytes, info_bytes, topic->transformed_cloud_bytes | None,
  //          status, topic->original_stamp_seconds, dropped_frames_missing_transform).
  py::tuple combine_pointclouds(
    const std::unordered_map<std::string, py::bytes> & topic_to_cloud_bytes,
    std::optional<double> reference_timestamp, std::optional<double> noise_window)
  {
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr>
      topic_to_cloud_map;
    topic_to_cloud_map.reserve(topic_to_cloud_bytes.size());
    for (const auto & [topic, bytes] : topic_to_cloud_bytes) {
      topic_to_cloud_map[topic] = std::make_shared<sensor_msgs::msg::PointCloud2>(
        deserialize<sensor_msgs::msg::PointCloud2>(bytes));
    }

    std::shared_ptr<CollectorInfoBase> collector_info;
    if (reference_timestamp.has_value()) {
      collector_info =
        std::make_shared<AdvancedCollectorInfo>(*reference_timestamp, noise_window.value_or(0.0));
    } else {
      collector_info = std::make_shared<NaiveCollectorInfo>();
    }

    ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> result =
      handler_.combine_pointclouds(topic_to_cloud_map, collector_info);

    py::object concatenated_cloud_bytes = py::none();
    if (result.concatenate_cloud_ptr) {
      concatenated_cloud_bytes = serialize(*result.concatenate_cloud_ptr);
    }

    py::object info_bytes = py::none();
    if (result.concatenation_info_ptr) {
      info_bytes = serialize(*result.concatenation_info_ptr);
    }

    py::object transformed_clouds = py::none();
    if (result.topic_to_transformed_cloud_map) {
      py::dict transformed;
      for (const auto & [topic, cloud_ptr] : *result.topic_to_transformed_cloud_map) {
        transformed[py::str(topic)] = serialize(*cloud_ptr);
      }
      transformed_clouds = std::move(transformed);
    }

    py::dict original_stamps;
    for (const auto & [topic, stamp] : result.topic_to_original_stamp_map) {
      original_stamps[py::str(topic)] = stamp;
    }

    return py::make_tuple(
      concatenated_cloud_bytes, info_bytes, transformed_clouds, result.motion_compensation_status,
      original_stamps, result.dropped_frames_missing_transform);
  }

private:
  CombineCloudHandler<sensor_msgs::msg::PointCloud2> handler_;
};
}  // namespace

PYBIND11_MODULE(_concatenate_pointclouds_pybind, m)
{
  m.doc() =
    "Python bindings for the autoware_pointcloud_preprocessor point cloud concatenation core.";

  py::class_<MotionCompensationStatus>(m, "MotionCompensationStatus")
    .def_readonly("no_twist_available", &MotionCompensationStatus::no_twist_available)
    .def_readonly("twist_time_gap_too_large", &MotionCompensationStatus::twist_time_gap_too_large);

  py::class_<CombineCloudHandlerWrapper>(m, "CombineCloudHandler")
    .def(
      py::init<
        const std::vector<std::string> &, const std::string &, bool, bool, bool,
        const std::string &>(),
      py::arg("input_topics"), py::arg("output_frame"), py::arg("is_motion_compensated") = true,
      py::arg("publish_synchronized_pointcloud") = false,
      py::arg("keep_input_frame_in_synchronized_pointcloud") = false,
      py::arg("matching_strategy") = "naive")
    .def(
      "set_transform", &CombineCloudHandlerWrapper::set_transform,
      py::arg("sensor_to_output_frame"))
    .def("process_twist", &CombineCloudHandlerWrapper::process_twist, py::arg("twist"))
    .def("process_odometry", &CombineCloudHandlerWrapper::process_odometry, py::arg("odometry"))
    .def(
      "combine_pointclouds", &CombineCloudHandlerWrapper::combine_pointclouds,
      py::arg("topic_to_cloud_bytes"), py::arg("reference_timestamp") = std::nullopt,
      py::arg("noise_window") = std::nullopt);

  // ---- Matching core (shared with the ROS node) ----
  py::class_<IncomingCloudInfo>(m, "IncomingCloudInfo")
    .def(
      py::init(
        [](const std::string & topic_name, double cloud_timestamp, double cloud_arrival_time) {
          return IncomingCloudInfo{topic_name, cloud_timestamp, cloud_arrival_time};
        }),
      py::arg("topic_name"), py::arg("cloud_timestamp"), py::arg("cloud_arrival_time"));

  py::class_<CandidateCollectorState>(m, "CandidateCollectorState")
    .def(
      py::init([](double reference_time, double noise_window, bool has_topic) {
        return CandidateCollectorState{reference_time, noise_window, has_topic};
      }),
      py::arg("reference_time"), py::arg("noise_window"), py::arg("has_topic"));

  py::class_<CollectorReference>(m, "CollectorReference")
    .def_readonly("reference_time", &CollectorReference::reference_time)
    .def_readonly("noise_window", &CollectorReference::noise_window);

  py::class_<MatchingPolicyWrapper>(m, "MatchingPolicy")
    .def_static("make_naive", &MatchingPolicyWrapper::make_naive)
    .def_static(
      "make_advanced", &MatchingPolicyWrapper::make_advanced, py::arg("input_topics"),
      py::arg("lidar_timestamp_offsets"), py::arg("lidar_timestamp_noise_window"))
    .def("match", &MatchingPolicyWrapper::match, py::arg("collectors"), py::arg("params"))
    .def("reference_for", &MatchingPolicyWrapper::reference_for, py::arg("params"));
}
