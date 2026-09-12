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

// Python bindings for the (ROS-runtime-free) point cloud concatenation core. The binding exposes
// only the high-level concatenator: matching, combining, and diagnostics all run in C++, so Python
// holds no concatenation logic. ROS messages cross the boundary as CDR-serialized bytes
// (rclpy::serialize_message on the Python side), which keeps the binding free of any C++/Python
// message-conversion machinery and fully deterministic.

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_concatenator.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_diagnostics.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
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
using autoware::pointcloud_preprocessor::build_diagnostic_status;
using autoware::pointcloud_preprocessor::ConcatenatedFrameStatus;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsOptions;
using autoware::pointcloud_preprocessor::parse_matching_strategy;
using ConcatenatedFrame =
  autoware::pointcloud_preprocessor::ConcatenatedFrame<sensor_msgs::msg::PointCloud2>;
using CloudConcatenator =
  autoware::pointcloud_preprocessor::CloudConcatenator<sensor_msgs::msg::PointCloud2>;

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

// One emitted frame, exposed to Python with its clouds as CDR bytes. Keeps the C++ frame alive so
// diagnostics can be built from it directly on the C++ side.
class FrameWrapper
{
public:
  explicit FrameWrapper(ConcatenatedFrame && frame)
  : frame_(std::make_shared<const ConcatenatedFrame>(std::move(frame)))
  {
  }

  [[nodiscard]] std::string status() const
  {
    return frame_->status == ConcatenatedFrameStatus::kComplete ? "complete" : "timeout";
  }

  [[nodiscard]] py::object concatenated_cloud() const
  {
    if (!frame_->result.concatenate_cloud_ptr) return py::none();
    return serialize(*frame_->result.concatenate_cloud_ptr);
  }

  [[nodiscard]] py::object concatenation_info() const
  {
    if (!frame_->result.concatenation_info_ptr) return py::none();
    return serialize(*frame_->result.concatenation_info_ptr);
  }

  [[nodiscard]] py::object transformed_clouds() const
  {
    if (!frame_->result.topic_to_transformed_cloud_map) return py::none();
    py::dict transformed;
    for (const auto & [topic, cloud_ptr] : *frame_->result.topic_to_transformed_cloud_map) {
      transformed[py::str(topic)] = serialize(*cloud_ptr);
    }
    return transformed;
  }

  [[nodiscard]] bool no_twist_available() const
  {
    return frame_->result.motion_compensation_status.no_twist_available;
  }

  [[nodiscard]] bool twist_time_gap_too_large() const
  {
    return frame_->result.motion_compensation_status.twist_time_gap_too_large;
  }

  [[nodiscard]] const std::unordered_map<std::string, double> & topic_to_original_stamp() const
  {
    return frame_->result.topic_to_original_stamp_map;
  }

  [[nodiscard]] const std::vector<std::string> & dropped_frames_missing_transform() const
  {
    return frame_->result.dropped_frames_missing_transform;
  }

  [[nodiscard]] double reference_time() const { return frame_->reference_time; }
  [[nodiscard]] double noise_window() const { return frame_->noise_window; }
  [[nodiscard]] double first_arrival_time() const { return frame_->first_arrival_time; }

  [[nodiscard]] py::bytes build_diagnostics(
    const std::vector<std::string> & input_topics, const std::string & node_name,
    const std::optional<std::string> & diagnostic_name,
    const std::optional<double> & processing_time_ms, const std::optional<double> & now_sec,
    bool drop_previous_but_late) const
  {
    ConcatenationDiagnosticsOptions options;
    options.node_name = node_name;
    options.diagnostic_name = diagnostic_name.value_or("");
    options.processing_time_ms = processing_time_ms;
    options.now_sec = now_sec;
    options.drop_previous_but_late = drop_previous_but_late;
    return serialize(build_diagnostic_status(*frame_, input_topics, options));
  }

private:
  std::shared_ptr<const ConcatenatedFrame> frame_;
};

// Owns the C++ concatenator and converts serialized messages at the boundary. All matching,
// combining, and diagnostics logic lives in the core.
class ConcatenatorWrapper
{
public:
  ConcatenatorWrapper(
    const std::vector<std::string> & input_topics, const std::string & output_frame,
    double timeout_sec, bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud, const std::string & matching_strategy,
    const std::optional<std::vector<double>> & lidar_timestamp_offsets,
    const std::optional<std::vector<double>> & lidar_timestamp_noise_window)
  : concatenator_(
      input_topics, output_frame, timeout_sec, is_motion_compensated,
      publish_synchronized_pointcloud, keep_input_frame_in_synchronized_pointcloud,
      parse_matching_strategy(matching_strategy), lidar_timestamp_offsets,
      lidar_timestamp_noise_window)
  {
  }

  void set_transform(const py::bytes & sensor_to_output_frame)
  {
    concatenator_.set_transform(
      deserialize<geometry_msgs::msg::TransformStamped>(sensor_to_output_frame));
  }

  void process_twist(const py::bytes & twist)
  {
    concatenator_.process_twist(
      std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(
        deserialize<geometry_msgs::msg::TwistWithCovarianceStamped>(twist)));
  }

  void process_odometry(const py::bytes & odometry)
  {
    concatenator_.process_odometry(
      std::make_shared<nav_msgs::msg::Odometry>(deserialize<nav_msgs::msg::Odometry>(odometry)));
  }

  [[nodiscard]] std::vector<FrameWrapper> process_cloud(
    const std::string & topic, const py::bytes & cloud, double arrival_time)
  {
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(
      deserialize<sensor_msgs::msg::PointCloud2>(cloud));
    return wrap(concatenator_.process_cloud(topic, std::move(cloud_msg), arrival_time));
  }

  [[nodiscard]] std::vector<FrameWrapper> close_expired_collectors(double now_sec)
  {
    return wrap(concatenator_.close_expired_collectors(now_sec));
  }

  [[nodiscard]] std::vector<FrameWrapper> flush() { return wrap(concatenator_.flush()); }

private:
  static std::vector<FrameWrapper> wrap(std::vector<ConcatenatedFrame> && frames)
  {
    std::vector<FrameWrapper> wrapped;
    wrapped.reserve(frames.size());
    for (auto & frame : frames) {
      wrapped.emplace_back(std::move(frame));
    }
    return wrapped;
  }

  CloudConcatenator concatenator_;
};
}  // namespace

PYBIND11_MODULE(_concatenate_pointclouds_pybind, m)
{
  m.doc() =
    "Python bindings for the autoware_pointcloud_preprocessor point cloud concatenation core.";

  py::class_<FrameWrapper>(m, "ConcatenatedFrame")
    .def_property_readonly("status", &FrameWrapper::status)
    .def_property_readonly("concatenated_cloud", &FrameWrapper::concatenated_cloud)
    .def_property_readonly("concatenation_info", &FrameWrapper::concatenation_info)
    .def_property_readonly("transformed_clouds", &FrameWrapper::transformed_clouds)
    .def_property_readonly("no_twist_available", &FrameWrapper::no_twist_available)
    .def_property_readonly("twist_time_gap_too_large", &FrameWrapper::twist_time_gap_too_large)
    .def_property_readonly("topic_to_original_stamp", &FrameWrapper::topic_to_original_stamp)
    .def_property_readonly(
      "dropped_frames_missing_transform", &FrameWrapper::dropped_frames_missing_transform)
    .def_property_readonly("reference_time", &FrameWrapper::reference_time)
    .def_property_readonly("noise_window", &FrameWrapper::noise_window)
    .def_property_readonly("first_arrival_time", &FrameWrapper::first_arrival_time)
    .def(
      "build_diagnostics", &FrameWrapper::build_diagnostics, py::arg("input_topics"),
      py::arg("node_name"), py::arg("diagnostic_name") = std::nullopt,
      py::arg("processing_time_ms") = std::nullopt, py::arg("now") = std::nullopt,
      py::arg("drop_previous_but_late") = false);

  py::class_<ConcatenatorWrapper>(m, "Concatenator")
    .def(
      py::init<
        const std::vector<std::string> &, const std::string &, double, bool, bool, bool,
        const std::string &, const std::optional<std::vector<double>> &,
        const std::optional<std::vector<double>> &>(),
      py::arg("input_topics"), py::arg("output_frame"), py::arg("timeout_sec"),
      py::arg("is_motion_compensated") = true, py::arg("publish_synchronized_pointcloud") = false,
      py::arg("keep_input_frame_in_synchronized_pointcloud") = false,
      py::arg("matching_strategy") = "naive", py::arg("lidar_timestamp_offsets") = std::nullopt,
      py::arg("lidar_timestamp_noise_window") = std::nullopt)
    .def("set_transform", &ConcatenatorWrapper::set_transform, py::arg("sensor_to_output_frame"))
    .def("process_twist", &ConcatenatorWrapper::process_twist, py::arg("twist"))
    .def("process_odometry", &ConcatenatorWrapper::process_odometry, py::arg("odometry"))
    .def(
      "process_cloud", &ConcatenatorWrapper::process_cloud, py::arg("topic"), py::arg("cloud"),
      py::arg("arrival_time"))
    .def("close_expired_collectors", &ConcatenatorWrapper::close_expired_collectors, py::arg("now"))
    .def("flush", &ConcatenatorWrapper::flush);
}
