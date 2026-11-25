// Copyright 2026 TIER IV, Inc.
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

// Python bindings for CloudConcatenator. Messages cross the boundary as CDR bytes
// (rclpy.serialization on the Python side), so the binding needs no message conversion code, and
// all concatenation logic stays in the core.

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_concatenator.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/matching_strategy_type.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace
{
using autoware::pointcloud_preprocessor::build_diagnostic_status;
using autoware::pointcloud_preprocessor::ConcatenatedFrameStatus;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsOptions;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsSummary;
using autoware::pointcloud_preprocessor::LateCloudPolicy;
using autoware::pointcloud_preprocessor::MotionCompensationStatus;
using autoware::pointcloud_preprocessor::parse_matching_strategy;
using autoware::pointcloud_preprocessor::PointFormat;
using autoware::pointcloud_preprocessor::ReferenceWindow;
using autoware::pointcloud_preprocessor::summarize_frame;
using ConcatenatedFrame =
  autoware::pointcloud_preprocessor::ConcatenatedFrame<sensor_msgs::msg::PointCloud2>;
using CloudConcatenator =
  autoware::pointcloud_preprocessor::CloudConcatenator<sensor_msgs::msg::PointCloud2>;
using InputTopics = std::shared_ptr<const std::vector<std::string>>;

PointFormat parse_output_point_type(const std::string & output_point_type)
{
  // The same values as the node's output_point_type parameter.
  if (output_point_type == "XYZIRC") return PointFormat::XYZIRC;
  if (output_point_type == "XYZIRCT") return PointFormat::XYZIRCT;
  throw std::invalid_argument(
    "output_point_type must be 'XYZIRC' or 'XYZIRCT', got '" + output_point_type + "'");
}

template <typename MsgT>
MsgT deserialize(const py::bytes & data)
{
  const std::string_view buffer = data;
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

// What build_diagnostic_status() needs from a frame: a few scalars and strings plus the input
// topics, so Python can keep it (instead of the frame and its clouds) to build diagnostics later.
class FrameDiagnostics
{
public:
  FrameDiagnostics(ConcatenationDiagnosticsSummary summary, InputTopics input_topics)
  : summary_(std::move(summary)), input_topics_(std::move(input_topics))
  {
  }

  [[nodiscard]] py::bytes build(
    const std::optional<std::string> & node_name,
    const std::optional<std::string> & diagnostic_name,
    const std::optional<double> & processing_time_ms, const std::optional<double> & now_sec,
    bool drop_previous_but_late) const
  {
    ConcatenationDiagnosticsOptions options;
    if (node_name) options.node_name = *node_name;
    if (diagnostic_name) options.diagnostic_name = *diagnostic_name;
    options.processing_time_ms = processing_time_ms;
    options.now_sec = now_sec;
    options.drop_previous_but_late = drop_previous_but_late;
    return serialize(build_diagnostic_status(summary_, *input_topics_, options));
  }

  // Pickle support (so frames can be copied or sent to worker processes).
  using State = std::tuple<
    double, bool, std::optional<double>, std::optional<std::pair<double, double>>,
    std::unordered_map<std::string, double>, std::vector<std::string>, std::vector<std::string>,
    std::vector<std::string>>;

  [[nodiscard]] State state() const
  {
    std::optional<std::pair<double, double>> window;
    if (summary_.reference_window) {
      window =
        std::make_pair(summary_.reference_window->time, summary_.reference_window->noise_window);
    }
    return State{
      summary_.concatenated_cloud_timestamp_sec,
      summary_.is_concatenated_cloud_empty,
      summary_.first_arrival_time,
      window,
      summary_.topic_to_original_stamp,
      summary_.topics_missing_transform,
      summary_.frames_missing_transform,
      *input_topics_};
  }

  static FrameDiagnostics from_state(const State & state)
  {
    ConcatenationDiagnosticsSummary summary;
    summary.concatenated_cloud_timestamp_sec = std::get<0>(state);
    summary.is_concatenated_cloud_empty = std::get<1>(state);
    summary.first_arrival_time = std::get<2>(state);
    if (const auto & window = std::get<3>(state)) {
      summary.reference_window = ReferenceWindow{window->first, window->second};
    }
    summary.topic_to_original_stamp = std::get<4>(state);
    summary.topics_missing_transform = std::get<5>(state);
    summary.frames_missing_transform = std::get<6>(state);
    return FrameDiagnostics(
      std::move(summary), std::make_shared<const std::vector<std::string>>(std::get<7>(state)));
  }

private:
  ConcatenationDiagnosticsSummary summary_;
  InputTopics input_topics_;
};

// One emitted frame. It owns the (move-only) C++ frame while Python reads its clouds out; Python
// keeps only `diagnostics` afterwards, so the C++ clouds are freed once the frame is converted.
class FrameWrapper
{
public:
  FrameWrapper(ConcatenatedFrame && frame, InputTopics input_topics)
  : frame_(std::make_shared<const ConcatenatedFrame>(std::move(frame))),
    diagnostics_(summarize_frame(*frame_), std::move(input_topics))
  {
  }

  [[nodiscard]] ConcatenatedFrameStatus status() const { return frame_->status; }

  // Frames always carry a cloud and its info: the core never emits an empty collector.
  [[nodiscard]] py::bytes concatenated_cloud() const
  {
    return serialize(*frame_->result.concatenate_cloud_ptr);
  }

  [[nodiscard]] py::bytes concatenation_info() const
  {
    return serialize(*frame_->result.concatenation_info_ptr);
  }

  // None unless publish_synchronized_pointcloud is set.
  [[nodiscard]] py::object transformed_clouds() const
  {
    if (!frame_->result.topic_to_transformed_cloud_map) return py::none();
    py::dict transformed;
    for (const auto & [topic, cloud_ptr] : *frame_->result.topic_to_transformed_cloud_map) {
      transformed[py::str(topic)] = serialize(*cloud_ptr);
    }
    return transformed;
  }

  [[nodiscard]] MotionCompensationStatus motion_compensation_status() const
  {
    return frame_->result.motion_compensation_status;
  }

  [[nodiscard]] const std::unordered_map<std::string, double> & topic_to_original_stamp() const
  {
    return frame_->result.topic_to_original_stamp_map;
  }

  [[nodiscard]] py::dict dropped_sources_missing_transform() const
  {
    py::dict dropped;
    for (const auto & source : frame_->result.dropped_sources_missing_transform) {
      dropped[py::str(source.topic)] = source.frame_id;
    }
    return dropped;
  }

  [[nodiscard]] double reference_time() const { return frame_->reference_time; }
  [[nodiscard]] double noise_window() const { return frame_->noise_window; }
  [[nodiscard]] double first_arrival_time() const { return frame_->first_arrival_time; }
  // By value: a reference would keep this wrapper, and with it the C++ clouds, alive in Python.
  [[nodiscard]] FrameDiagnostics diagnostics() const { return diagnostics_; }

private:
  std::shared_ptr<const ConcatenatedFrame> frame_;
  FrameDiagnostics diagnostics_;
};

class ConcatenatorWrapper
{
public:
  ConcatenatorWrapper(
    const std::vector<std::string> & input_topics, const std::string & output_frame,
    double timeout_sec, bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud, const std::string & matching_strategy,
    const std::optional<std::vector<double>> & lidar_timestamp_offsets,
    const std::optional<std::vector<double>> & lidar_timestamp_noise_window,
    std::size_t max_open_collectors, bool reject_late_clouds, double replay_jump_sec,
    const std::string & output_point_type)
  : input_topics_(std::make_shared<const std::vector<std::string>>(input_topics)),
    concatenator_(
      input_topics, output_frame, parse_output_point_type(output_point_type), timeout_sec,
      is_motion_compensated, publish_synchronized_pointcloud,
      keep_input_frame_in_synchronized_pointcloud, parse_matching_strategy(matching_strategy),
      lidar_timestamp_offsets, lidar_timestamp_noise_window, max_open_collectors,
      LateCloudPolicy{reject_late_clouds, replay_jump_sec})
  {
  }

  void set_transform(const py::bytes & sensor_to_output_frame)
  {
    const auto transform =
      deserialize<geometry_msgs::msg::TransformStamped>(sensor_to_output_frame);
    if (!concatenator_.set_transform(transform)) {
      throw std::invalid_argument(
        "rejected transform '" + transform.header.frame_id + "' -> '" + transform.child_frame_id +
        "': header.frame_id must be the output frame and child_frame_id the sensor frame");
    }
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
  std::vector<FrameWrapper> wrap(std::vector<ConcatenatedFrame> && frames) const
  {
    std::vector<FrameWrapper> wrapped;
    wrapped.reserve(frames.size());
    for (auto & frame : frames) {
      wrapped.emplace_back(std::move(frame), input_topics_);
    }
    return wrapped;
  }

  InputTopics input_topics_;
  CloudConcatenator concatenator_;
};
}  // namespace

PYBIND11_MODULE(_concatenate_pointclouds_pybind, m)
{
  m.doc() =
    "Python bindings for the autoware_pointcloud_preprocessor point cloud concatenation core.";

  py::enum_<ConcatenatedFrameStatus>(m, "FrameStatus")
    .value("COMPLETE", ConcatenatedFrameStatus::kComplete)
    .value("TIMEOUT", ConcatenatedFrameStatus::kTimeout)
    .value("LATE", ConcatenatedFrameStatus::kLate);

  py::enum_<MotionCompensationStatus>(m, "MotionCompensationStatus")
    .value("VALID", MotionCompensationStatus::kValid)
    .value("NO_TWIST_AVAILABLE", MotionCompensationStatus::kNoTwistAvailable)
    .value("TWIST_TIME_GAP_TOO_LARGE", MotionCompensationStatus::kTwistTimeGapTooLarge);

  py::class_<FrameDiagnostics>(m, "FrameDiagnostics")
    .def(
      "build", &FrameDiagnostics::build, py::arg("node_name") = std::nullopt,
      py::arg("diagnostic_name") = std::nullopt, py::arg("processing_time_ms") = std::nullopt,
      py::arg("now") = std::nullopt, py::arg("drop_previous_but_late") = false)
    .def(
      py::pickle(
        [](const FrameDiagnostics & self) { return self.state(); },
        [](const FrameDiagnostics::State & state) { return FrameDiagnostics::from_state(state); }));

  py::class_<FrameWrapper>(m, "CoreFrame")
    .def_property_readonly("status", &FrameWrapper::status)
    .def_property_readonly("concatenated_cloud", &FrameWrapper::concatenated_cloud)
    .def_property_readonly("concatenation_info", &FrameWrapper::concatenation_info)
    .def_property_readonly("transformed_clouds", &FrameWrapper::transformed_clouds)
    .def_property_readonly("motion_compensation_status", &FrameWrapper::motion_compensation_status)
    .def_property_readonly("topic_to_original_stamp", &FrameWrapper::topic_to_original_stamp)
    .def_property_readonly(
      "dropped_sources_missing_transform", &FrameWrapper::dropped_sources_missing_transform)
    .def_property_readonly("reference_time", &FrameWrapper::reference_time)
    .def_property_readonly("noise_window", &FrameWrapper::noise_window)
    .def_property_readonly("first_arrival_time", &FrameWrapper::first_arrival_time)
    .def_property_readonly("diagnostics", &FrameWrapper::diagnostics);

  py::class_<ConcatenatorWrapper>(m, "Concatenator")
    .def(
      py::init<
        const std::vector<std::string> &, const std::string &, double, bool, bool, bool,
        const std::string &, const std::optional<std::vector<double>> &,
        const std::optional<std::vector<double>> &, std::size_t, bool, double,
        const std::string &>(),
      py::arg("input_topics"), py::arg("output_frame"), py::arg("timeout_sec"),
      py::arg("is_motion_compensated"), py::arg("publish_synchronized_pointcloud"),
      py::arg("keep_input_frame_in_synchronized_pointcloud"), py::arg("matching_strategy"),
      py::arg("lidar_timestamp_offsets"), py::arg("lidar_timestamp_noise_window"),
      py::arg("max_open_collectors"), py::arg("reject_late_clouds"), py::arg("replay_jump_sec"),
      py::arg("output_point_type"))
    .def("set_transform", &ConcatenatorWrapper::set_transform, py::arg("sensor_to_output_frame"))
    .def("process_twist", &ConcatenatorWrapper::process_twist, py::arg("twist"))
    .def("process_odometry", &ConcatenatorWrapper::process_odometry, py::arg("odometry"))
    .def(
      "process_cloud", &ConcatenatorWrapper::process_cloud, py::arg("topic"), py::arg("cloud"),
      py::arg("arrival_time"))
    .def("close_expired_collectors", &ConcatenatorWrapper::close_expired_collectors, py::arg("now"))
    .def("flush", &ConcatenatorWrapper::flush);
}
