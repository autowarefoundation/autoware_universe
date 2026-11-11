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

// Python bindings for the distortion corrector core. Messages cross the boundary as CDR bytes
// (rclpy.serialization on the Python side), so the binding needs no message conversion code, and
// all correction logic stays in the core.

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

namespace py = pybind11;

namespace
{
using autoware::pointcloud_preprocessor::AngleConversion;
using autoware::pointcloud_preprocessor::DistortionCorrector2D;
using autoware::pointcloud_preprocessor::DistortionCorrector3D;
using autoware::pointcloud_preprocessor::DistortionCorrectorBase;
using autoware::pointcloud_preprocessor::PointcloudValidity;
using autoware::pointcloud_preprocessor::UndistortionResult;

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

// Owns the corrector and the cached angle conversion, as the node does, and converts serialized
// messages to and from the core's C++ types.
class DistortionCorrectorWrapper
{
public:
  explicit DistortionCorrectorWrapper(bool use_3d_distortion_correction)
  {
    if (use_3d_distortion_correction) {
      corrector_ = std::make_unique<DistortionCorrector3D>();
    } else {
      corrector_ = std::make_unique<DistortionCorrector2D>();
    }
  }

  void insert_twist(const py::bytes & twist)
  {
    corrector_->insert_twist(
      std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(
        deserialize<geometry_msgs::msg::TwistWithCovarianceStamped>(twist)));
  }

  void set_imu_transform(const py::bytes & imu_to_base_link)
  {
    corrector_->set_imu_transform(
      deserialize<geometry_msgs::msg::TransformStamped>(imu_to_base_link));
  }

  void insert_imu(const py::bytes & imu)
  {
    corrector_->insert_imu(
      std::make_shared<sensor_msgs::msg::Imu>(deserialize<sensor_msgs::msg::Imu>(imu)));
  }

  std::pair<py::bytes, UndistortionResult> undistort_pointcloud(
    const py::bytes & pointcloud, const py::bytes & lidar_to_base_link, bool use_imu,
    bool update_azimuth_and_distance)
  {
    auto cloud = deserialize<sensor_msgs::msg::PointCloud2>(pointcloud);

    if (update_azimuth_and_distance && !angle_conversion_opt_.has_value()) {
      if (const auto angle_conversion = corrector_->try_compute_angle_conversion(cloud)) {
        angle_conversion_opt_ = *angle_conversion;
      }
    }

    const auto result = corrector_->undistort_pointcloud(
      use_imu, update_azimuth_and_distance ? angle_conversion_opt_ : std::nullopt,
      deserialize<geometry_msgs::msg::TransformStamped>(lidar_to_base_link), cloud);
    return {serialize(cloud), result};
  }

  [[nodiscard]] int get_timestamp_mismatch_count() const
  {
    return corrector_->get_timestamp_mismatch_count();
  }

  [[nodiscard]] double get_timestamp_mismatch_fraction() const
  {
    return corrector_->get_timestamp_mismatch_fraction();
  }

private:
  std::unique_ptr<DistortionCorrectorBase> corrector_;
  std::optional<AngleConversion> angle_conversion_opt_;
};
}  // namespace

PYBIND11_MODULE(_distortion_corrector_pybind, m)
{
  m.doc() = "Python bindings for the autoware_pointcloud_preprocessor distortion corrector core.";

  py::enum_<PointcloudValidity>(m, "PointcloudValidity")
    .value("VALID", PointcloudValidity::kValid)
    .value("EMPTY", PointcloudValidity::kEmpty)
    .value("MISSING_TIME_STAMP_FIELD", PointcloudValidity::kMissingTimeStampField)
    .value("INCOMPATIBLE_LAYOUT", PointcloudValidity::kIncompatibleLayout);

  py::class_<UndistortionResult>(m, "UndistortionResult")
    .def_readonly("validity", &UndistortionResult::validity)
    .def_readonly("twist_queue_empty", &UndistortionResult::twist_queue_empty)
    .def_readonly("twist_timestamp_too_late", &UndistortionResult::twist_timestamp_too_late)
    .def_readonly("imu_timestamp_too_late", &UndistortionResult::imu_timestamp_too_late);

  py::class_<DistortionCorrectorWrapper>(m, "DistortionCorrector")
    .def(py::init<bool>(), py::arg("use_3d_distortion_correction") = false)
    .def("insert_twist", &DistortionCorrectorWrapper::insert_twist, py::arg("twist"))
    .def(
      "set_imu_transform", &DistortionCorrectorWrapper::set_imu_transform,
      py::arg("imu_to_base_link"))
    .def("insert_imu", &DistortionCorrectorWrapper::insert_imu, py::arg("imu"))
    .def(
      "undistort_pointcloud", &DistortionCorrectorWrapper::undistort_pointcloud,
      py::arg("pointcloud"), py::arg("lidar_to_base_link"), py::arg("use_imu") = true,
      py::arg("update_azimuth_and_distance") = false)
    .def_property_readonly(
      "timestamp_mismatch_count", &DistortionCorrectorWrapper::get_timestamp_mismatch_count)
    .def_property_readonly(
      "timestamp_mismatch_fraction", &DistortionCorrectorWrapper::get_timestamp_mismatch_fraction);
}
