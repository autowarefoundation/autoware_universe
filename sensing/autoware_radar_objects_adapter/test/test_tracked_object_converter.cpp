// Copyright 2026 The Autoware Contributors
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

// Unit tests of TrackedObjectConverter: what one radar object becomes as a tracked object, given
// which fields the radar provides and how ids are made. The arithmetic of the conversion (the
// rotations, the covariances) is covered by the adapter test; here the parameters of the
// converter are what is varied.

#include "radar_objects_adapter.hpp"

#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_sensing_msgs/msg/radar_object.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::radar_objects_adapter::ConversionConfiguration;
using autoware::radar_objects_adapter::GenerateObjectUUID;
using autoware::radar_objects_adapter::TrackedObjectConverter;
using autoware_perception_msgs::msg::Shape;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;
using autoware_sensing_msgs::msg::RadarObject;

constexpr std::size_t cov_yaw_yaw = 35;

// The configuration for a radar that provides no optional field: a parameter in place of each
// of the six, with values that no radar object below carries, and no standard deviations.
ConversionConfiguration nothing_provided()
{
  ConversionConfiguration config;
  config.default_position_z = 0.25;
  config.default_velocity_z = 0.5;
  config.default_acceleration_z = 0.75;
  config.default_size_x = 5.0;
  config.default_size_y = 2.0;
  config.default_size_z = 1.5;
  return config;
}

// The configuration for a radar that provides every optional field: nothing in place of the six,
// and both standard deviations.
ConversionConfiguration all_provided()
{
  ConversionConfiguration config;
  config.orientation_std_provided = true;
  config.orientation_rate_std_provided = true;
  return config;
}

// A way of making ids that the test can recognize: the object id in the first byte, a marker in
// the last.
const GenerateObjectUUID generate_uuid = [](std::uint32_t object_id) {
  std::array<std::uint8_t, 16> uuid{};
  uuid[0] = static_cast<std::uint8_t>(object_id & 0xFF);
  uuid[15] = 0xEE;
  return uuid;
};

TrackedObjectConverter make_converter(const ConversionConfiguration & config)
{
  return TrackedObjectConverter(config, generate_uuid);
}

// A radar object with every field set to a distinct, recognizable value, and a classification
// that the converter is expected to leave alone.
RadarObject make_radar_object()
{
  RadarObject object;
  object.object_id = 0x04030201u;
  object.movement_status = RadarObject::MOVEMENT_STATUS_DYNAMIC;
  object.position.x = 10.0;
  object.position.y = -2.0;
  object.position.z = 0.8;
  object.velocity.x = 3.0;
  object.velocity.y = 0.5;
  object.velocity.z = 0.1;
  object.acceleration.x = 0.2;
  object.acceleration.y = -0.1;
  object.acceleration.z = 0.05;
  object.size.x = 4.5;
  object.size.y = 1.8;
  object.size.z = 1.4;
  object.position_covariance = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  object.velocity_covariance = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
  object.acceleration_covariance = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f};
  object.orientation = 0.0f;
  object.orientation_std = 0.1f;
  object.orientation_rate = 0.05f;
  object.orientation_rate_std = 0.02f;
  object.existence_probability = 0.9f;
  object.classifications.resize(1);
  object.classifications[0].label = 1;
  object.classifications[0].probability = 0.8f;
  return object;
}
}  // namespace

// Each optional field with a parameter is set to the value the configuration holds in its place,
// and copied from the object when the configuration holds none; the fields act one by one.
TEST(TrackedObjectConverter, OptionalFields_ParameterInPlaceUnlessProvided)
{
  const RadarObject radar = make_radar_object();
  const ConversionConfiguration config = nothing_provided();

  struct Case
  {
    std::string field;
    std::function<void(ConversionConfiguration &)> provide;
    std::function<double(const TrackedObject &)> read;
    double from_object;
    double from_params;
  };
  const std::vector<Case> cases = {
    {"position_z", [](ConversionConfiguration & c) { c.default_position_z.reset(); },
     [](const TrackedObject & t) { return t.kinematics.pose_with_covariance.pose.position.z; },
     radar.position.z, *config.default_position_z},
    {"velocity_z", [](ConversionConfiguration & c) { c.default_velocity_z.reset(); },
     [](const TrackedObject & t) { return t.kinematics.twist_with_covariance.twist.linear.z; },
     radar.velocity.z, *config.default_velocity_z},
    {"acceleration_z", [](ConversionConfiguration & c) { c.default_acceleration_z.reset(); },
     [](const TrackedObject & t) {
       return t.kinematics.acceleration_with_covariance.accel.linear.z;
     },
     radar.acceleration.z, *config.default_acceleration_z},
    {"size_x", [](ConversionConfiguration & c) { c.default_size_x.reset(); },
     [](const TrackedObject & t) { return t.shape.dimensions.x; }, radar.size.x,
     *config.default_size_x},
    {"size_y", [](ConversionConfiguration & c) { c.default_size_y.reset(); },
     [](const TrackedObject & t) { return t.shape.dimensions.y; }, radar.size.y,
     *config.default_size_y},
    {"size_z", [](ConversionConfiguration & c) { c.default_size_z.reset(); },
     [](const TrackedObject & t) { return t.shape.dimensions.z; }, radar.size.z,
     *config.default_size_z},
  };

  for (const auto & c : cases) {
    SCOPED_TRACE(c.field);
    ConversionConfiguration only_this = nothing_provided();
    c.provide(only_this);
    EXPECT_DOUBLE_EQ(c.read(make_converter(only_this)(radar)), c.from_object);
    EXPECT_DOUBLE_EQ(c.read(make_converter(nothing_provided())(radar)), c.from_params);
  }
}

// The yaw variance of the pose is the square of orientation_std when that field is provided and
// stays zero otherwise; the same for the yaw rate variance of the twist.
TEST(TrackedObjectConverter, YawVariances_SquaredWhenProvidedZeroOtherwise)
{
  const RadarObject radar = make_radar_object();
  const double yaw_std = radar.orientation_std;
  const double yaw_rate_std = radar.orientation_rate_std;

  const TrackedObject with = make_converter(all_provided())(radar);
  EXPECT_NEAR(
    with.kinematics.pose_with_covariance.covariance[cov_yaw_yaw], yaw_std * yaw_std, 1e-6);
  EXPECT_NEAR(
    with.kinematics.twist_with_covariance.covariance[cov_yaw_yaw], yaw_rate_std * yaw_rate_std,
    1e-6);

  const TrackedObject without = make_converter(nothing_provided())(radar);
  EXPECT_DOUBLE_EQ(without.kinematics.pose_with_covariance.covariance[cov_yaw_yaw], 0.0);
  EXPECT_DOUBLE_EQ(without.kinematics.twist_with_covariance.covariance[cov_yaw_yaw], 0.0);
}

// The id comes from the function the converter was given, called with the radar's object id.
TEST(TrackedObjectConverter, Id_FromGivenFunction)
{
  RadarObject radar = make_radar_object();
  radar.object_id = 0x8899AABBu;

  const TrackedObject track = make_converter(all_provided())(radar);

  EXPECT_EQ(track.object_id.uuid, generate_uuid(0x8899AABBu));
  EXPECT_EQ(track.object_id.uuid[0], 0xBB);
  EXPECT_EQ(track.object_id.uuid[15], 0xEE);
}

// The converter fills in what the radar reports and sets the kinematics flags, but leaves the
// classification empty: that is the remapper's.
TEST(TrackedObjectConverter, Output_ReportedFieldsSetClassificationLeftEmpty)
{
  const RadarObject radar = make_radar_object();

  const TrackedObject track = make_converter(all_provided())(radar);

  EXPECT_FLOAT_EQ(track.existence_probability, radar.existence_probability);
  EXPECT_EQ(track.shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(track.kinematics.pose_with_covariance.pose.position.x, radar.position.x);
  EXPECT_DOUBLE_EQ(track.kinematics.pose_with_covariance.pose.position.y, radar.position.y);
  EXPECT_DOUBLE_EQ(
    track.kinematics.twist_with_covariance.twist.angular.z,
    static_cast<double>(radar.orientation_rate));
  EXPECT_EQ(track.kinematics.orientation_availability, TrackedObjectKinematics::AVAILABLE);
  EXPECT_FALSE(track.kinematics.is_stationary);
  EXPECT_TRUE(track.classification.empty());
}

// Only MOVEMENT_STATUS_DYNAMIC counts as moving.
TEST(TrackedObjectConverter, Stationary_OnlyDynamicIsMoving)
{
  const TrackedObjectConverter convert = make_converter(all_provided());
  const std::vector<std::pair<std::uint8_t, bool>> cases = {
    {RadarObject::MOVEMENT_STATUS_DYNAMIC, false},
    {RadarObject::MOVEMENT_STATUS_STATIC, true},
    {RadarObject::MOVEMENT_STATUS_INVALID, true},
    {RadarObject::MOVEMENT_STATUS_UNKNOWN, true},
  };
  for (const auto & [status, stationary] : cases) {
    SCOPED_TRACE(static_cast<int>(status));
    RadarObject radar = make_radar_object();
    radar.movement_status = status;

    EXPECT_EQ(convert(radar).kinematics.is_stationary, stationary);
  }
}
