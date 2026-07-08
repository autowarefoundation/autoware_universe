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

#include "autoware/ptv3/experimental/point_type.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ptv3::experimental
{
namespace
{
sensor_msgs::msg::PointField make_field(
  const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

std::vector<sensor_msgs::msg::PointField> make_point_xyzcpe_fields()
{
  using PointType = PointXYZCPE;

  return {
    make_field("x", offsetof(PointType, x), sensor_msgs::msg::PointField::FLOAT32),
    make_field("y", offsetof(PointType, y), sensor_msgs::msg::PointField::FLOAT32),
    make_field("z", offsetof(PointType, z), sensor_msgs::msg::PointField::FLOAT32),
    make_field("class_id", offsetof(PointType, class_id), sensor_msgs::msg::PointField::UINT8),
    make_field(
      "probability", offsetof(PointType, probability), sensor_msgs::msg::PointField::FLOAT32),
    make_field("entropy", offsetof(PointType, entropy), sensor_msgs::msg::PointField::FLOAT32),
  };
}
}  // namespace

class PointTypeTest : public ::testing::Test
{
};

TEST_F(PointTypeTest, PointXYZCPELayout)
{
  EXPECT_EQ(offsetof(PointXYZCPE, x), 0U);
  EXPECT_EQ(offsetof(PointXYZCPE, y), 4U);
  EXPECT_EQ(offsetof(PointXYZCPE, z), 8U);
  EXPECT_EQ(offsetof(PointXYZCPE, class_id), 12U);
  EXPECT_EQ(offsetof(PointXYZCPE, probability), 16U);
  EXPECT_EQ(offsetof(PointXYZCPE, entropy), 20U);
  EXPECT_EQ(sizeof(PointXYZCPE), 24U);
}

TEST_F(PointTypeTest, PointXYZCPEIndexOrderMatchesFieldOrder)
{
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::X), 0U);
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::Y), 1U);
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::Z), 2U);
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::CLASS_ID), 3U);
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::PROBABILITY), 4U);
  EXPECT_EQ(static_cast<std::size_t>(PointXYZCPEIndex::ENTROPY), 5U);
}

TEST_F(PointTypeTest, PointXYZCPEEqualityUsesAllFields)
{
  const PointXYZCPE base{1.0F, 2.0F, 3.0F, 4U, 0.5F, 0.25F};

  EXPECT_EQ(base, base);
  EXPECT_FALSE(base == (PointXYZCPE{1.1F, 2.0F, 3.0F, 4U, 0.5F, 0.25F}));
  EXPECT_FALSE(base == (PointXYZCPE{1.0F, 2.1F, 3.0F, 4U, 0.5F, 0.25F}));
  EXPECT_FALSE(base == (PointXYZCPE{1.0F, 2.0F, 3.1F, 4U, 0.5F, 0.25F}));
  EXPECT_FALSE(base == (PointXYZCPE{1.0F, 2.0F, 3.0F, 5U, 0.5F, 0.25F}));
  EXPECT_FALSE(base == (PointXYZCPE{1.0F, 2.0F, 3.0F, 4U, 0.6F, 0.25F}));
  EXPECT_FALSE(base == (PointXYZCPE{1.0F, 2.0F, 3.0F, 4U, 0.5F, 0.35F}));
}

TEST_F(PointTypeTest, CompatibleFieldsAreAccepted)
{
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzcpe(make_point_xyzcpe_fields()));

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = make_point_xyzcpe_fields();
  EXPECT_TRUE(is_data_layout_compatible_with_point_xyzcpe(cloud));
}

TEST_F(PointTypeTest, WrongFieldElementCountIsRejected)
{
  auto fields = make_point_xyzcpe_fields();
  fields.pop_back();

  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzcpe(fields));
}

TEST_F(PointTypeTest, WrongFieldNameIsRejected)
{
  auto fields = make_point_xyzcpe_fields();
  fields.at(static_cast<std::size_t>(PointXYZCPEIndex::CLASS_ID)).name = "label";

  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzcpe(fields));
}

TEST_F(PointTypeTest, WrongFieldOffsetIsRejected)
{
  auto fields = make_point_xyzcpe_fields();
  fields.at(static_cast<std::size_t>(PointXYZCPEIndex::PROBABILITY)).offset = 13U;

  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzcpe(fields));
}

TEST_F(PointTypeTest, WrongFieldDatatypeIsRejected)
{
  auto fields = make_point_xyzcpe_fields();
  fields.at(static_cast<std::size_t>(PointXYZCPEIndex::CLASS_ID)).datatype =
    sensor_msgs::msg::PointField::FLOAT32;

  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzcpe(fields));
}

TEST_F(PointTypeTest, WrongFieldCountIsRejected)
{
  auto fields = make_point_xyzcpe_fields();
  fields.at(static_cast<std::size_t>(PointXYZCPEIndex::ENTROPY)).count = 2;

  EXPECT_FALSE(is_data_layout_compatible_with_point_xyzcpe(fields));
}
}  // namespace autoware::ptv3::experimental
