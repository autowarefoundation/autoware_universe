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

#include "../src/crosswalk_traffic_light_estimator.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <vector>

using autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimator;
using autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorConfig;
using autoware::crosswalk_traffic_light_estimator::FlashingDetectionConfig;
using autoware::crosswalk_traffic_light_estimator::TrafficSignal;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalArray;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalElement;

namespace
{

constexpr lanelet::Id VEHICLE_TL_REG_ELEM_ID = 100;
constexpr lanelet::Id CROSSWALK_TL_REG_ELEM_ID = 200;
constexpr lanelet::Id VEHICLE_TL_RIGHT_ID = 300;

CrosswalkTrafficLightEstimatorConfig make_default_config()
{
  CrosswalkTrafficLightEstimatorConfig config;
  config.use_last_detect_color = true;
  config.use_pedestrian_signal_detect = false;
  config.last_detect_color_hold_time = 2.0;
  FlashingDetectionConfig flashing_config;
  flashing_config.last_colors_hold_time = 1.0;
  config.flashing_detection = flashing_config;
  return config;
}

rclcpp::Time make_time(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

TrafficSignal make_signal(lanelet::Id tl_id, uint8_t color, float confidence = 1.0)
{
  TrafficSignal signal;
  signal.traffic_light_group_id = tl_id;
  TrafficSignalElement element;
  element.color = color;
  element.shape = TrafficSignalElement::CIRCLE;
  element.status = TrafficSignalElement::SOLID_ON;
  element.confidence = confidence;
  signal.elements.push_back(element);
  return signal;
}

TrafficSignalArray make_signal_array(const std::vector<TrafficSignal> & signals)
{
  TrafficSignalArray array;
  array.traffic_light_groups = signals;
  return array;
}

/// @brief Create a lanelet map with a straight vehicle lanelet (TL=100),
/// a right-turn vehicle lanelet (TL=300), and a crosswalk (TL=200) that overlaps both.
///
///           y
///           ^
///       6   |    +------------+
///           |    |  Crosswalk |
///       4   +----+--+---------+--+---------------------+
///           |    |  | overlap |  |  Straight (TL=100)  |
///       0   +----+--+---------+--+---------------------+
///           |    |  | overlap |  |  Right-turn (TL=300)|
///      -4   +----+--+---------+--+---------------------+---> x
///           0    |  8        12  |                     20
///      -6        +------------+
///                  TL ID=200
///                  Crosswalk (ID=2000)
lanelet::LaneletMapPtr create_test_map(const lanelet::AttributeMap & vehicle_tl_extra_attrs = {})
{
  // Straight vehicle lanelet bounds (y=0..4)
  lanelet::Point3d vehicle_right_start(1, 0.0, 0.0, 0.0);
  lanelet::Point3d vehicle_right_end(2, 20.0, 0.0, 0.0);
  lanelet::Point3d vehicle_left_start(3, 0.0, 4.0, 0.0);
  lanelet::Point3d vehicle_left_end(4, 20.0, 4.0, 0.0);

  lanelet::LineString3d vehicle_right(10, {vehicle_right_start, vehicle_right_end});
  lanelet::LineString3d vehicle_left(11, {vehicle_left_start, vehicle_left_end});

  // Right-turn vehicle lanelet bounds (y=-4..0)
  lanelet::LineString3d rt_right(
    610, {lanelet::Point3d(601, 0.0, -4.0, 0.0), lanelet::Point3d(602, 20.0, -4.0, 0.0)});
  lanelet::LineString3d rt_left(
    611, {lanelet::Point3d(603, 0.0, 0.0, 0.0), lanelet::Point3d(604, 20.0, 0.0, 0.0)});

  // Crosswalk lanelet bounds (overlaps both vehicle lanelets)
  lanelet::Point3d crosswalk_left_start(5, 8.0, -6.0, 0.0);
  lanelet::Point3d crosswalk_left_end(6, 8.0, 6.0, 0.0);
  lanelet::Point3d crosswalk_right_start(7, 12.0, -6.0, 0.0);
  lanelet::Point3d crosswalk_right_end(8, 12.0, 6.0, 0.0);

  lanelet::LineString3d crosswalk_left(12, {crosswalk_left_start, crosswalk_left_end});
  lanelet::LineString3d crosswalk_right(13, {crosswalk_right_start, crosswalk_right_end});

  // Traffic light physical locations
  lanelet::LineString3d vehicle_tl_linestring(15, {lanelet::Point3d(14, 10.0, 5.0, 3.0)});
  lanelet::LineString3d rt_tl_linestring(612, {lanelet::Point3d(605, 10.0, -4.0, 3.0)});
  lanelet::LineString3d crosswalk_tl_linestring(17, {lanelet::Point3d(16, 13.0, 2.0, 3.0)});

  // Create traffic light regulatory elements
  auto vehicle_traffic_light = lanelet::TrafficLight::make(
    VEHICLE_TL_REG_ELEM_ID, vehicle_tl_extra_attrs, {vehicle_tl_linestring});
  auto rt_traffic_light =
    lanelet::TrafficLight::make(VEHICLE_TL_RIGHT_ID, lanelet::AttributeMap{}, {rt_tl_linestring});
  auto crosswalk_traffic_light = lanelet::TrafficLight::make(
    CROSSWALK_TL_REG_ELEM_ID, lanelet::AttributeMap{}, {crosswalk_tl_linestring});

  // Create straight vehicle lanelet
  lanelet::Lanelet vehicle_lanelet(1000, vehicle_left, vehicle_right);
  vehicle_lanelet.attributes()["type"] = "lanelet";
  vehicle_lanelet.attributes()["subtype"] = "road";
  vehicle_lanelet.attributes()["turn_direction"] = "straight";
  vehicle_lanelet.addRegulatoryElement(vehicle_traffic_light);

  // Create right-turn vehicle lanelet
  lanelet::Lanelet rt_lanelet(3000, rt_left, rt_right);
  rt_lanelet.attributes()["type"] = "lanelet";
  rt_lanelet.attributes()["subtype"] = "road";
  rt_lanelet.attributes()["turn_direction"] = "right";
  rt_lanelet.addRegulatoryElement(rt_traffic_light);

  // Create crosswalk lanelet
  lanelet::Lanelet crosswalk_lanelet(2000, crosswalk_left, crosswalk_right);
  crosswalk_lanelet.attributes()["type"] = "lanelet";
  crosswalk_lanelet.attributes()["subtype"] = "crosswalk";
  crosswalk_lanelet.addRegulatoryElement(crosswalk_traffic_light);

  // Build map
  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(vehicle_lanelet);
  map->add(rt_lanelet);
  map->add(crosswalk_lanelet);

  return map;
}

const TrafficSignal * find_signal(const TrafficSignalArray & array, lanelet::Id id)
{
  for (const auto & signal : array.traffic_light_groups) {
    if (signal.traffic_light_group_id == id) {
      return &signal;
    }
  }
  return nullptr;
}

void assert_crosswalk_color(const TrafficSignalArray & result, uint8_t expected_color)
{
  const auto * crosswalk_signal = find_signal(result, CROSSWALK_TL_REG_ELEM_ID);
  ASSERT_NE(crosswalk_signal, nullptr);
  ASSERT_FALSE(crosswalk_signal->elements.empty());
  EXPECT_EQ(crosswalk_signal->elements.front().color, expected_color);
}

CrosswalkTrafficLightEstimator make_estimator_with_map()
{
  CrosswalkTrafficLightEstimator estimator(make_default_config());
  estimator.update_map(create_test_map());
  return estimator;
}

lanelet::LaneletMapPtr create_test_map_with_vehicle_tl_rule(
  const std::string & rule_attribute_key, const std::string & crosswalk_tl_ids_value)
{
  lanelet::AttributeMap vehicle_tl_attrs;
  vehicle_tl_attrs[rule_attribute_key] = crosswalk_tl_ids_value;
  return create_test_map(vehicle_tl_attrs);
}

CrosswalkTrafficLightEstimator make_estimator_with_rule(const std::string & rule_attribute_key)
{
  CrosswalkTrafficLightEstimator estimator(make_default_config());
  estimator.update_map(create_test_map_with_vehicle_tl_rule(
    rule_attribute_key, std::to_string(CROSSWALK_TL_REG_ELEM_ID)));
  return estimator;
}

}  // namespace

TEST(CrosswalkTrafficLightEstimatorTest, ConstructorDoesNotThrow)
{
  // Arrange
  auto config = make_default_config();

  // Act & Assert
  EXPECT_NO_THROW({ CrosswalkTrafficLightEstimator estimator(config); });
}

TEST(CrosswalkTrafficLightEstimatorTest, IsMapLoadedInitiallyFalse)
{
  // Arrange
  CrosswalkTrafficLightEstimator estimator(make_default_config());

  // Act
  const bool loaded = estimator.is_map_loaded();

  // Assert
  EXPECT_FALSE(loaded);
}

TEST(CrosswalkTrafficLightEstimatorTest, IsMapLoadedAfterUpdateMap)
{
  // Arrange
  CrosswalkTrafficLightEstimator estimator(make_default_config());
  auto map = create_test_map();

  // Act
  estimator.update_map(map);

  // Assert
  EXPECT_TRUE(estimator.is_map_loaded());
}

TEST(CrosswalkTrafficLightEstimatorTest, FindUnregistered_AllRegistered_ReturnsEmpty)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray msg =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::GREEN)});

  // Act
  const auto unregistered = estimator.find_unregistered_traffic_light_group_ids(msg);

  // Assert
  EXPECT_TRUE(unregistered.empty());
}

TEST(CrosswalkTrafficLightEstimatorTest, FindUnregistered_WithUnknownId_ReturnsIt)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignal unknown_signal;
  unknown_signal.traffic_light_group_id = 99999;
  TrafficSignalArray msg = make_signal_array({unknown_signal});

  // Act
  const auto unregistered = estimator.find_unregistered_traffic_light_group_ids(msg);

  // Assert
  ASSERT_EQ(unregistered.size(), 1u);
  EXPECT_EQ(unregistered[0], 99999);
}

TEST(CrosswalkTrafficLightEstimatorTest, Estimate_FirstCallWithGreenVehicle_CrosswalkRed)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray green_msg =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::GREEN)});

  // Act: GREEN vehicle signal is sufficient to estimate crosswalk as RED, even on first call
  const auto result = estimator.estimate(green_msg, make_time(0.0));

  // Assert
  assert_crosswalk_color(result, TrafficSignalElement::RED);
}

TEST(CrosswalkTrafficLightEstimatorTest, Estimate_UnknownVehicleNoHistory_CrosswalkUnknown)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray unknown_msg =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::UNKNOWN)});

  // Act: UNKNOWN vehicle signal with no prior history → crosswalk cannot be estimated
  const auto result = estimator.estimate(unknown_msg, make_time(0.0));

  // Assert
  assert_crosswalk_color(result, TrafficSignalElement::UNKNOWN);
}

TEST(CrosswalkTrafficLightEstimatorTest, Estimate_StraightGreenVehicle_CrosswalkRed)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray green_msg =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::GREEN)});

  // Act
  const auto result = estimator.estimate(green_msg, make_time(0.0));

  // Assert: straight green vehicle signal → crosswalk should be RED
  assert_crosswalk_color(result, TrafficSignalElement::RED);
}

TEST(CrosswalkTrafficLightEstimatorTest, Estimate_RedVehicle_CrosswalkUnknown)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray red_msg =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::RED)});

  // Act
  const auto result = estimator.estimate(red_msg, make_time(0.0));

  // Assert: vehicle is RED → crosswalk signal is UNKNOWN (cannot determine)
  assert_crosswalk_color(result, TrafficSignalElement::UNKNOWN);
}

TEST(CrosswalkTrafficLightEstimatorTest, Estimate_EmptyInput_ReturnsEmpty)
{
  // Arrange
  auto estimator = make_estimator_with_map();
  TrafficSignalArray empty_msg;

  // Act
  const auto result = estimator.estimate(empty_msg, make_time(0.0));

  // Assert
  EXPECT_TRUE(result.traffic_light_groups.empty());
}

TEST(
  CrosswalkTrafficLightEstimatorTest,
  ParseSignalEstimationRules_ValidGreenToGreen_OverridesEstimation)
{
  // Arrange: rule "signal_color_relation:green:green" → crosswalk TL ID 200
  // Normal estimation: GREEN vehicle + straight lane → crosswalk RED
  // With override:     green matches, crosswalk becomes GREEN
  auto estimator = make_estimator_with_rule("signal_color_relation:green:green");
  TrafficSignalArray input =
    make_signal_array({make_signal(VEHICLE_TL_REG_ELEM_ID, TrafficSignalElement::GREEN)});

  // Act
  const auto result = estimator.estimate(input, make_time(0.0));

  // Assert: override (GREEN) wins over normal estimation (RED)
  assert_crosswalk_color(result, TrafficSignalElement::GREEN);
}

/// @brief Right-turn arrow scenario: only the right-turn TL (ID=300) reports GREEN,
/// while the straight TL (ID=100) has no detection at all.
/// The crosswalk must be UNKNOWN because the straight lane's signal state is unknown.
TEST(
  CrosswalkTrafficLightEstimatorTest, Estimate_RightArrowGreen_StraightUndetected_CrosswalkUnknown)
{
  // Arrange: existing map (straight lane TL=100 + crosswalk) with an added right-turn lane TL=300
  CrosswalkTrafficLightEstimator estimator(make_default_config());
  estimator.update_map(create_test_map());

  // Only the right-turn TL reports GREEN; straight TL (100) is absent from the message
  TrafficSignalArray input =
    make_signal_array({make_signal(VEHICLE_TL_RIGHT_ID, TrafficSignalElement::GREEN)});

  // Act
  const auto result = estimator.estimate(input, make_time(0.0));

  // Assert: crosswalk must be UNKNOWN, not RED
  assert_crosswalk_color(result, TrafficSignalElement::UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
