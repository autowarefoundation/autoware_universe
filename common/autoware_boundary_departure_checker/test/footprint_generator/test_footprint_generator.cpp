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

#include "autoware/boundary_departure_checker/footprint_generator/footprint_manager.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/localization_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/longitudinal_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/normal_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/steering_footprint.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "test_plot_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{
void plot_steering_footprints(
  [[maybe_unused]] const TrajectoryPoints & pred_traj,
  [[maybe_unused]] const std::vector<autoware_utils_geometry::LinearRing2d> & footprints,
  [[maybe_unused]] const std::string & title, [[maybe_unused]] const std::string & sub_dir)
{
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot the original predicted path
    std::vector<double> px;
    std::vector<double> py;
    for (const auto & pt : pred_traj) {
      px.push_back(pt.pose.position.x);
      py.push_back(pt.pose.position.y);
    }
    plt.plot(
      Args(px, py), Kwargs(
                      "color"_a = "gray", "linestyle"_a = "--", "marker"_a = "o",
                      "label"_a = "Original Planned Path"));

    // Plot the generated footprints
    for (size_t i = 0; i < footprints.size(); ++i) {
      std::vector<double> fx;
      std::vector<double> fy;
      for (const auto & p_fp : footprints[i]) {
        fx.push_back(p_fp.x());
        fy.push_back(p_fp.y());
      }
      // Only attach the label to the first footprint to avoid legend clutter
      if (i == 0) {
        plt.plot(
          Args(fx, fy),
          Kwargs("color"_a = "blue", "alpha"_a = 0.5, "label"_a = "Generated Footprints"));
      } else {
        plt.plot(Args(fx, fy), Kwargs("color"_a = "blue", "alpha"_a = 0.5));
      }
    }

    plt.title(Args(title));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.axis(Args("equal"));
    plt.legend();

    save_figure(plt, sub_dir);
  });
}

class FootprintGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 1. Create Mock Vehicle Info
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 0.128, 0.128, 2.5, 0.70);

    // 2. Setup Trajectory
    TrajectoryPoint p1;
    p1.pose.position.x = 0.0;
    p1.pose.position.y = 0.0;
    p1.pose.orientation.w = 1.0;
    p1.longitudinal_velocity_mps = 10.0;
    p1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    pred_traj_.push_back(p1);

    TrajectoryPoint p2 = p1;
    p2.pose.position.x = 1.0;
    p2.time_from_start = rclcpp::Duration::from_seconds(0.1);
    pred_traj_.push_back(p2);

    // 3. Setup Param and Configs
    param_.footprint_extra_margin = 0.0;

    // Normal
    param_.abnormality_configs[FootprintType::NORMAL] = NormalConfig{};

    // Localization
    LocalizationConfig loc_config;
    loc_config.footprint_envelop.lon_m = 0.1;
    loc_config.footprint_envelop.lat_m = 0.2;
    param_.abnormality_configs[FootprintType::LOCALIZATION] = loc_config;

    // Longitudinal
    LongitudinalConfig lon_config;
    lon_config.lon_tracking.scale = 0.5;
    lon_config.lon_tracking.extra_margin_m = 1.0;
    param_.abnormality_configs[FootprintType::LONGITUDINAL] = lon_config;

    // Steering
    SteeringConfig steer_config;
    steer_config.delay_s = 0.0;
    steer_config.offset_rps = 0.1;
    steer_config.factor = 1.0;
    steer_config.steering_rate_velocities_mps = {0.0, 10.0};
    steer_config.steering_rate_limits_rps = {0.5, 0.5};
    param_.abnormality_configs[FootprintType::STEERING_STUCK] = steer_config;

    // 4. Setup base uncertainty margin
    pose_with_cov_.pose.orientation.w = 1.0;
    for (auto & c : pose_with_cov_.covariance) {
      c = 0.0;
    }
    uncertainty_margin_ = {0.0, 0.0};
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  TrajectoryPoints pred_traj_;
  Param param_;
  geometry_msgs::msg::PoseWithCovariance pose_with_cov_;
  FootprintMargin uncertainty_margin_;
};

TEST_F(FootprintGeneratorTest, TestNormalFootprintGenerator)
{
  NormalFootprintGenerator generator;
  EXPECT_EQ(generator.get_type(), FootprintType::NORMAL);

  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  ASSERT_EQ(footprints.size(), pred_traj_.size());
  // Normal footprint should strictly follow base vehicle specs since extra margin is 0.0
  // Left Front is at X = Front_Overhang + Wheelbase, Y = Half_Tread + Left_Overhang
  const double expected_x = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
}

TEST_F(FootprintGeneratorTest, TestLocalizationFootprintGenerator)
{
  LocalizationFootprintGenerator generator;
  EXPECT_EQ(generator.get_type(), FootprintType::LOCALIZATION);

  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  ASSERT_EQ(footprints.size(), pred_traj_.size());

  // Should include the loc_config margin (lon = 0.1, lat = 0.2)
  const double expected_x = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m + 0.1;
  const double expected_y =
    (vehicle_info_.wheel_tread_m / 2.0) + vehicle_info_.left_overhang_m + 0.2;

  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].y(), expected_y);
}

TEST_F(FootprintGeneratorTest, TestLongitudinalFootprintGenerator)
{
  LongitudinalFootprintGenerator generator;
  EXPECT_EQ(generator.get_type(), FootprintType::LONGITUDINAL);

  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  ASSERT_EQ(footprints.size(), pred_traj_.size());

  // Should include dynamic velocity margin: v(10.0) * scale(0.5) + extra(1.0) = 6.0
  constexpr double dynamic_margin = (10.0 * 0.5) + 1.0;
  const double expected_x =
    vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m + dynamic_margin;

  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGenerator)
{
  SteeringFootprintGenerator generator(FootprintType::STEERING_STUCK);
  EXPECT_EQ(generator.get_type(), FootprintType::STEERING_STUCK);

  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  // Must return the same size of footprints as the input trajectory length
  ASSERT_EQ(footprints.size(), pred_traj_.size());
}

TEST_F(FootprintGeneratorTest, TestFootprintManagerGenerateAll)
{
  std::vector<FootprintType> types = {
    FootprintType::NORMAL, FootprintType::LOCALIZATION, FootprintType::LONGITUDINAL,
    FootprintType::STEERING_STUCK};

  FootprintManager manager(types);

  // Generate all footprints concurrently
  const auto all_footprints =
    manager.generate_all(pred_traj_, vehicle_info_, pose_with_cov_, param_);

  // Check if map contains all requested types
  EXPECT_TRUE(all_footprints.find(FootprintType::NORMAL) != all_footprints.end());
  EXPECT_TRUE(all_footprints.find(FootprintType::LOCALIZATION) != all_footprints.end());
  EXPECT_TRUE(all_footprints.find(FootprintType::LONGITUDINAL) != all_footprints.end());
  EXPECT_TRUE(all_footprints.find(FootprintType::STEERING_STUCK) != all_footprints.end());

  // NORMAL is strictly the first in priority sequence
  const auto & type_order = manager.get_footprint_type_order();
  ASSERT_GE(type_order.size(), 1);
  EXPECT_EQ(type_order.front(), FootprintType::NORMAL);

  // Validate the sizes
  EXPECT_EQ(all_footprints.at(FootprintType::NORMAL).size(), pred_traj_.size());
  EXPECT_EQ(all_footprints.at(FootprintType::LOCALIZATION).size(), pred_traj_.size());
}

TEST_F(FootprintGeneratorTest, TestFootprintManagerDuplicateNormalIgnored)
{
  // If NORMAL is passed manually, it should not be registered twice.
  std::vector<FootprintType> types = {FootprintType::NORMAL, FootprintType::NORMAL};
  FootprintManager manager(types);

  const auto & type_order = manager.get_footprint_type_order();

  // Because the constructor forces NORMAL first and ignores subsequent NORMALs
  ASSERT_EQ(type_order.size(), 1);
  EXPECT_EQ(type_order.front(), FootprintType::NORMAL);
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorEmptyTrajectory)
{
  TrajectoryPoints empty_traj;
  SteeringFootprintGenerator generator(FootprintType::STEERING_STUCK);

  const auto footprints =
    generator.generate(empty_traj, vehicle_info_, param_, uncertainty_margin_);

  // Hit the "return {};" branch when pred_traj is empty
  EXPECT_TRUE(footprints.empty());
}

TEST_F(FootprintGeneratorTest, TestMissingConfig)
{
  // Clear the configs so param.get_abnormality_config returns nullopt

  std::vector<FootprintType> types = {
    FootprintType::NORMAL, FootprintType::LOCALIZATION, FootprintType::LONGITUDINAL,
    FootprintType::STEERING_STUCK};
  auto param = param_;
  param.abnormality_configs = {};

  FootprintManager manager(types);
  const auto all_footprints =
    manager.generate_all(pred_traj_, vehicle_info_, pose_with_cov_, param);

  EXPECT_TRUE(all_footprints.at(FootprintType::STEERING_STUCK).empty());
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorWithDelay)
{
  // Create a trajectory with enough points to span past the delay
  pred_traj_.clear();
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = i * 1.0;
    p.time_from_start = rclcpp::Duration::from_seconds(i * 0.1);
    p.longitudinal_velocity_mps = 10.0;
    p.front_wheel_angle_rad = 0.0;
    pred_traj_.push_back(p);
  }

  // Set delay to 0.25s.
  // Points at 0.0s, 0.1s, and 0.2s will be processed in the pre-delay loop.
  // Points at 0.3s and 0.4s will be processed in the bicycle model loop.
  auto steer_config =
    std::get<SteeringConfig>(param_.abnormality_configs[FootprintType::STEERING_STUCK]);
  steer_config.delay_s = 0.25;
  param_.abnormality_configs[FootprintType::STEERING_STUCK] = steer_config;

  SteeringFootprintGenerator generator(FootprintType::STEERING_STUCK);
  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  ASSERT_EQ(footprints.size(), pred_traj_.size());

  plot_steering_footprints(
    pred_traj_, footprints, "Steering Stuck with 0.6s Delay", "test_steering_footprint");
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorArcMotion)
{
  // 1. Create a longer trajectory to clearly see the "arc" effect of stuck steering
  pred_traj_.clear();
  for (int i = 0; i < 15; ++i) {
    TrajectoryPoint p;
    // Assuming the original planned path was a straight line along the X axis
    p.pose.position.x = i * 1.5;
    p.pose.position.y = 0.0;
    p.pose.orientation.w = 1.0;
    p.time_from_start = rclcpp::Duration::from_seconds(i * 0.15);
    p.longitudinal_velocity_mps = 10.0;

    // Simulate the steering wheel being stuck at ~11.4 degrees
    p.front_wheel_angle_rad = 0.2;
    pred_traj_.push_back(p);
  }

  // 2. Ensure config delay is 0.0 so it immediately enters the bicycle model logic
  auto steer_config =
    std::get<SteeringConfig>(param_.abnormality_configs[FootprintType::STEERING_STUCK]);
  steer_config.delay_s = 0.0;
  steer_config.offset_rps = 0.0;
  param_.abnormality_configs[FootprintType::STEERING_STUCK] = steer_config;

  // 3. Generate the footprints
  SteeringFootprintGenerator generator(FootprintType::STEERING_STUCK);
  const auto footprints =
    generator.generate(pred_traj_, vehicle_info_, param_, uncertainty_margin_);

  ASSERT_EQ(footprints.size(), pred_traj_.size());
  plot_steering_footprints(
    pred_traj_, footprints, "Steering Stuck Abnormality Simulation", "test_steering_footprint");
}

}  // namespace autoware::boundary_departure_checker
