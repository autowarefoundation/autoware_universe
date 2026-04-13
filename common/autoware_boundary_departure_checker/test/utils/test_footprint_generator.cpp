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

#include "autoware/boundary_departure_checker/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "test_plot_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{

namespace
{
void plot_steering_footprints(
  [[maybe_unused]] const TrajectoryPoints & pred_traj,
  [[maybe_unused]] const std::vector<autoware_utils_geometry::LinearRing2d> & footprints,
  [[maybe_unused]] const std::string & title, [[maybe_unused]] const std::string & sub_dir)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
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

    autoware::boundary_departure_checker::save_figure(plt, sub_dir);
  });
#endif
}

void plot_sides_results(
  [[maybe_unused]] const footprints::Footprints & test_footprints,
  [[maybe_unused]] const std::vector<Side<autoware_utils_geometry::Segment2d>> & sides_array)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    for (size_t i = 0; i < test_footprints.size(); ++i) {
      std::vector<double> fx, fy;
      for (const auto & p : test_footprints[i]) {
        fx.push_back(p.x());
        fy.push_back(p.y());
      }
      // Plot the full footprint polygon in gray
      plt.plot(
        Args(fx, fy),
        Kwargs("color"_a = "gray", "alpha"_a = 0.5, "label"_a = i == 0 ? "Full Footprint" : ""));

      auto left_side = sides_array[i].left;
      auto right_side = sides_array[i].right;

      // Plot the extracted left side in blue
      plt.plot(
        Args(
          std::vector<double>{left_side.first.x(), left_side.second.x()},
          std::vector<double>{left_side.first.y(), left_side.second.y()}),
        Kwargs(
          "color"_a = "blue", "linewidth"_a = 2.0,
          "label"_a = i == 0 ? "Left Extracted Side" : ""));

      // Plot the extracted right side in red
      plt.plot(
        Args(
          std::vector<double>{right_side.first.x(), right_side.second.x()},
          std::vector<double>{right_side.first.y(), right_side.second.y()}),
        Kwargs(
          "color"_a = "red", "linewidth"_a = 2.0,
          "label"_a = i == 0 ? "Right Extracted Side" : ""));
    }

    plt.axis(Args("equal"));
    plt.title(Args("Extracted Ego Sides from Footprints"));
    plt.legend();
    autoware::boundary_departure_checker::save_figure(plt, "test_get_sides_from_footprints");
  });
#endif
}
}  // namespace

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

    // 4. Setup base uncertainty margin
    pose_with_cov_.pose.orientation.w = 1.0;
    for (auto & c : pose_with_cov_.covariance) {
      c = 0.0;
    }
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  TrajectoryPoints pred_traj_;
  geometry_msgs::msg::PoseWithCovariance pose_with_cov_;
};

// Evaluates normal footprint generation without extra covariance-based margin.
TEST_F(FootprintGeneratorTest, TestNormalFootprintGenerator)
{
  // Act:
  const auto footprints = footprints::generate(pred_traj_, vehicle_info_, pose_with_cov_);

  // Assert:
  ASSERT_EQ(footprints.size(), pred_traj_.size());
  // Expected x: wheelbase(2.79) + front_overhang(1.0) = 3.79m
  const double expected_x = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
}

// Evaluates that an empty trajectory results in empty footprints.
TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;

  // Act:
  const auto footprints = footprints::generate(empty_traj, vehicle_info_, pose_with_cov_);

  // Assert:
  EXPECT_TRUE(footprints.empty());
}

// Evaluates side extraction behavior from an empty footprint list.
TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprintsEmpty)
{
  // Arrange:
  footprints::Footprints empty_footprints;

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(empty_footprints);

  // Assert:
  EXPECT_TRUE(sides_array.empty());
}

// Evaluates the extraction of left and right vehicle sides from footprint polygons.
TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprints)
{
  // Arrange:
  using autoware::vehicle_info_utils::VehicleInfo;

  const auto base_fp = vehicle_info_.createFootprint(0.0, 0.0);
  footprints::Footprints test_footprints = {base_fp};

  // offset by 5.0m in X
  auto offset_fp = base_fp;
  for (auto & p : offset_fp) {
    p = autoware_utils_geometry::Point2d{p.x() + 5.0, p.y()};
  }
  test_footprints.push_back(offset_fp);

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(test_footprints);

  // Assert:
  ASSERT_EQ(sides_array.size(), 2);
  // first footprint
  EXPECT_DOUBLE_EQ(sides_array[0].left.first.x(), base_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.first.y(), base_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.x(), base_fp[VehicleInfo::RearLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.y(), base_fp[VehicleInfo::RearLeftIndex].y());

  EXPECT_DOUBLE_EQ(sides_array[0].right.first.x(), base_fp[VehicleInfo::FrontRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.first.y(), base_fp[VehicleInfo::FrontRightIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.x(), base_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.y(), base_fp[VehicleInfo::RearRightIndex].y());

  // second footprint
  EXPECT_DOUBLE_EQ(sides_array[1].left.first.x(), offset_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].left.first.y(), offset_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.x(), offset_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.y(), offset_fp[VehicleInfo::RearRightIndex].y());

  // Assert: (Plotting)
  plot_sides_results(test_footprints, sides_array);
}
}  // namespace autoware::boundary_departure_checker
