#include "waypoint_maker_utils/conversion.hpp"
#include "waypoint_maker_utils/dubug_maker.hpp"
#include "waypoint_maker_utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>

namespace wm = waypoint_maker;

TEST(Conversion, SpeedUnits)
{
  // 36 km/h == 10 m/s
  EXPECT_NEAR(wm::kmph2mps(36.0), 10.0, 1e-9);
  EXPECT_NEAR(wm::mps2kmph(10.0), 36.0, 1e-9);

  EXPECT_NEAR(wm::kmph2mps(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wm::mps2kmph(0.0), 0.0, 1e-12);
}

TEST(Conversion, QuaternionFromYaw)
{
  const double yaw = 1.234;  // rad
  auto q = wm::getQuaternionFromYaw(yaw);

  // yaw = 2*atan2(z, w) when roll=pitch=0
  const double yaw_back = 2.0 * std::atan2(q.z, q.w);
  EXPECT_NEAR(yaw_back, yaw, 1e-9);

  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  EXPECT_NEAR(norm, 1.0, 1e-12);
}

TEST(Utils, CountAndParseColumns)
{
  std::string line = " 1.0 , 2.0,3.0 , 4.0 ,5.0 ";
  EXPECT_EQ(wm::countColumns(line), static_cast<size_t>(5));

  std::vector<std::string> cols;
  wm::parseColumns(line, &cols);
  ASSERT_EQ(cols.size(), static_cast<size_t>(5));
  EXPECT_EQ(cols[0], "1.0");
  EXPECT_EQ(cols[1], "2.0");
  EXPECT_EQ(cols[2], "3.0");
  EXPECT_EQ(cols[3], "4.0");
  EXPECT_EQ(cols[4], "5.0");
}

TEST(Utils, VerifyFileConsistency_ValidAndInvalid)
{
  const std::string tmpdir = (std::filesystem::temp_directory_path() / "wm_utils_test").string();
  std::filesystem::create_directories(tmpdir);

  const std::string valid_csv = tmpdir + "/valid.csv";
  {
    std::ofstream ofs(valid_csv);
    ofs << "x,y,z,yaw,velocity\n";
    ofs << "0,0,0,0,10\n";
    ofs << "1,2,3,0.5,20\n";
  }
  EXPECT_TRUE(wm::verifyFileConsistency(valid_csv.c_str()));

  const std::string invalid_csv = tmpdir + "/invalid.csv";
  {
    std::ofstream ofs(invalid_csv);
    ofs << "x,y,z,yaw,velocity\n";
    ofs << "0,0,0,0\n";  // only 4 columns -> invalid
  }
  EXPECT_FALSE(wm::verifyFileConsistency(invalid_csv.c_str()));
}

TEST(Utils, ParseWaypoint_MaxVelocityFlag)
{
  autoware_planning_msgs::msg::TrajectoryPoint pt1;
  wm::parseWaypoint("1,2,3,1.57,36", &pt1, false, 5.0);
  EXPECT_DOUBLE_EQ(pt1.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pt1.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pt1.pose.position.z, 3.0);
  EXPECT_NEAR(pt1.longitudinal_velocity_mps, wm::kmph2mps(36.0), 1e-9);

  autoware_planning_msgs::msg::TrajectoryPoint pt2;
  wm::parseWaypoint("1,2,3,1.57,36", &pt2, true, 5.5);
  EXPECT_NEAR(pt2.longitudinal_velocity_mps, 5.5, 1e-9);
}

TEST(Utils, LoadAndCreateTrajectory)
{
  const std::string tmpdir = (std::filesystem::temp_directory_path() / "wm_utils_test2").string();
  std::filesystem::create_directories(tmpdir);

  const std::string csv = tmpdir + "/wp.csv";
  {
    std::ofstream ofs(csv);
    ofs << "x,y,z,yaw,velocity\n";
    ofs << "0,0,0,0,10\n";
    ofs << "1,0,0,0,20\n";
    ofs << "2,0,0,0,30\n";
  }

  auto traj = wm::createWaypointTrajectory(csv, false, 0.0);
  EXPECT_EQ(traj.header.frame_id, "map");
  ASSERT_EQ(traj.points.size(), static_cast<size_t>(3));
  EXPECT_DOUBLE_EQ(traj.points[0].pose.position.x, 0.0);
  EXPECT_NEAR(traj.points[0].longitudinal_velocity_mps, wm::kmph2mps(10.0), 1e-9);

  auto traj2 = wm::createWaypointTrajectory(csv, true, 4.2);
  ASSERT_EQ(traj2.points.size(), static_cast<size_t>(3));
  EXPECT_NEAR(traj2.points[1].longitudinal_velocity_mps, 4.2, 1e-9);
}

TEST(Utils, CurrentDateTime_Format)
{
  const std::string ts = wm::currentDateTime();
  std::regex re(R"(^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$)");
  EXPECT_TRUE(std::regex_match(ts, re));
}

TEST(Utils, DirExists)
{
  const std::string tmpdir = (std::filesystem::temp_directory_path() / "wm_utils_dir").string();
  std::filesystem::create_directories(tmpdir);
  EXPECT_TRUE(wm::dirExists(tmpdir));

  const std::string not_exists = tmpdir + "_nope";
  if (std::filesystem::exists(not_exists)) {
    std::filesystem::remove_all(not_exists);
  }
  EXPECT_FALSE(wm::dirExists(not_exists));
}

TEST(DebugMarkers, TrajectoryMarker)
{
  autoware_planning_msgs::msg::Trajectory t;
  t.header.frame_id = "map";
  for (int i = 0; i < 5; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = i * 1.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    t.points.push_back(p);
  }

  auto m = waypoint_maker::createTrajectoryMarkerArray(t);
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.ns, "trajectory_debug");
  EXPECT_EQ(m.type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(m.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(m.scale.x, 0.3);
  EXPECT_NEAR(m.color.g, 1.0, 1e-12);
  ASSERT_EQ(m.points.size(), static_cast<size_t>(5));
  EXPECT_DOUBLE_EQ(m.points[4].x, 4.0);
}

TEST(DebugMarkers, BoundMarkers)
{
  std::array<std::vector<geometry_msgs::msg::Point>, 2> bounds;
  geometry_msgs::msg::Point p;
  p.x = 1.0;
  p.y = 2.0;
  p.z = 0.0;
  bounds[0].push_back(p);
  p.x = 3.0;
  p.y = 4.0;
  bounds[1].push_back(p);

  auto arr = waypoint_maker::createBoundMarkerArray(bounds);
  ASSERT_EQ(arr.markers.size(), static_cast<size_t>(2));
  EXPECT_EQ(arr.markers[0].ns, "bound_debug");
  EXPECT_EQ(arr.markers[0].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(arr.markers[1].type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(arr.markers[0].points.size(), static_cast<size_t>(1));
  EXPECT_EQ(arr.markers[1].points.size(), static_cast<size_t>(1));
  EXPECT_DOUBLE_EQ(arr.markers[1].points[0].x, 3.0);
}

// gtest main with rclcpp init/shutdown (safe for logging/time usages)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}