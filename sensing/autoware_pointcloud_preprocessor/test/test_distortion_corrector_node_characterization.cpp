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

// Characterization test for the distortion corrector node.
//
// Records what the node currently does, seen from outside, so a rewrite of the internals is
// provably invisible to the rest of the system. The core classes are already unit-tested by
// test_distortion_corrector_node.cpp; this file covers the ROS node layer instead.
//
// A "...KnownIssue..." test pins behavior that is arguably wrong, so that fixing it is a
// deliberate change. One case is deliberately not pinned -- see
// StillPublishesWhenTheImuTransformIsMissing.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;

// ---------------------------------------------------------------------------------------
// The world every test runs in
// ---------------------------------------------------------------------------------------

constexpr char plugin_name[] = "autoware::pointcloud_preprocessor::DistortionCorrectorComponent";
constexpr char package_name[] = "autoware_pointcloud_preprocessor";

// Frames are prefixed so this test can never collide with the TF tree of another test binary
// sharing the same /tf_static.
constexpr char base_frame[] = "char_base_link";
constexpr char lidar_frame[] = "char_lidar_top";
constexpr char imu_frame[] = "char_imu_link";
// Rotated 180 degrees about x, so that a transformed angular velocity is trivially checkable.
constexpr char imu_flipped_frame[] = "char_imu_flipped";
// Deliberately absent from the TF tree published by this test.
constexpr char unmapped_frame[] = "char_unmapped_lidar";

// Message stamps are arbitrary but must stay well clear of the wall clock, so that a
// clock-based comparison inside the node cannot accidentally succeed.
constexpr double base_stamp_sec = 100.0;

// Each point of a generated cloud is stamped 10 ms after the previous one, so a 10-point cloud
// spans 90 ms -- just inside the core's 100 ms twist/IMU association window.
constexpr double point_interval_sec = 0.01;
constexpr size_t num_points = 10;

// Tight where the arithmetic is exact; looser for rotations, which go through the
// sin_and_cos() lookup table (~2.4e-4 of displacement at this cloud's 10 m radius).
constexpr float exact_tolerance = 1e-4F;
constexpr float rotation_tolerance = 1e-3F;
// opencv_fast_atan2(), used for the azimuth update, is accurate to a fraction of a degree.
constexpr float azimuth_tolerance = 0.02F;

enum class Layout {
  // PointXYZIRCAEDT: the only layout the node accepts.
  xyzircaedt,
  // Four fields only; rejected by is_data_layout_compatible_with_point_xyzircaedt().
  xyzi,
};

// Azimuth conventions a real sensor might use. The node only rewrites azimuths when
// update_azimuth_and_distance is on, and only if it can infer the convention from the cloud.
enum class AzimuthConvention { cartesian, velodyne };

// All nine sign combinations of (x, y) plus the degenerate origin, so that a sign or
// quadrant error anywhere in the pipeline shows up on at least one point.
const std::vector<std::array<float, 3>> & source_points()
{
  static const std::vector<std::array<float, 3>> points = {
    {0.0F, 0.0F, 0.0F},   {0.0F, 0.0F, 0.0F},   {10.0F, 0.0F, 1.0F},  {5.0F, -5.0F, 2.0F},
    {0.0F, -10.0F, 3.0F}, {-5.0F, -5.0F, 4.0F}, {-10.0F, 0.0F, 5.0F}, {-5.0F, 5.0F, -5.0F},
    {0.0F, 10.0F, -4.0F}, {5.0F, 5.0F, -3.0F},
  };
  return points;
}

// ---------------------------------------------------------------------------------------
// Small value helpers
// ---------------------------------------------------------------------------------------

rclcpp::Time to_time(double seconds)
{
  const auto sec = static_cast<int32_t>(seconds);
  const auto nanosec = static_cast<uint32_t>(std::llround((seconds - sec) * 1e9));
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
}

// Velodyne convention: x-axis is 0, angle grows clockwise. Written out here so the azimuth
// assertions test the node against the sensor convention rather than against itself.
float velodyne_azimuth_of(float x, float y)
{
  float cartesian = std::atan2(y, x);
  if (cartesian < 0.0F) cartesian += 2.0F * static_cast<float>(M_PI);
  float velodyne = 2.0F * static_cast<float>(M_PI) - cartesian;
  if (velodyne >= 2.0F * static_cast<float>(M_PI)) velodyne = 0.0F;
  return velodyne;
}

float azimuth_of(float x, float y, AzimuthConvention convention)
{
  return convention == AzimuthConvention::velodyne ? velodyne_azimuth_of(x, y) : std::atan2(y, x);
}

struct Point
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float azimuth{0.0F};
  float distance{0.0F};
  uint32_t time_stamp{0};
};

std::vector<Point> read_points(const PointCloud2 & cloud)
{
  std::vector<Point> points;
  if (cloud.width * cloud.height == 0) return points;

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> it_azimuth(cloud, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> it_distance(cloud, "distance");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> it_time(cloud, "time_stamp");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_azimuth, ++it_distance, ++it_time) {
    points.push_back(Point{*it_x, *it_y, *it_z, *it_azimuth, *it_distance, *it_time});
  }
  return points;
}

// The nanosecond offset the i-th point of a generated cloud carries in its time_stamp field.
uint32_t point_time_offset_ns(size_t index)
{
  return static_cast<uint32_t>(std::llround(point_interval_sec * 1e9)) *
         static_cast<uint32_t>(index);
}

// ---------------------------------------------------------------------------------------
// Node parameters. All six are declared without a default, so all must be supplied.
// ---------------------------------------------------------------------------------------

struct NodeParams
{
  bool use_imu{false};
  bool use_3d_distortion_correction{false};
  bool update_azimuth_and_distance{false};
  // Large enough that the latency diagnostic never fires on a loaded CI machine.
  double processing_time_threshold_sec{10.0};
  double timestamp_mismatch_fraction_threshold{0.5};

  // Spelled as setters rather than designated initializers, which are C++20.
  NodeParams & with_imu()
  {
    use_imu = true;
    return *this;
  }
  NodeParams & with_3d()
  {
    use_3d_distortion_correction = true;
    return *this;
  }
  NodeParams & with_azimuth_update()
  {
    update_azimuth_and_distance = true;
    return *this;
  }
};

// Loaded once per test binary; nullptr if the plugin is not registered. Leaked on purpose:
// class_loader would otherwise unload the library after rclcpp::shutdown() and abort.
std::shared_ptr<rclcpp_components::NodeFactory> get_component_factory()
{
  static auto * factory = [] {
    auto * cached = new std::shared_ptr<rclcpp_components::NodeFactory>();
    auto * manager = new rclcpp_components::ComponentManager();
    for (const auto & resource : manager->get_component_resources(package_name)) {
      if (resource.first == plugin_name) {
        *cached = manager->create_component_factory(resource);
        break;
      }
    }
    return cached;
  }();
  return *factory;
}

// The value of `key` in `status`, or nullptr if the status does not carry it.
const std::string * get_value_of(const DiagnosticStatus & status, const std::string & key)
{
  const auto it = std::find_if(
    status.values.begin(), status.values.end(),
    [&key](const auto & entry) { return entry.key == key; });
  return it == status.values.end() ? nullptr : &it->value;
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Test fixture: one verb per thing a test needs to do, so the tests stay flat.
// ---------------------------------------------------------------------------------------

class DistortionCorrectorCharacterizationTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    executor_.reset();
    node_wrapper_ = rclcpp_components::NodeInstanceWrapper();
    test_node_.reset();
  }

  // Brings the node up with `params` and connects every publisher and subscriber this test
  // could need.
  void start(const NodeParams & params)
  {
    params_ = params;

    // Each test gets a private namespace named after itself, so nothing leaks between tests
    // and every topic in a failure message says which test owns it.
    test_node_ = std::make_shared<rclcpp::Node>("characterization_driver", test_namespace());
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    // Static TF is transient_local, so the node picks it up whenever it starts.
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
    tf_broadcaster_->sendTransform(make_static_transforms());

    create_input_publishers();
    create_output_subscriptions();

    load_node_under_test();

    // The node drops any cloud that arrives before something is subscribed to its output
    // (see the points_sub_count guard), so discovery has to finish before the first publish.
    ASSERT_TRUE(wait_until(
      [this] { return cloud_publisher_->get_subscription_count() > 0; }, std::chrono::seconds(5)))
      << "the node never subscribed to " << input_cloud_topic();
    spin_for(std::chrono::milliseconds(300));
  }

  // -- driving ------------------------------------------------------------------------

  // Twist and IMU are drained by polling from inside the pointcloud callback, so they must be
  // published -- and given time to land in the middleware queue -- before the cloud.
  void publish_twist(double stamp_sec, double linear_x, double angular_z)
  {
    TwistWithCovarianceStamped msg;
    msg.header.stamp = to_time(stamp_sec);
    msg.header.frame_id = base_frame;
    msg.twist.twist.linear.x = linear_x;
    msg.twist.twist.angular.z = angular_z;
    twist_publisher_->publish(msg);
    spin_for(std::chrono::milliseconds(50));
  }

  void publish_full_twist(
    double stamp_sec, const std::array<double, 3> & linear, const std::array<double, 3> & angular)
  {
    TwistWithCovarianceStamped msg;
    msg.header.stamp = to_time(stamp_sec);
    msg.header.frame_id = base_frame;
    msg.twist.twist.linear.x = linear[0];
    msg.twist.twist.linear.y = linear[1];
    msg.twist.twist.linear.z = linear[2];
    msg.twist.twist.angular.x = angular[0];
    msg.twist.twist.angular.y = angular[1];
    msg.twist.twist.angular.z = angular[2];
    twist_publisher_->publish(msg);
    spin_for(std::chrono::milliseconds(50));
  }

  void publish_imu(
    double stamp_sec, double wx, double wy, double wz, const std::string & frame = imu_frame)
  {
    Imu msg;
    msg.header.stamp = to_time(stamp_sec);
    msg.header.frame_id = frame;
    msg.angular_velocity.x = wx;
    msg.angular_velocity.y = wy;
    msg.angular_velocity.z = wz;
    imu_publisher_->publish(msg);
    spin_for(std::chrono::milliseconds(50));
  }

  PointCloud2 publish_cloud(
    const std::string & frame, double stamp_sec, Layout layout = Layout::xyzircaedt,
    AzimuthConvention convention = AzimuthConvention::cartesian, bool empty = false)
  {
    const auto cloud = make_cloud(frame, stamp_sec, layout, convention, empty);
    cloud_publisher_->publish(cloud);
    spin_for(std::chrono::milliseconds(50));
    return cloud;
  }

  // -- observing ------------------------------------------------------------------------

  void spin_for(std::chrono::nanoseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(5));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  template <typename Predicate>
  bool wait_until(const Predicate & predicate, std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) return true;
      executor_->spin_some(std::chrono::milliseconds(5));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
  }

  // Waits for the n-th output cloud (1-based), then spins briefly so an unexpected extra
  // publication is recorded too. Generous timeout: a missing TF costs the node a full second.
  PointCloud2 await_output_cloud(
    size_t count = 1, std::chrono::nanoseconds timeout = std::chrono::seconds(6))
  {
    EXPECT_TRUE(wait_until([this, count] { return output_clouds_.size() >= count; }, timeout))
      << "expected " << count << " output cloud(s) on " << output_cloud_topic() << ", got "
      << output_clouds_.size();
    spin_for(std::chrono::milliseconds(250));
    // count is 1-based; 0 would wrap `count - 1` around to SIZE_MAX.
    return count == 0 || output_clouds_.size() < count ? PointCloud2{}
                                                       : output_clouds_.at(count - 1);
  }

  DiagnosticStatus await_diagnostic(
    size_t count = 1, std::chrono::nanoseconds timeout = std::chrono::seconds(6))
  {
    EXPECT_TRUE(wait_until([this, count] { return diagnostics_.size() >= count; }, timeout))
      << "expected " << count << " diagnostic(s) from " << node_name_ << ", got "
      << diagnostics_.size();
    spin_for(std::chrono::milliseconds(250));
    return count == 0 || diagnostics_.size() < count ? DiagnosticStatus{}
                                                     : diagnostics_.at(count - 1);
  }

  // -- assertions shared by several tests ----------------------------------------------

  // Every field the node is allowed to touch, compared against the cloud as published.
  static void expect_cloud_unchanged(const PointCloud2 & output, const PointCloud2 & input)
  {
    EXPECT_EQ(output.width, input.width);
    EXPECT_EQ(output.height, input.height);
    EXPECT_EQ(output.point_step, input.point_step);
    EXPECT_EQ(output.header.frame_id, input.header.frame_id);
    EXPECT_EQ(rclcpp::Time(output.header.stamp), rclcpp::Time(input.header.stamp));
    ASSERT_EQ(output.data, input.data) << "the node rewrote a cloud it should have passed through";
  }

  // Compares against the source points displaced by `shift`, which the caller computes from
  // the scenario rather than from the node.
  static void expect_points_shifted_by(
    const PointCloud2 & output, const std::vector<std::array<float, 3>> & shifts, float tolerance)
  {
    const auto points = read_points(output);
    ASSERT_EQ(points.size(), source_points().size());
    // Without this, a short `shifts` throws out of at() instead of failing the expectation.
    ASSERT_EQ(shifts.size(), source_points().size()) << "expected one shift per source point";
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & source = source_points().at(i);
      const auto & shift = shifts.at(i);
      EXPECT_NEAR(points.at(i).x, source[0] + shift[0], tolerance) << "point " << i << " x";
      EXPECT_NEAR(points.at(i).y, source[1] + shift[1], tolerance) << "point " << i << " y";
      EXPECT_NEAR(points.at(i).z, source[2] + shift[2], tolerance) << "point " << i << " z";
    }
  }

  // Closed form of the 2D corrector's accumulated heading for a twist with no linear
  // component, written out here so the expectation does not come from the code under test.
  static std::vector<std::array<float, 3>> rotation_shifts(double rate)
  {
    std::vector<std::array<float, 3>> shifts;
    for (size_t i = 0; i < source_points().size(); ++i) {
      const auto & point = source_points().at(i);
      const auto theta = rate * point_interval_sec * static_cast<double>(i);
      const auto rotated_x = point[0] * std::cos(theta) - point[1] * std::sin(theta);
      const auto rotated_y = point[0] * std::sin(theta) + point[1] * std::cos(theta);
      shifts.push_back(
        {static_cast<float>(rotated_x - point[0]), static_cast<float>(rotated_y - point[1]), 0.0F});
    }
    return shifts;
  }

  // Travelling along base_link's x at `speed` m/s; each point is displaced by the distance
  // covered since the cloud's first point.
  static std::vector<std::array<float, 3>> linear_shifts(double speed)
  {
    std::vector<std::array<float, 3>> shifts;
    for (size_t i = 0; i < source_points().size(); ++i) {
      shifts.push_back(
        {static_cast<float>(speed * point_interval_sec * static_cast<double>(i)), 0.0F, 0.0F});
    }
    return shifts;
  }

  std::vector<rclcpp::TopicEndpointInfo> get_publishers_on(const std::string & topic)
  {
    return endpoints_of_node_under_test(test_node_->get_publishers_info_by_topic(topic));
  }

  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_on(const std::string & topic)
  {
    return endpoints_of_node_under_test(test_node_->get_subscriptions_info_by_topic(topic));
  }

  // -- topic names ---------------------------------------------------------------------
  //
  // The node's topics are private ("~/input/twist") so they sit under <ns>/<node name>; the
  // debug publisher uses a relative name, so its topics sit directly under <ns>.

  std::string test_namespace() const
  {
    return "/" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
  }

  std::string node_prefix() const { return test_namespace() + "/" + node_name_; }
  std::string input_cloud_topic() const { return node_prefix() + "/input/pointcloud"; }
  std::string input_twist_topic() const { return node_prefix() + "/input/twist"; }
  std::string input_imu_topic() const { return node_prefix() + "/input/imu"; }
  std::string output_cloud_topic() const { return node_prefix() + "/output/pointcloud"; }
  std::string debug_topic(const std::string & leaf) const
  {
    return test_namespace() + "/distortion_corrector/debug/" + leaf;
  }

  std::vector<PointCloud2> output_clouds_;
  std::vector<DiagnosticStatus> diagnostics_;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> cyclic_times_;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> processing_times_;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> pipeline_latencies_;

  std::string node_name_{"distortion_corrector_under_test"};
  NodeParams params_;

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

private:
  std::vector<rclcpp::TopicEndpointInfo> endpoints_of_node_under_test(
    const std::vector<rclcpp::TopicEndpointInfo> & endpoints) const
  {
    std::vector<rclcpp::TopicEndpointInfo> matching;
    std::copy_if(
      endpoints.begin(), endpoints.end(), std::back_inserter(matching),
      [this](const auto & endpoint) { return endpoint.node_name() == node_name_; });
    return matching;
  }

  // One base_link -> child extrinsic, i.e. where the sensor sits on the vehicle. Per the TF
  // convention it maps points from the child frame into base_link: p_base = R * p_child + t.
  static geometry_msgs::msg::TransformStamped make_transform(
    const std::string & child, double x, double y, double z, double qx, double qy, double qz,
    double qw)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = to_time(0.0);
    tf.header.frame_id = base_frame;
    tf.child_frame_id = child;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    return tf;
  }

  static std::vector<geometry_msgs::msg::TransformStamped> make_static_transforms()
  {
    // The lidar is offset and yawed 90 degrees, so that a cloud given in the lidar frame is
    // only undistorted correctly if the transform is applied in both directions.
    const double half_sqrt2 = std::sqrt(0.5);
    return {
      make_transform(lidar_frame, 5.0, 5.0, 5.0, 0.0, 0.0, half_sqrt2, half_sqrt2),
      // Unrotated, so an IMU's angular velocity reaches the corrector unchanged and the
      // tests that care about the IMU can state an exact expectation.
      make_transform(imu_frame, 1.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0),
      // Rolled 180 degrees: (wx, wy, wz) becomes (wx, -wy, -wz).
      make_transform(imu_flipped_frame, 1.0, 1.0, 3.0, 1.0, 0.0, 0.0, 0.0)};
  }

  void create_input_publishers()
  {
    cloud_publisher_ = test_node_->create_publisher<PointCloud2>(
      input_cloud_topic(), rclcpp::SensorDataQoS().keep_last(10));
    twist_publisher_ = test_node_->create_publisher<TwistWithCovarianceStamped>(
      input_twist_topic(), rclcpp::QoS(100));
    imu_publisher_ = test_node_->create_publisher<Imu>(input_imu_topic(), rclcpp::QoS(100));
  }

  void create_output_subscriptions()
  {
    output_subscription_ = test_node_->create_subscription<PointCloud2>(
      output_cloud_topic(), rclcpp::SensorDataQoS().keep_last(10),
      [this](PointCloud2::ConstSharedPtr msg) { output_clouds_.push_back(*msg); });

    diagnostics_subscription_ = test_node_->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(100), [this](DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.hardware_id == node_name_) diagnostics_.push_back(status);
        }
      });

    using autoware_internal_debug_msgs::msg::Float64Stamped;
    cyclic_time_subscription_ = test_node_->create_subscription<Float64Stamped>(
      debug_topic("cyclic_time_ms"), rclcpp::QoS(10),
      [this](Float64Stamped::ConstSharedPtr msg) { cyclic_times_.push_back(*msg); });
    processing_time_subscription_ = test_node_->create_subscription<Float64Stamped>(
      debug_topic("processing_time_ms"), rclcpp::QoS(10),
      [this](Float64Stamped::ConstSharedPtr msg) { processing_times_.push_back(*msg); });
    pipeline_latency_subscription_ = test_node_->create_subscription<Float64Stamped>(
      debug_topic("pipeline_latency_ms"), rclcpp::QoS(10),
      [this](Float64Stamped::ConstSharedPtr msg) { pipeline_latencies_.push_back(*msg); });
  }

  void load_node_under_test()
  {
    const auto factory = get_component_factory();
    ASSERT_NE(factory, nullptr) << "component " << plugin_name << " is not registered in package "
                                << package_name;

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "-r", "__ns:=" + test_namespace(), "-r", "__node:=" + node_name_});
    options.parameter_overrides(
      {{"base_frame", std::string(base_frame)},
       {"use_imu", params_.use_imu},
       {"use_3d_distortion_correction", params_.use_3d_distortion_correction},
       {"update_azimuth_and_distance", params_.update_azimuth_and_distance},
       {"processing_time_threshold_sec", params_.processing_time_threshold_sec},
       {"timestamp_mismatch_fraction_threshold", params_.timestamp_mismatch_fraction_threshold}});

    node_wrapper_ = factory->create_node_instance(options);
    executor_->add_node(node_wrapper_.get_node_base_interface());
  }

  static PointCloud2 make_cloud(
    const std::string & frame, double stamp_sec, Layout layout, AzimuthConvention convention,
    bool empty)
  {
    PointCloud2 cloud;
    cloud.header.stamp = to_time(stamp_sec);
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    if (layout == Layout::xyzi) {
      modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(empty ? 0 : source_points().size());
      if (!empty) {
        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
        for (const auto & point : source_points()) {
          *it_x = point[0];
          *it_y = point[1];
          *it_z = point[2];
          ++it_x;
          ++it_y;
          ++it_z;
        }
      }
      return cloud;
    }

    modifier.setPointCloud2Fields(
      10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
      sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
      "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
      sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
      "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(empty ? 0 : source_points().size());
    if (empty) return cloud;

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_azimuth(cloud, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> it_distance(cloud, "distance");
    sensor_msgs::PointCloud2Iterator<uint32_t> it_time(cloud, "time_stamp");

    for (size_t i = 0; i < source_points().size(); ++i) {
      const auto & point = source_points().at(i);
      *it_x = point[0];
      *it_y = point[1];
      *it_z = point[2];
      *it_azimuth = azimuth_of(point[0], point[1], convention);
      *it_distance = std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
      *it_time = point_time_offset_ns(i);
      ++it_x;
      ++it_y;
      ++it_z;
      ++it_azimuth;
      ++it_distance;
      ++it_time;
    }
    return cloud;
  }

  rclcpp_components::NodeInstanceWrapper node_wrapper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
  rclcpp::Publisher<Imu>::SharedPtr imu_publisher_;

  rclcpp::Subscription<PointCloud2>::SharedPtr output_subscription_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    cyclic_time_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pipeline_latency_subscription_;
};

// ---------------------------------------------------------------------------------------
// The topic interface
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, AdvertisesUndistortedPointcloudTopic)
{
  // Arrange
  start(NodeParams{});

  // Act
  const auto publishers = get_publishers_on(output_cloud_topic());

  // Assert
  ASSERT_EQ(publishers.size(), 1U);
  EXPECT_EQ(publishers.front().topic_type(), "sensor_msgs/msg/PointCloud2");
  // SensorDataQoS: best effort, volatile, depth 5.
  EXPECT_EQ(publishers.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(publishers.front().qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  EXPECT_EQ(publishers.front().qos_profile().depth(), 5U);
}

TEST_F(DistortionCorrectorCharacterizationTest, SubscribesToPointcloudTwistAndImu)
{
  // Arrange
  start(NodeParams{});

  // Act
  const auto cloud_subscriptions = get_subscriptions_on(input_cloud_topic());
  const auto twist_subscriptions = get_subscriptions_on(input_twist_topic());
  const auto imu_subscriptions = get_subscriptions_on(input_imu_topic());

  // Assert
  ASSERT_EQ(cloud_subscriptions.size(), 1U);
  EXPECT_EQ(
    cloud_subscriptions.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);

  // The twist and IMU queues are sized for a twist rate far above the cloud rate, so that a
  // whole cloud interval of samples is never dropped by the middleware.
  ASSERT_EQ(twist_subscriptions.size(), 1U);
  EXPECT_EQ(twist_subscriptions.front().qos_profile().depth(), 100U);
  ASSERT_EQ(imu_subscriptions.size(), 1U);
  EXPECT_EQ(imu_subscriptions.front().qos_profile().depth(), 100U);
}

TEST_F(DistortionCorrectorCharacterizationTest, SubscribesToImuEvenWhenImuIsDisabled)
{
  // Arrange
  // use_imu gates whether the samples are consumed, not whether the subscription exists.
  start(NodeParams{});

  // Act
  const auto imu_subscriptions = get_subscriptions_on(input_imu_topic());

  // Assert
  EXPECT_EQ(imu_subscriptions.size(), 1U);
}

TEST_F(DistortionCorrectorCharacterizationTest, AdvertisesDebugTopicsAndPublishesThemPerCloud)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 1.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  await_output_cloud();

  // Assert
  EXPECT_EQ(cyclic_times_.size(), 1U);
  EXPECT_EQ(processing_times_.size(), 1U);
  EXPECT_EQ(pipeline_latencies_.size(), 1U);
}

// ---------------------------------------------------------------------------------------
// Inputs the node declines to undistort. Each is still republished byte for byte.
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, RepublishesCloudUnchangedWhenNoTwistHasArrived)
{
  // Arrange
  start(NodeParams{});

  // Act
  const auto input = publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_cloud_unchanged(output, input);
}

TEST_F(DistortionCorrectorCharacterizationTest, RepublishesEmptyCloudUnchanged)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  const auto input = publish_cloud(
    base_frame, base_stamp_sec, Layout::xyzircaedt, AzimuthConvention::cartesian, true);
  const auto output = await_output_cloud();

  // Assert
  EXPECT_EQ(output.width, 0U);
  expect_cloud_unchanged(output, input);
}

TEST_F(DistortionCorrectorCharacterizationTest, RepublishesCloudWithIncompatibleLayoutUnchanged)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  const auto input = publish_cloud(base_frame, base_stamp_sec, Layout::xyzi);
  const auto output = await_output_cloud();

  // Assert
  expect_cloud_unchanged(output, input);
}

TEST_F(DistortionCorrectorCharacterizationTest, PublishesExactlyOneCloudPerInputCloud)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  publish_cloud(base_frame, base_stamp_sec + 0.1);
  await_output_cloud(2);

  // Assert
  EXPECT_EQ(output_clouds_.size(), 2U);
}

// ---------------------------------------------------------------------------------------
// Undistortion, 2D
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, LinearTwistShiftsEachPointByItsOwnTimeOffset)
{
  // Arrange
  constexpr double speed = 10.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, speed, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  // 10 m/s over 10 ms steps: the i-th point moves 0.1 * i metres along x.
  expect_points_shifted_by(output, linear_shifts(speed), exact_tolerance);
}

TEST_F(DistortionCorrectorCharacterizationTest, RotationalTwistRotatesEachPointAboutBaseLink)
{
  // Arrange
  constexpr double rate = 1.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, 0.0, rate);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_points_shifted_by(output, rotation_shifts(rate), rotation_tolerance);
}

TEST_F(DistortionCorrectorCharacterizationTest, UsesImuAngularVelocityInsteadOfTwistAngularVelocity)
{
  // Arrange
  constexpr double imu_rate = 1.0;
  start(NodeParams{}.with_imu());
  // The twist's angular velocity is the opposite sign and five times the magnitude, so the
  // output can only match the IMU if the twist's yaw rate was discarded.
  publish_twist(base_stamp_sec, 0.0, -5.0);
  publish_imu(base_stamp_sec, 0.0, 0.0, imu_rate);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_points_shifted_by(output, rotation_shifts(imu_rate), rotation_tolerance);
}

TEST_F(DistortionCorrectorCharacterizationTest, RotatesImuAngularVelocityIntoTheBaseFrame)
{
  // Arrange
  constexpr double imu_rate = 1.0;
  start(NodeParams{}.with_imu());
  publish_twist(base_stamp_sec, 0.0, 0.0);
  // char_imu_flipped is rolled 180 degrees about x, so +z in the IMU frame is -z in base_link.
  publish_imu(base_stamp_sec, 0.0, 0.0, imu_rate, imu_flipped_frame);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_points_shifted_by(output, rotation_shifts(-imu_rate), rotation_tolerance);
}

TEST_F(DistortionCorrectorCharacterizationTest, IgnoresImuWhenImuIsDisabled)
{
  // Arrange
  constexpr double twist_rate = 1.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, 0.0, twist_rate);
  // Would dominate the yaw rate if it were consumed.
  publish_imu(base_stamp_sec, 0.0, 0.0, -5.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_points_shifted_by(output, rotation_shifts(twist_rate), rotation_tolerance);
}

TEST_F(DistortionCorrectorCharacterizationTest, UndistortsCloudGivenInTheLidarFrame)
{
  // Arrange
  constexpr double speed = 10.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, speed, 0.0);

  // Act
  publish_cloud(lidar_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  // char_lidar_top is yawed +90 deg, so a correction of +x in base_link lands as -y here.
  std::vector<std::array<float, 3>> shifts;
  for (size_t i = 0; i < num_points; ++i) {
    const auto travelled = static_cast<float>(speed * point_interval_sec * static_cast<double>(i));
    shifts.push_back({0.0F, -travelled, 0.0F});
  }
  expect_points_shifted_by(output, shifts, rotation_tolerance);
}

// ---------------------------------------------------------------------------------------
// Undistortion, 3D
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, ThreeDimensionalCorrectionShiftsPointsByLinearTwist)
{
  // Arrange
  constexpr double speed = 10.0;
  start(NodeParams{}.with_3d());
  publish_twist(base_stamp_sec, speed, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  expect_points_shifted_by(output, linear_shifts(speed), exact_tolerance);
}

TEST_F(
  DistortionCorrectorCharacterizationTest,
  ThreeDimensionalCorrectionUsesEveryLinearVelocityComponent)
{
  // Arrange
  // The 2D corrector reads only linear.x and angular.z; the 3D one reads all six components.
  // Driving y and z proves which strategy is in use.
  start(NodeParams{}.with_3d());
  publish_full_twist(base_stamp_sec, {0.0, 4.0, 8.0}, {0.0, 0.0, 0.0});

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  std::vector<std::array<float, 3>> shifts;
  for (size_t i = 0; i < num_points; ++i) {
    const auto elapsed = point_interval_sec * static_cast<double>(i);
    shifts.push_back({0.0F, static_cast<float>(4.0 * elapsed), static_cast<float>(8.0 * elapsed)});
  }
  expect_points_shifted_by(output, shifts, exact_tolerance);
}

// ---------------------------------------------------------------------------------------
// The azimuth and distance rewrite
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, LeavesAzimuthAndDistanceAloneByDefault)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  const auto input =
    publish_cloud(lidar_frame, base_stamp_sec, Layout::xyzircaedt, AzimuthConvention::velodyne);
  const auto output = await_output_cloud();

  // Assert
  const auto output_points = read_points(output);
  const auto input_points = read_points(input);
  ASSERT_EQ(output_points.size(), input_points.size());
  for (size_t i = 0; i < output_points.size(); ++i) {
    EXPECT_FLOAT_EQ(output_points.at(i).azimuth, input_points.at(i).azimuth) << "point " << i;
    EXPECT_FLOAT_EQ(output_points.at(i).distance, input_points.at(i).distance) << "point " << i;
  }
}

TEST_F(DistortionCorrectorCharacterizationTest, RewritesAzimuthAndDistanceFromTheCorrectedPosition)
{
  // Arrange
  start(NodeParams{}.with_azimuth_update());
  // A stationary vehicle keeps the positions put, so the azimuth and distance assertions are
  // about the rewrite alone.
  publish_twist(base_stamp_sec, 0.0, 0.0);

  // Act
  publish_cloud(lidar_frame, base_stamp_sec, Layout::xyzircaedt, AzimuthConvention::velodyne);
  const auto output = await_output_cloud();

  // Assert
  const auto points = read_points(output);
  ASSERT_EQ(points.size(), num_points);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & point = points.at(i);
    // distance is the norm of the corrected position, exactly.
    const auto norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    EXPECT_NEAR(point.distance, norm, exact_tolerance) << "point " << i << " distance";
    // azimuth is that position expressed in the sensor's own angle convention.
    EXPECT_NEAR(point.azimuth, velodyne_azimuth_of(point.x, point.y), azimuth_tolerance)
      << "point " << i << " azimuth";
  }
}

TEST_F(
  DistortionCorrectorCharacterizationTest,
  AbortsTheCallbackWhenAzimuthUpdateIsAskedForACloudAlreadyInTheBaseFrame)
{
  // Arrange
  start(NodeParams{}.with_azimuth_update());
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act & Assert
  // Fused because the act is what throws. A base_link cloud has no sensor azimuth, so the
  // core throws, and nothing in the node catches it before it reaches the spinning fixture.
  EXPECT_THROW(
    {
      publish_cloud(base_frame, base_stamp_sec);
      spin_for(std::chrono::seconds(2));
    },
    std::runtime_error);

  EXPECT_TRUE(output_clouds_.empty()) << "a cloud was published despite the aborted callback";
}

// ---------------------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, PublishesDiagnosticsForEveryCloud)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  await_output_cloud();
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.hardware_id, node_name_);
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  // Not "Distortion correction successful": DiagnosticsInterface overwrites the message of an
  // OK status with "OK", so the node's success string never reaches a subscriber.
  EXPECT_EQ(status.message, "OK");
  for (const auto * key :
       {"Pointcloud header timestamp", "Processing time (ms)", "Pipeline latency (ms)",
        "Timestamp mismatch count", "Timestamp mismatch fraction", "Use 3D distortion correction",
        "Update azimuth and distance"}) {
    EXPECT_NE(get_value_of(status, key), nullptr) << key << " is missing from the diagnostics";
  }
}

TEST_F(DistortionCorrectorCharacterizationTest, DiagnosticsEchoTheStrategyParameters)
{
  // Arrange
  start(NodeParams{}.with_3d());
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  await_output_cloud();
  const auto status = await_diagnostic();

  // Assert
  ASSERT_NE(get_value_of(status, "Use 3D distortion correction"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Use 3D distortion correction"), "True");
  ASSERT_NE(get_value_of(status, "Update azimuth and distance"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Update azimuth and distance"), "False");
}

TEST_F(DistortionCorrectorCharacterizationTest, ReportsNoTimestampMismatchWhenEveryPointFindsATwist)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  await_output_cloud();
  const auto status = await_diagnostic();

  // Assert
  ASSERT_NE(get_value_of(status, "Timestamp mismatch count"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Timestamp mismatch count"), "0");
}

TEST_F(
  DistortionCorrectorCharacterizationTest,
  GoesToErrorWhenTheTimestampMismatchFractionExceedsTheThreshold)
{
  // Arrange
  start(NodeParams{});
  // A twist a full second before the cloud is outside the core's 100 ms association window,
  // so no point can use it.
  publish_twist(base_stamp_sec - 1.0, 10.0, 0.0);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  await_output_cloud();
  const auto status = await_diagnostic();

  // Assert
  ASSERT_NE(get_value_of(status, "Timestamp mismatch count"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Timestamp mismatch count"), std::to_string(num_points));
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_NE(status.message.find("timestamp mismatch fraction"), std::string::npos)
    << "unexpected diagnostic message: " << status.message;
}

// ---------------------------------------------------------------------------------------
// Sensor queue hygiene. The core binary-searches its twist/IMU deques, so they must stay
// ordered and bounded. A surviving stale sample leaves the cloud uncorrected.
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, BackwardTimeJumpDiscardsTheTwistQueue)
{
  // Arrange
  // A rosbag restart: the first twist is 10 s ahead. Driving a cloud through it is what puts
  // the pre-jump sample into the core's queue.
  constexpr double speed = 10.0;
  constexpr double rewound_speed = 1.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec + 10.0, speed, 0.0);
  publish_cloud(base_frame, base_stamp_sec + 10.0);
  await_output_cloud(1);

  // Act
  publish_twist(base_stamp_sec, rewound_speed, 0.0);
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud(2);
  const auto status = await_diagnostic(2);

  // Assert
  // Only reachable if the pre-jump sample was dropped; had it survived, the search would
  // settle on the 10 s old sample and the cloud would come back untouched.
  expect_points_shifted_by(output, linear_shifts(rewound_speed), exact_tolerance);
  ASSERT_NE(get_value_of(status, "Timestamp mismatch count"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Timestamp mismatch count"), "0");
}

TEST_F(
  DistortionCorrectorCharacterizationTest, TwistOlderThanOneSecondIsDroppedWhenANewerOneArrives)
{
  // Arrange
  // 1.5 s apart, so inserting the second retires the first.
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);
  publish_twist(base_stamp_sec + 1.5, 1.0, 0.0);

  // Act
  const auto input = publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();
  const auto status = await_diagnostic();

  // Assert
  // The only surviving sample is 1.5 s away from every point, far outside the association
  // window, so nothing is corrected.
  expect_cloud_unchanged(output, input);
  ASSERT_NE(get_value_of(status, "Timestamp mismatch count"), nullptr);
  EXPECT_EQ(*get_value_of(status, "Timestamp mismatch count"), std::to_string(num_points));
}

// ---------------------------------------------------------------------------------------
// Missing transforms
// ---------------------------------------------------------------------------------------

TEST_F(DistortionCorrectorCharacterizationTest, StillPublishesWhenTheCloudTransformIsMissing)
{
  // Arrange
  constexpr double speed = 10.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, speed, 0.0);

  // Act
  publish_cloud(unmapped_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  // The lookup fails, so the node has no lidar-to-base transform -- but it undistorts the
  // cloud anyway, treating the sensor coordinates as if they were already base_link's.
  expect_points_shifted_by(output, linear_shifts(speed), exact_tolerance);
}

TEST_F(
  DistortionCorrectorCharacterizationTest,
  DiagnosticsDescribeTheCloudJustProcessedWhenItsTransformIsMissing)
{
  // Arrange
  // A first cloud with a matching twist. Asserted as a precondition: if this half is already
  // wrong, the second half proves nothing.
  start(NodeParams{});
  publish_twist(base_stamp_sec, 10.0, 0.0);
  publish_cloud(unmapped_frame, base_stamp_sec);
  await_output_cloud(1);
  const auto first = await_diagnostic(1);
  ASSERT_NE(get_value_of(first, "Timestamp mismatch count"), nullptr);
  ASSERT_EQ(*get_value_of(first, "Timestamp mismatch count"), "0");

  // Act
  // A second cloud half a second later with no new twist, so every point mismatches.
  publish_cloud(unmapped_frame, base_stamp_sec + 0.5);
  await_output_cloud(2);
  const auto second = await_diagnostic(2);

  // Assert
  // The diagnostics describe THIS cloud, not the previous one -- which only holds while the
  // mismatch counters are reset per cloud.
  ASSERT_NE(get_value_of(second, "Timestamp mismatch count"), nullptr);
  EXPECT_EQ(*get_value_of(second, "Timestamp mismatch count"), std::to_string(num_points));
}

TEST_F(DistortionCorrectorCharacterizationTest, StillPublishesWhenTheImuTransformIsMissing)
{
  // Arrange
  start(NodeParams{}.with_imu());
  publish_twist(base_stamp_sec, 10.0, 0.0);
  // char_unmapped_lidar is not in the TF tree, so the IMU-to-base lookup fails.
  publish_imu(base_stamp_sec, 0.0, 0.0, 1.0, unmapped_frame);

  // Act
  publish_cloud(base_frame, base_stamp_sec);
  const auto output = await_output_cloud();

  // Assert
  // Only the absence of a crash. The core rotates by an uninitialized matrix when the lookup
  // fails, so the positions are indeterminate -- fix that rather than pinning them.
  EXPECT_EQ(output.width, static_cast<uint32_t>(num_points));
  EXPECT_EQ(output.header.frame_id, base_frame);
}

TEST_F(
  DistortionCorrectorCharacterizationTest,
  KnownIssueFirstResolvedCloudTransformIsReusedForEveryLaterFrame)
{
  // Arrange
  // A base_link cloud resolves an identity transform and latches it. Asserted as a
  // precondition: the latch only means something if this cloud was corrected normally.
  constexpr double speed = 10.0;
  start(NodeParams{});
  publish_twist(base_stamp_sec, speed, 0.0);
  publish_cloud(base_frame, base_stamp_sec);
  const auto first = await_output_cloud(1);
  expect_points_shifted_by(first, linear_shifts(speed), exact_tolerance);

  // Act
  // A cloud from the yawed lidar now arrives.
  publish_twist(base_stamp_sec + 0.1, speed, 0.0);
  publish_cloud(lidar_frame, base_stamp_sec + 0.1);
  const auto second = await_output_cloud(2);

  // Assert
  // This is a bug:
  // The lookup is skipped because a transform is cached, so this cloud is corrected as though
  // it too were in base_link: the shift stays along x instead of becoming -y. Recorded as a
  // bug so fixing it is deliberate; compare UndistortsCloudGivenInTheLidarFrame.
  expect_points_shifted_by(second, linear_shifts(speed), exact_tolerance);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
