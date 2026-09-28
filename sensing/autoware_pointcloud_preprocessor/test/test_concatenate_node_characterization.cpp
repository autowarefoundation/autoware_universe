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

// Characterization test for the concatenate-and-time-sync node.
//
// This file is a safety net for refactoring the concatenate_data module. It does not say what
// the node should do; it records what the node currently does, as seen from outside. While
// every assertion here keeps passing, an arbitrary rewrite of the internals is invisible to
// any other node in the system.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <autoware_sensing_msgs/msg/source_point_cloud_info.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo;
using autoware_sensing_msgs::msg::SourcePointCloudInfo;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

// ---------------------------------------------------------------------------------------
// The world every test runs in
// ---------------------------------------------------------------------------------------

constexpr char plugin_name[] =
  "autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent";
constexpr char package_name[] = "autoware_pointcloud_preprocessor";
constexpr char output_frame[] = "base_link";

constexpr size_t num_sensors = 3;
constexpr size_t num_points = 3;

// Sensor frames are prefixed so this test can never collide with the TF tree of another test.
const std::vector<std::string> sensor_frames = {
  "char_left_lidar", "char_right_lidar", "char_top_lidar"};

const std::vector<std::string> sensor_names = {"left", "right", "top"};

// base_link -> sensor translations, integral and distinct so expected points are obvious.
const std::vector<std::array<double, 3>> sensor_translations = {
  {0.0, 1.0, 0.0}, {0.0, -2.0, 0.0}, {0.0, 0.0, 3.0}};

// The same three points are published by every sensor, in its own frame.
const std::vector<std::array<float, 3>> sensor_points = {
  {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}};

// Per-sensor markers, so a point can be traced back to its source topic after concatenation
// has thrown the frame_id away.
const std::vector<uint8_t> sensor_intensities = {10, 20, 30};
const std::vector<uint8_t> sensor_return_types = {1, 2, 3};
const std::vector<uint16_t> sensor_channels = {100, 200, 300};

constexpr double timeout_sec = 0.2;
constexpr double noise_window = 0.01;
const std::vector<double> timestamp_offsets = {0.0, 0.04, 0.08};
constexpr double velocity_mps = 1.0;

// The concatenated cloud is always PointXYZIRC, whatever the input layout was.
constexpr uint32_t xyzirc_point_step = 16;

// Message stamps are arbitrary but must stay well clear of the wall clock, so that a stale
// transform lookup or a clock-based comparison cannot accidentally succeed.
constexpr double base_stamp_sec = 100.0;

enum class Layout { xyzircaedt, xyzirc, xyzi };

// ---------------------------------------------------------------------------------------
// Default Node parameters.
// ---------------------------------------------------------------------------------------

struct NodeParams
{
  std::string matching_strategy{"advanced"};
  bool is_motion_compensated{true};
  std::string input_twist_topic_type{"twist"};
  bool publish_synchronized_pointcloud{true};
  bool keep_input_frame_in_synchronized_pointcloud{true};
  bool publish_previous_but_late_pointcloud{true};
  std::string synchronized_pointcloud_postfix{"pointcloud_sync"};
  int maximum_queue_size{5};
  double rosbag_length{0.0};
  std::vector<double> lidar_timestamp_noise_windows{noise_window, noise_window, noise_window};
};

// ---------------------------------------------------------------------------------------
// Small value helpers
// ---------------------------------------------------------------------------------------

rclcpp::Time to_time(double seconds)
{
  const auto sec = static_cast<int32_t>(seconds);
  const auto nanosec = static_cast<uint32_t>(std::llround((seconds - sec) * 1e9));
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
}

struct Point
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  uint8_t intensity{0};
  uint8_t return_type{0};
  uint16_t channel{0};
};

std::vector<Point> read_points(const PointCloud2 & cloud)
{
  std::vector<Point> points;
  if (cloud.width * cloud.height == 0) return points;

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> it_i(cloud, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> it_r(cloud, "return_type");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> it_c(cloud, "channel");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
    points.push_back(Point{*it_x, *it_y, *it_z, *it_i, *it_r, *it_c});
  }
  return points;
}

// Where the points of one sensor end up in the output frame. The shift is along x only: the
// tests drive the node with a purely longitudinal twist (linear.x), so motion compensation
// never displaces a point in y or z.
std::vector<Point> expected_points_in_output_frame(size_t sensor_index, double motion_shift_x)
{
  const auto & translation = sensor_translations.at(sensor_index);
  std::vector<Point> points;
  for (const auto & point : sensor_points) {
    points.push_back(
      Point{
        static_cast<float>(point[0] + translation[0] + motion_shift_x),
        static_cast<float>(point[1] + translation[1]),
        static_cast<float>(point[2] + translation[2]), sensor_intensities.at(sensor_index),
        sensor_return_types.at(sensor_index), sensor_channels.at(sensor_index)});
  }
  return points;
}

// How far the ego vehicle travelled along x between this sensor's stamp and the oldest stamp
// of its group. correct_pointcloud_motion() accumulates linear.x * dt over every older stamp,
// so at a constant velocity the shift is exactly the sensor's timestamp offset.
double calculate_motion_shift_x(size_t sensor_index)
{
  return velocity_mps * timestamp_offsets.at(sensor_index);
}

::testing::AssertionResult positions_match(
  const std::vector<Point> & actual, const std::vector<Point> & expected)
{
  if (actual.size() != expected.size()) {
    return ::testing::AssertionFailure()
           << "expected " << expected.size() << " points, got " << actual.size();
  }
  for (size_t i = 0; i < actual.size(); ++i) {
    const auto & a = actual[i];
    const auto & e = expected[i];
    if (std::abs(a.x - e.x) > 1e-3F || std::abs(a.y - e.y) > 1e-3F || std::abs(a.z - e.z) > 1e-3F) {
      return ::testing::AssertionFailure()
             << "point " << i << " is (" << a.x << ", " << a.y << ", " << a.z << "), expected ("
             << e.x << ", " << e.y << ", " << e.z << ")";
    }
  }
  return ::testing::AssertionSuccess();
}

// Every point of a source should carry that source's marker values unchanged.
::testing::AssertionResult markers_match(const std::vector<Point> & actual, size_t sensor_index)
{
  for (size_t i = 0; i < actual.size(); ++i) {
    const auto & point = actual[i];
    if (
      point.intensity != sensor_intensities.at(sensor_index) ||
      point.return_type != sensor_return_types.at(sensor_index) ||
      point.channel != sensor_channels.at(sensor_index)) {
      return ::testing::AssertionFailure()
             << "point " << i << " carries intensity " << static_cast<int>(point.intensity)
             << ", return_type " << static_cast<int>(point.return_type) << ", channel "
             << point.channel << "; expected "
             << static_cast<int>(sensor_intensities.at(sensor_index)) << ", "
             << static_cast<int>(sensor_return_types.at(sensor_index)) << ", "
             << sensor_channels.at(sensor_index);
    }
  }
  return ::testing::AssertionSuccess();
}

// "name:offset:datatype:count" per field, so a layout can be compared in a single EXPECT_EQ.
std::vector<std::string> get_field_signature(const PointCloud2 & cloud)
{
  std::vector<std::string> signature;
  signature.reserve(cloud.fields.size());
  for (const auto & field : cloud.fields) {
    signature.push_back(
      field.name + ":" + std::to_string(field.offset) + ":" + std::to_string(field.datatype) + ":" +
      std::to_string(field.count));
  }
  return signature;
}

const std::vector<std::string> xyzirc_field_signature = {
  "x:0:7:1", "y:4:7:1", "z:8:7:1", "intensity:12:2:1", "return_type:13:2:1", "channel:14:4:1"};

// The points of one sensor as they come back on that sensor's synchronized topic: motion
// compensation is applied in the output frame and then undone by the transform back into the
// sensor frame, so only the compensation offset survives.
std::vector<Point> expected_points_in_sensor_frame(size_t sensor_index, double motion_shift_x)
{
  std::vector<Point> points;
  for (const auto & point : sensor_points) {
    points.push_back(
      Point{
        static_cast<float>(point[0] + motion_shift_x), point[1], point[2],
        sensor_intensities.at(sensor_index), sensor_return_types.at(sensor_index),
        sensor_channels.at(sensor_index)});
  }
  return points;
}

// The (idx_begin, length) pair of every source, sorted, for comparing against an expected
// partition of the concatenated cloud.
std::vector<std::pair<uint32_t, uint32_t>> get_sorted_source_segments(
  const ConcatenatedPointCloudInfo & info)
{
  std::vector<std::pair<uint32_t, uint32_t>> segments;
  segments.reserve(info.source_info.size());
  for (const auto & source : info.source_info) {
    segments.emplace_back(source.idx_begin, source.length);
  }
  std::sort(segments.begin(), segments.end());
  return segments;
}

// One point's position in the output frame, for pinning values that no simple formula
// describes: once the twist queue holds several entries the compensation integrates both
// position and yaw, so points are rotated as well as shifted.
struct Position
{
  float x;
  float y;
  float z;
};

std::vector<Point> as_expected_points(const std::vector<Position> & positions, size_t sensor_index)
{
  std::vector<Point> points;
  points.reserve(positions.size());
  for (const auto & position : positions) {
    points.push_back(
      Point{
        position.x, position.y, position.z, sensor_intensities.at(sensor_index),
        sensor_return_types.at(sensor_index), sensor_channels.at(sensor_index)});
  }
  return points;
}

// The keys check_concat_status() emits, in the order it emits them, for the advanced strategy.
std::vector<std::string> expected_diagnostic_keys(const std::vector<std::string> & input_topics)
{
  std::vector<std::string> keys = {
    "Concatenated pointcloud timestamp", "Minimum reference timestamp",
    "Maximum reference timestamp", "Processing time (ms)", "Pipeline latency (ms)"};
  for (const auto & topic : input_topics) {
    keys.push_back("Concatenated: " + topic);
    keys.push_back("Timestamp: " + topic);
    keys.push_back("Latency (ms): " + topic);
  }
  keys.push_back("Pointcloud concatenation succeeded");
  return keys;
}

// Mirrors format_timestamp() in the node: fixed notation, nine decimals.
std::string format_seconds(double seconds)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << seconds;
  return oss.str();
}

std::string get_diagnostic_value(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & entry : status.values) {
    if (entry.key == key) return entry.value;
  }
  return {};
}

bool has_diagnostic_key(const DiagnosticStatus & status, const std::string & key)
{
  return std::any_of(status.values.begin(), status.values.end(), [&key](const auto & entry) {
    return entry.key == key;
  });
}

std::vector<std::string> get_diagnostic_keys(const DiagnosticStatus & status)
{
  std::vector<std::string> keys;
  keys.reserve(status.values.size());
  for (const auto & entry : status.values) {
    keys.push_back(entry.key);
  }
  return keys;
}

// Everything a reader needs to know about one topic the node is supposed to offer. Tests
// state the expectation as a value and hand it to expect_node_has_publisher_for() or
// expect_node_has_subscription_for(), so the assertion reads as one sentence.
struct ExpectedTopic
{
  std::string name;
  std::string type;
  rclcpp::ReliabilityPolicy reliability;
  rclcpp::DurabilityPolicy durability;
  size_t depth;
};

// Point cloud endpoints use SensorDataQoS with the depth overridden by maximum_queue_size.
ExpectedTopic make_point_cloud_topic(const std::string & name, size_t depth)
{
  return ExpectedTopic{
    name, "sensor_msgs/msg/PointCloud2", rclcpp::ReliabilityPolicy::BestEffort,
    rclcpp::DurabilityPolicy::Volatile, depth};
}

// The velocity input uses a plain rclcpp::QoS{100} instead: reliable, and far deeper.
ExpectedTopic make_velocity_topic(const std::string & name, const std::string & type)
{
  return ExpectedTopic{
    name, type, rclcpp::ReliabilityPolicy::Reliable, rclcpp::DurabilityPolicy::Volatile, 100};
}

// Loads the component library once for the whole test binary; nullptr if the plugin is not
// registered. The manager and factory are leaked on purpose: class_loader unloads the library
// in its destructor, which runs after rclcpp::shutdown() and aborts the process.
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

// The source entry for `topic`, or nullptr if the info message does not carry it. A plain
// lookup: whether a missing topic is a failure is the caller's decision, not this one's.
const SourcePointCloudInfo * get_source_for(
  const ConcatenatedPointCloudInfo & info, const std::string & topic)
{
  const auto it = std::find_if(
    info.source_info.begin(), info.source_info.end(),
    [&topic](const auto & source) { return source.topic == topic; });
  return it == info.source_info.end() ? nullptr : &*it;
}

}  // namespace

// ---------------------------------------------------------------------------------------
// Test fixture: loads the node under test by plugin name, wires the topics, and offers one
// verb per thing a test needs to do. Every loop lives here so the tests stay flat.
// ---------------------------------------------------------------------------------------

class ConcatenateNodeTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    executor_.reset();
    // TODO(sasakisasaki): this drops one reference but does not destroy the node. The node
    // owns its CloudCollectors and each collector owns the node back, so the shared_ptr cycle
    // keeps both alive. Harmless here, but production wants ros2_parent_node_ to be weak.
    node_wrapper_ = rclcpp_components::NodeInstanceWrapper();
    test_node_.reset();
  }

  // Brings the node up with `params` and connects every publisher and subscriber this test
  // could need.
  void start(const NodeParams & params)
  {
    node_name_ = "concat_under_test";
    params_ = params;

    for (size_t i = 0; i < num_sensors; ++i) {
      input_topics_.push_back(in_namespace(get_input_topic_for(i)));
    }

    // Each test gets a private namespace named after itself, so nothing leaks
    // between tests and every topic in a failure message says which test owns it.
    test_node_ = std::make_shared<rclcpp::Node>("characterization_driver", test_namespace());
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    // Static TF is transient_local, so the node picks it up whenever it starts.
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
    tf_broadcaster_->sendTransform(make_static_transforms());

    create_input_publishers();
    create_output_subscriptions();

    load_node_under_test();
    spin_for(std::chrono::milliseconds(300));
  }

  // -- driving ------------------------------------------------------------------------

  void publish_twist(double stamp_sec, double linear_x = velocity_mps, double angular_z = 0.0)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = to_time(stamp_sec);
    msg.header.frame_id = output_frame;
    msg.twist.twist.linear.x = linear_x;
    msg.twist.twist.angular.z = angular_z;
    twist_publisher_->publish(msg);
    spin_for(std::chrono::milliseconds(100));
  }

  void publish_odometry(double stamp_sec)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = to_time(stamp_sec);
    msg.header.frame_id = output_frame;
    msg.twist.twist.linear.x = velocity_mps;
    odometry_publisher_->publish(msg);
    spin_for(std::chrono::milliseconds(100));
  }

  void publish_cloud(
    size_t sensor_index, double stamp_sec, Layout layout = Layout::xyzircaedt, bool empty = false)
  {
    input_publishers_.at(sensor_index)->publish(make_cloud(sensor_index, stamp_sec, layout, empty));
    spin_for(std::chrono::milliseconds(10));
  }

  // Publishes one cloud per sensor at base_sec + its configured offset, and returns the
  // stamps used, in sensor order.
  std::vector<double> publish_all_clouds(
    double base_sec, Layout layout = Layout::xyzircaedt, bool empty = false)
  {
    std::vector<double> stamps;
    for (size_t i = 0; i < num_sensors; ++i) {
      stamps.push_back(base_sec + timestamp_offsets.at(i));
    }
    for (size_t i = 0; i < num_sensors; ++i) {
      publish_cloud(i, stamps.at(i), layout, empty);
    }
    return stamps;
  }

  // Publishes only the sensors listed, so the rest have to time out.
  std::vector<double> publish_clouds_from(double base_sec, const std::vector<size_t> & sensors)
  {
    std::vector<double> stamps;
    for (size_t i = 0; i < num_sensors; ++i) {
      stamps.push_back(base_sec + timestamp_offsets.at(i));
    }
    for (const auto sensor : sensors) {
      publish_cloud(sensor, stamps.at(sensor));
    }
    return stamps;
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

  template <typename T>
  bool wait_until(const T & predicate, std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) return true;
      executor_->spin_some(std::chrono::milliseconds(5));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
  }

  // Waits for the concatenated cloud, then keeps spinning briefly so that a second,
  // unexpected publication would also be recorded.
  PointCloud2 await_concatenated_cloud(std::chrono::nanoseconds timeout = std::chrono::seconds(2))
  {
    EXPECT_TRUE(wait_until([this] { return !concatenated_clouds_.empty(); }, timeout))
      << "no concatenated cloud was published";
    spin_for(std::chrono::milliseconds(250));
    return concatenated_clouds_.empty() ? PointCloud2{} : concatenated_clouds_.front();
  }

  // Waits for `count` concatenated clouds, then keeps spinning briefly so that an
  // unexpected extra publication is recorded too.
  std::vector<PointCloud2> await_concatenated_clouds(
    size_t count, std::chrono::nanoseconds timeout = std::chrono::seconds(3))
  {
    EXPECT_TRUE(wait_until([this, count] { return concatenated_clouds_.size() >= count; }, timeout))
      << "expected " << count << " concatenated clouds, got " << concatenated_clouds_.size();
    spin_for(std::chrono::milliseconds(300));
    return concatenated_clouds_;
  }

  ConcatenatedPointCloudInfo await_concatenation_info(
    std::chrono::nanoseconds timeout = std::chrono::seconds(2))
  {
    EXPECT_TRUE(wait_until([this] { return !concatenation_infos_.empty(); }, timeout))
      << "no concatenation info was published";
    spin_for(std::chrono::milliseconds(250));
    return concatenation_infos_.empty() ? ConcatenatedPointCloudInfo{}
                                        : concatenation_infos_.front();
  }

  PointCloud2 await_synchronized_cloud(
    size_t sensor_index, std::chrono::nanoseconds timeout = std::chrono::seconds(2))
  {
    auto & buffer = synchronized_clouds_.at(sensor_index);
    EXPECT_TRUE(wait_until([&buffer] { return !buffer.empty(); }, timeout))
      << "no synchronized cloud on " << synchronized_topics_.at(sensor_index);
    return buffer.empty() ? PointCloud2{} : buffer.front();
  }

  DiagnosticStatus await_diagnostic(std::chrono::nanoseconds timeout = std::chrono::seconds(2))
  {
    EXPECT_TRUE(wait_until([this] { return !diagnostics_.empty(); }, timeout))
      << "no diagnostics from " << node_name_;
    spin_for(std::chrono::milliseconds(250));
    return diagnostics_.empty() ? DiagnosticStatus{} : diagnostics_.back();
  }

  // Gives the node time to do nothing, for the tests that assert on silence.
  void wait_out_the_timeout()
  {
    spin_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_sec * 4)));
  }

  // The slice of the concatenated cloud that `info` attributes to `sensor_index`.
  std::vector<Point> get_segment_of(
    const ConcatenatedPointCloudInfo & info, const PointCloud2 & cloud, size_t sensor_index)
  {
    const auto & topic = input_topics_.at(sensor_index);
    const auto * source = get_source_for(info, topic);
    if (source == nullptr) {
      ADD_FAILURE() << topic << " is missing from source_info";
      return {};
    }
    const auto points = read_points(cloud);
    if (source->idx_begin + source->length > points.size()) {
      ADD_FAILURE() << topic << " claims points [" << source->idx_begin << ", "
                    << source->idx_begin + source->length << ") of a cloud holding only "
                    << points.size();
      return {};
    }
    return std::vector<Point>(
      points.begin() + source->idx_begin, points.begin() + source->idx_begin + source->length);
  }

  // -- topic graph introspection ---------------------------------------------------------

  void expect_node_has_publisher_for(const ExpectedTopic & expected)
  {
    SCOPED_TRACE("publisher on " + expected.name);
    expect_single_endpoint_matches(get_publishers_on(expected.name), expected);
  }

  void expect_node_has_no_publisher_for(const std::string & topic)
  {
    EXPECT_TRUE(get_publishers_on(topic).empty()) << topic << " should not be advertised";
  }

  void expect_node_has_subscription_for(const ExpectedTopic & expected)
  {
    SCOPED_TRACE("subscription on " + expected.name);
    expect_single_endpoint_matches(get_subscriptions_on(expected.name), expected);
  }

  void expect_node_has_no_subscription_for(const std::string & topic)
  {
    EXPECT_TRUE(get_subscriptions_on(topic).empty()) << topic << " should not be subscribed";
  }

  ExpectedTopic expected_input_topic(size_t sensor_index) const
  {
    return make_point_cloud_topic(
      get_input_topic_for(sensor_index), static_cast<size_t>(params_.maximum_queue_size));
  }

  std::vector<rclcpp::TopicEndpointInfo> get_publishers_on(const std::string & topic)
  {
    return get_endpoints_of_node_under_test(
      test_node_->get_publishers_info_by_topic(in_namespace(topic)));
  }

  std::vector<rclcpp::TopicEndpointInfo> get_subscriptions_on(const std::string & topic)
  {
    return get_endpoints_of_node_under_test(
      test_node_->get_subscriptions_info_by_topic(in_namespace(topic)));
  }

  // -- topic names ------------------------------------------------------------------------
  //
  // Topics are relative to the test's private namespace ("/output"); in_namespace() expands
  // them, and the assertions above apply it for you. Naming the namespace after the test is
  // what keeps a finished test's lingering endpoints out of the node-under-test lookups.
  std::string test_namespace() const
  {
    return "/" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());
  }

  std::string in_namespace(const std::string & relative_topic) const
  {
    return test_namespace() + relative_topic;
  }

  std::string get_input_topic_for(size_t sensor_index) const
  {
    return "/lidar/" + sensor_names.at(sensor_index) + "/pointcloud";
  }

  // Mirrors what the node does to build a synchronized topic name, so the fixture can
  // subscribe. The naming rule itself is asserted separately by the topic interface tests.
  std::string get_synchronized_topic_for(size_t sensor_index) const
  {
    const auto topic = get_input_topic_for(sensor_index);
    const auto slash = topic.find_last_of('/');
    const auto replaced = topic.substr(0, slash) + "/" + params_.synchronized_pointcloud_postfix;
    return replaced == topic ? topic + "_synchronized" : replaced;
  }

  std::string get_twist_topic() const { return "/" + node_name_ + "/input/twist"; }
  std::string get_odometry_topic() const { return "/" + node_name_ + "/input/odom"; }
  std::string get_debug_topic(const std::string & leaf) const
  {
    return "/concatenate_data_synchronizer/debug/" + leaf;
  }

  std::vector<std::string> input_topics_;
  std::vector<std::string> synchronized_topics_;
  std::vector<PointCloud2> concatenated_clouds_;
  std::vector<ConcatenatedPointCloudInfo> concatenation_infos_;
  std::array<std::vector<PointCloud2>, num_sensors> synchronized_clouds_;
  std::vector<DiagnosticStatus> diagnostics_;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> processing_times_;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> cyclic_times_;
  std::array<std::vector<autoware_internal_debug_msgs::msg::Float64Stamped>, num_sensors>
    pipeline_latencies_;
  std::string node_name_;
  NodeParams params_;

private:
  static void expect_single_endpoint_matches(
    const std::vector<rclcpp::TopicEndpointInfo> & endpoints, const ExpectedTopic & expected)
  {
    ASSERT_EQ(endpoints.size(), 1u) << "expected exactly one endpoint on " << expected.name;
    const auto & endpoint = endpoints.front();
    EXPECT_EQ(endpoint.topic_type(), expected.type);
    EXPECT_EQ(endpoint.qos_profile().reliability(), expected.reliability);
    EXPECT_EQ(endpoint.qos_profile().durability(), expected.durability);
    EXPECT_EQ(endpoint.qos_profile().depth(), expected.depth);
  }

  std::vector<rclcpp::TopicEndpointInfo> get_endpoints_of_node_under_test(
    const std::vector<rclcpp::TopicEndpointInfo> & endpoints) const
  {
    std::vector<rclcpp::TopicEndpointInfo> matching;
    std::copy_if(
      endpoints.begin(), endpoints.end(), std::back_inserter(matching),
      [this](const auto & endpoint) { return endpoint.node_name() == node_name_; });
    return matching;
  }

  std::vector<geometry_msgs::msg::TransformStamped> make_static_transforms() const
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (size_t i = 0; i < num_sensors; ++i) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = to_time(0.0);
      tf.header.frame_id = output_frame;
      tf.child_frame_id = sensor_frames.at(i);
      tf.transform.translation.x = sensor_translations.at(i)[0];
      tf.transform.translation.y = sensor_translations.at(i)[1];
      tf.transform.translation.z = sensor_translations.at(i)[2];
      tf.transform.rotation.w = 1.0;
      transforms.push_back(tf);
    }
    return transforms;
  }

  void create_input_publishers()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    for (const auto & topic : input_topics_) {
      input_publishers_.push_back(test_node_->create_publisher<PointCloud2>(topic, sensor_qos));
    }
    twist_publisher_ = test_node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      in_namespace(get_twist_topic()), 10);
    odometry_publisher_ =
      test_node_->create_publisher<nav_msgs::msg::Odometry>(in_namespace(get_odometry_topic()), 10);
  }

  void create_output_subscriptions()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);

    concatenated_subscription_ = test_node_->create_subscription<PointCloud2>(
      in_namespace("/output"), sensor_qos,
      [this](PointCloud2::ConstSharedPtr msg) { concatenated_clouds_.push_back(*msg); });

    info_subscription_ = test_node_->create_subscription<ConcatenatedPointCloudInfo>(
      in_namespace("/output_info"), sensor_qos,
      [this](ConcatenatedPointCloudInfo::ConstSharedPtr msg) {
        concatenation_infos_.push_back(*msg);
      });

    for (size_t i = 0; i < num_sensors; ++i) {
      synchronized_topics_.push_back(in_namespace(get_synchronized_topic_for(i)));
      synchronized_subscriptions_.push_back(test_node_->create_subscription<PointCloud2>(
        synchronized_topics_.back(), sensor_qos, [this, i](PointCloud2::ConstSharedPtr msg) {
          synchronized_clouds_.at(i).push_back(*msg);
        }));
    }

    diagnostics_subscription_ = test_node_->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(100), [this](DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.hardware_id == node_name_) diagnostics_.push_back(status);
        }
      });

    using autoware_internal_debug_msgs::msg::Float64Stamped;
    processing_time_subscription_ = test_node_->create_subscription<Float64Stamped>(
      in_namespace(get_debug_topic("processing_time_ms")), rclcpp::QoS(10),
      [this](Float64Stamped::ConstSharedPtr msg) { processing_times_.push_back(*msg); });
    cyclic_time_subscription_ = test_node_->create_subscription<Float64Stamped>(
      in_namespace(get_debug_topic("cyclic_time_ms")), rclcpp::QoS(10),
      [this](Float64Stamped::ConstSharedPtr msg) { cyclic_times_.push_back(*msg); });

    for (size_t i = 0; i < num_sensors; ++i) {
      latency_subscriptions_.push_back(test_node_->create_subscription<Float64Stamped>(
        in_namespace(
          "/concatenate_data_synchronizer/debug" + input_topics_.at(i) + "/pipeline_latency_ms"),
        rclcpp::QoS(10), [this, i](Float64Stamped::ConstSharedPtr msg) {
          pipeline_latencies_.at(i).push_back(*msg);
        }));
    }
  }

  void load_node_under_test()
  {
    const auto factory = get_component_factory();
    ASSERT_NE(factory, nullptr) << "component " << plugin_name << " is not registered in package "
                                << package_name;

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "-r", "__ns:=" + test_namespace(), "-r", "__node:=" + node_name_});
    options.parameter_overrides(make_parameter_overrides());

    node_wrapper_ = factory->create_node_instance(options);
    executor_->add_node(node_wrapper_.get_node_base_interface());
  }

  std::vector<rclcpp::Parameter> make_parameter_overrides() const
  {
    std::vector<rclcpp::Parameter> overrides{
      {"debug_mode", false},
      {"rosbag_length", params_.rosbag_length},
      {"maximum_queue_size", params_.maximum_queue_size},
      {"timeout_sec", timeout_sec},
      {"is_motion_compensated", params_.is_motion_compensated},
      {"publish_synchronized_pointcloud", params_.publish_synchronized_pointcloud},
      {"keep_input_frame_in_synchronized_pointcloud",
       params_.keep_input_frame_in_synchronized_pointcloud},
      {"publish_previous_but_late_pointcloud", params_.publish_previous_but_late_pointcloud},
      {"synchronized_pointcloud_postfix", params_.synchronized_pointcloud_postfix},
      {"input_twist_topic_type", params_.input_twist_topic_type},
      {"input_topics", input_topics_},
      {"output_frame", std::string(output_frame)},
      {"matching_strategy.type", params_.matching_strategy}};

    if (params_.matching_strategy == "advanced") {
      overrides.emplace_back("matching_strategy.lidar_timestamp_offsets", timestamp_offsets);
      overrides.emplace_back(
        "matching_strategy.lidar_timestamp_noise_window", params_.lidar_timestamp_noise_windows);
    }
    return overrides;
  }

  static PointCloud2 make_cloud(size_t sensor_index, double stamp_sec, Layout layout, bool empty);

  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp_components::NodeInstanceWrapper node_wrapper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  std::vector<rclcpp::Publisher<PointCloud2>::SharedPtr> input_publishers_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

  rclcpp::Subscription<PointCloud2>::SharedPtr concatenated_subscription_;
  rclcpp::Subscription<ConcatenatedPointCloudInfo>::SharedPtr info_subscription_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> synchronized_subscriptions_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    processing_time_subscription_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    cyclic_time_subscription_;
  std::vector<rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr>
    latency_subscriptions_;
};

PointCloud2 ConcatenateNodeTest::make_cloud(
  size_t sensor_index, double stamp_sec, Layout layout, bool empty)
{
  PointCloud2 cloud;
  cloud.header.stamp = to_time(stamp_sec);
  cloud.header.frame_id = sensor_frames.at(sensor_index);
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  auto add_field = [&cloud](const char * name, uint32_t offset, uint8_t datatype) {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(field);
  };

  if (layout == Layout::xyzi) {
    // Rejected by the node: only four fields, and intensity is FLOAT32 rather than UINT8.
    add_field("x", 0, PointField::FLOAT32);
    add_field("y", 4, PointField::FLOAT32);
    add_field("z", 8, PointField::FLOAT32);
    add_field("intensity", 12, PointField::FLOAT32);
    cloud.point_step = 16;
  } else {
    add_field("x", 0, PointField::FLOAT32);
    add_field("y", 4, PointField::FLOAT32);
    add_field("z", 8, PointField::FLOAT32);
    add_field("intensity", 12, PointField::UINT8);
    add_field("return_type", 13, PointField::UINT8);
    add_field("channel", 14, PointField::UINT16);
    cloud.point_step = 16;
    if (layout == Layout::xyzircaedt) {
      add_field("azimuth", 16, PointField::FLOAT32);
      add_field("elevation", 20, PointField::FLOAT32);
      add_field("distance", 24, PointField::FLOAT32);
      add_field("time_stamp", 28, PointField::UINT32);
      cloud.point_step = 32;
    }
  }

  cloud.width = empty ? 0 : num_points;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step, 0);

  for (size_t i = 0; i < cloud.width; ++i) {
    auto * base = cloud.data.data() + i * cloud.point_step;
    const auto & point = sensor_points.at(i);
    std::memcpy(base + 0, &point[0], sizeof(float));
    std::memcpy(base + 4, &point[1], sizeof(float));
    std::memcpy(base + 8, &point[2], sizeof(float));
    if (layout == Layout::xyzi) {
      const float intensity = 1.0F;
      std::memcpy(base + 12, &intensity, sizeof(float));
      continue;
    }
    base[12] = sensor_intensities.at(sensor_index);
    base[13] = sensor_return_types.at(sensor_index);
    const uint16_t channel = sensor_channels.at(sensor_index);
    std::memcpy(base + 14, &channel, sizeof(uint16_t));
  }
  return cloud;
}

// =======================================================================================
//
//                          T H E   T E S T S   S T A R T   H E R E
//
// Everything above is harness: constants, value helpers, and the fixture. None of it
// asserts anything about the node.
//
// Sections below, in order:
//   Published topic interface   - what the node advertises, and with which QoS
//   Subscribed topic interface  - what it listens to, and which velocity input it picks
//   The concatenated cloud      - header, layout, payload, motion compensation
//   ConcatenatedPointCloudInfo  - the metadata published alongside each concatenated cloud
//   Synchronized clouds         - the per-source clouds, in both frame modes
//   Collector matching          - which collector a cloud joins, and what happens when none fit
//   A source that never arrives - the timeout path
//   Empty inputs                - sources that arrive carrying no points
//   Naive matching              - grouping by arrival time instead of by stamp
//   Late clouds                 - dropping, and the rosbag-loop exception
//   Odometry                    - the other velocity input
//   Diagnostics                 - level, message, and key/value payload
//   Debug topics                - processing time, cyclic time, per-input latency
//
// Each test builds its own node, so any one of them can be run on its own:
//   ./test_concatenate_node_characterization --gtest_filter='*DropsCloudOlder*'
//
// =======================================================================================

// ---------------------------------------------------------------------------------------
// Published topic interface. Covers initialize_pub_sub() and
// replace_sync_topic_name_postfix().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, AdvertisesConcatenatedCloudTopic)
{
  // Arrange
  start(NodeParams{});

  // Assert
  expect_node_has_publisher_for(make_point_cloud_topic("/output", 5));
}

TEST_F(ConcatenateNodeTest, AdvertisesConcatenationInfoTopic)
{
  // Arrange
  start(NodeParams{});

  // Assert
  expect_node_has_publisher_for(
    ExpectedTopic{
      "/output_info", "autoware_sensing_msgs/msg/ConcatenatedPointCloudInfo",
      rclcpp::ReliabilityPolicy::BestEffort, rclcpp::DurabilityPolicy::Volatile, 5});
}

TEST_F(ConcatenateNodeTest, AdvertisesOneSynchronizedCloudTopicPerInput)
{
  // Arrange
  NodeParams params;
  params.publish_synchronized_pointcloud = true;
  start(params);

  // Assert
  const std::vector<std::string> expected_names = {
    "/lidar/left/pointcloud_sync", "/lidar/right/pointcloud_sync", "/lidar/top/pointcloud_sync"};
  for (const auto & name : expected_names) {
    expect_node_has_publisher_for(make_point_cloud_topic(name, 5));
  }
}

TEST_F(ConcatenateNodeTest, AdvertisesNoSynchronizedCloudTopicWhenDisabled)
{
  // Arrange
  NodeParams params;
  params.publish_synchronized_pointcloud = false;
  start(params);

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    expect_node_has_no_publisher_for(get_synchronized_topic_for(i));
  }
}

TEST_F(ConcatenateNodeTest, SynchronizedTopicNameFallsBackWhenPostfixMatchesInputName)
{
  // Arrange
  NodeParams params;
  params.synchronized_pointcloud_postfix = "pointcloud";
  start(params);

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    expect_node_has_publisher_for(
      make_point_cloud_topic(get_input_topic_for(i) + "_synchronized", 5));
  }
}

TEST_F(ConcatenateNodeTest, PublisherQueueDepthFollowsMaximumQueueSize)
{
  // Arrange
  NodeParams params;
  params.maximum_queue_size = 3;
  start(params);

  // Assert
  expect_node_has_publisher_for(make_point_cloud_topic("/output", 3));
  expect_node_has_subscription_for(expected_input_topic(0));
}

// ---------------------------------------------------------------------------------------
// Subscribed topic interface. Covers initialize_pub_sub() and the twist/odom branch of the
// constructor.
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, SubscribesToEveryConfiguredInputTopic)
{
  // Arrange
  start(NodeParams{});

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    expect_node_has_subscription_for(expected_input_topic(i));
  }
}

TEST_F(ConcatenateNodeTest, SubscribesToTwistWhenTwistTopicTypeSelected)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = true;
  params.input_twist_topic_type = "twist";
  start(params);

  // Assert
  expect_node_has_subscription_for(
    make_velocity_topic(get_twist_topic(), "geometry_msgs/msg/TwistWithCovarianceStamped"));
  expect_node_has_no_subscription_for(get_odometry_topic());
}

TEST_F(ConcatenateNodeTest, SubscribesToOdometryWhenOdomTopicTypeSelected)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = true;
  params.input_twist_topic_type = "odom";
  start(params);

  // Assert
  expect_node_has_subscription_for(
    make_velocity_topic(get_odometry_topic(), "nav_msgs/msg/Odometry"));
  expect_node_has_no_subscription_for(get_twist_topic());
}

TEST_F(ConcatenateNodeTest, SubscribesToNoVelocityTopicWhenMotionCompensationDisabled)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  start(params);

  // Assert
  expect_node_has_no_subscription_for(get_twist_topic());
  expect_node_has_no_subscription_for(get_odometry_topic());
}

// ---------------------------------------------------------------------------------------
// The concatenated cloud. Covers the whole chain, cloud_callback() through publish_clouds().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, ConcatenatedCloudCarriesOldestInputStamp)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();

  // Assert
  EXPECT_EQ(
    rclcpp::Time(cloud.header.stamp), to_time(*std::min_element(stamps.begin(), stamps.end())));
}

TEST_F(ConcatenateNodeTest, ConcatenatedCloudIsExpressedInOutputFrame)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);

  // Assert
  EXPECT_EQ(await_concatenated_cloud().header.frame_id, output_frame);
}

TEST_F(ConcatenateNodeTest, ConcatenatedCloudUsesUnorganizedXyzircLayout)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();

  // Assert
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.width, num_sensors * num_points);
  EXPECT_EQ(cloud.point_step, xyzirc_point_step);
  EXPECT_EQ(cloud.row_step, xyzirc_point_step * cloud.width);
  EXPECT_TRUE(cloud.is_dense);
  EXPECT_FALSE(cloud.is_bigendian);
  EXPECT_EQ(get_field_signature(cloud), xyzirc_field_signature);
}

TEST_F(ConcatenateNodeTest, PublishesConcatenatedCloudOnlyOnceWhenAllInputsArrive)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();
  wait_out_the_timeout();

  // Assert
  EXPECT_EQ(concatenated_clouds_.size(), 1u);
}

TEST_F(ConcatenateNodeTest, EachSourceIsTransformedIntoTheOutputFrame)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  start(params);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(
      positions_match(get_segment_of(info, cloud, i), expected_points_in_output_frame(i, 0.0)))
      << "sensor " << i;
  }
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(stamps.front()));
}

TEST_F(ConcatenateNodeTest, ConcatenatedCloudPreservesIntensityReturnTypeAndChannel)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(markers_match(get_segment_of(info, cloud, i), i)) << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, AcceptsPlainXyzircInputAsWellAsXyzircaedt)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec, Layout::xyzirc);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(cloud.width, num_sensors * num_points);
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i),
      expected_points_in_output_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, RejectsCloudWhoseLayoutIsNotXyzircCompatible)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec, Layout::xyzi);
  // cloud_callback() returns before process_pointcloud(), so no collector ever starts and
  // no timeout fires either. Nothing at all is published.
  wait_out_the_timeout();

  // Assert
  EXPECT_TRUE(concatenated_clouds_.empty());
  EXPECT_TRUE(concatenation_infos_.empty());
  EXPECT_TRUE(diagnostics_.empty());
}

// ---------------------------------------------------------------------------------------
// Motion compensation. Covers correct_pointcloud_motion(),
// compute_transform_to_adjust_for_old_timestamp() and the twist queue in process_twist().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, MotionCompensationShiftsEachSourceByItsTimestampOffset)
{
  // Arrange
  start(NodeParams{});
  // A single twist, constant velocity, no rotation: the simplest case.
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i),
      expected_points_in_output_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, MotionCompensationIntegratesEveryTwistInTheWindow)
{
  // Arrange
  start(NodeParams{});
  // Four twists across the 80 ms the clouds span, so several land between one pair of cloud
  // stamps: dt has to be chained from one entry to the next, and yaw integrated.
  publish_twist(base_stamp_sec + 0.00, 1.0, 0.0);
  publish_twist(base_stamp_sec + 0.02, 2.0, 0.5);
  publish_twist(base_stamp_sec + 0.05, 3.0, -0.25);
  publish_twist(base_stamp_sec + 0.08, 4.0, 1.0);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  // Recorded from a run, not derived. y moves although every twist is longitudinal, and the
  // x shift varies between points of one sensor: both are the integrated yaw rotating them.
  const std::vector<std::vector<Position>> golden_positions = {
    {{1.000000F, 1.000000F, 0.000000F},
     {0.000000F, 2.000000F, 0.000000F},
     {0.000000F, 1.000000F, 1.000000F}},
    {{1.109985F, -1.994275F, 0.000000F},
     {0.104997F, -0.999287F, 0.000000F},
     {0.109997F, -1.999275F, 1.000000F}},
    {{1.249406F, 0.037169F, 3.000000F},
     {0.217440F, 1.004146F, 3.000000F},
     {0.249934F, 0.004674F, 4.000000F}}};
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i), as_expected_points(golden_positions.at(i), i)))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, MotionCompensationStopsAtATwistGapLongerThanTheLimit)
{
  // Arrange
  // Naive matching lets cloud stamps sit far apart. One hop is under the 0.1 s limit and one
  // is over it, so a twist that never arrived cannot make this test pass.
  NodeParams params;
  params.matching_strategy = "naive";
  start(params);
  publish_twist(base_stamp_sec, 10.0, 0.0);

  // Act
  const std::vector<double> stamps = {base_stamp_sec, base_stamp_sec + 0.05, base_stamp_sec + 0.60};
  for (size_t i = 0; i < num_sensors; ++i) {
    publish_cloud(i, stamps.at(i));
  }
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  // The 0.05 s hop integrates to 0.5 m; the 0.55 s hop is refused, so the last cloud stays
  // at 0.5 m rather than gaining another 5.5 m.
  const std::vector<double> expected_shifts = {0.0, 0.5, 0.5};
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i), expected_points_in_output_frame(i, expected_shifts.at(i))))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, MotionCompensationIsSkippedWhenNoTwistHasArrived)
{
  // Arrange
  // Motion compensation is enabled, but nothing is ever published on ~/input/twist.
  start(NodeParams{});

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(
      positions_match(get_segment_of(info, cloud, i), expected_points_in_output_frame(i, 0.0)))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, TwistQueueIsClearedWhenTwistTimeJumpsBackwards)
{
  // Arrange
  start(NodeParams{});
  // A twist after the cloud window, then one inside it: process_twist() sees time go back.
  publish_twist(base_stamp_sec + 0.5, 10.0, 0.0);
  publish_twist(base_stamp_sec, velocity_mps, 0.0);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  // Only the surviving 1 m/s twist is integrated. Keeping the 10 m/s entry would have
  // selected it instead, making every shift ten times larger.
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i),
      expected_points_in_output_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

// ---------------------------------------------------------------------------------------
// Collector matching. Covers AdvancedMatchingPolicy::match() and the collector selection
// that cloud_callback() does around it.
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, CloudOutsideTheReferenceWindowOpensASecondCollector)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  start(params);

  // Act
  // The first two clouds arrive at their configured offsets and share a collector. The third
  // is half a second late, so its reference time misses that window and starts its own.
  publish_cloud(0, base_stamp_sec);
  publish_cloud(1, base_stamp_sec + 0.04);
  publish_cloud(2, base_stamp_sec + 0.08 + 0.5);
  const auto clouds = await_concatenated_clouds(2);

  // Assert
  // Two collectors, each timing out on its own, so two concatenated clouds come out.
  ASSERT_EQ(clouds.size(), 2u);
  ASSERT_EQ(concatenation_infos_.size(), 2u);

  EXPECT_EQ(clouds.at(0).width, 2 * num_points);
  EXPECT_EQ(rclcpp::Time(clouds.at(0).header.stamp), to_time(base_stamp_sec));
  EXPECT_EQ(concatenation_infos_.at(0).source_info.at(0).status, SourcePointCloudInfo::STATUS_OK);
  EXPECT_EQ(concatenation_infos_.at(0).source_info.at(1).status, SourcePointCloudInfo::STATUS_OK);
  EXPECT_EQ(
    concatenation_infos_.at(0).source_info.at(2).status, SourcePointCloudInfo::STATUS_TIMEOUT);

  EXPECT_EQ(clouds.at(1).width, num_points);
  EXPECT_EQ(rclcpp::Time(clouds.at(1).header.stamp), to_time(base_stamp_sec + 0.58));
  EXPECT_EQ(
    concatenation_infos_.at(1).source_info.at(0).status, SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(
    concatenation_infos_.at(1).source_info.at(1).status, SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(concatenation_infos_.at(1).source_info.at(2).status, SourcePointCloudInfo::STATUS_OK);
}

TEST_F(ConcatenateNodeTest, MatchingUsesTheNoiseWindowOfTheArrivingTopic)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  // A wide window for the middle lidar, narrow ones either side. With all three equal, a
  // swap of the array would be invisible.
  params.lidar_timestamp_noise_windows = {0.001, 0.05, 0.001};
  start(params);

  // Act
  // Both later clouds sit 30 ms off their configured offset. The middle lidar's own window
  // absorbs that; the last lidar's does not.
  publish_cloud(0, base_stamp_sec);
  publish_cloud(1, base_stamp_sec + 0.04 + 0.03);
  publish_cloud(2, base_stamp_sec + 0.08 + 0.03);
  const auto clouds = await_concatenated_clouds(2);

  // Assert
  ASSERT_EQ(clouds.size(), 2u);
  EXPECT_EQ(clouds.at(0).width, 2 * num_points) << "lidars 0 and 1 should share a collector";
  EXPECT_EQ(clouds.at(1).width, num_points) << "lidar 2 should be alone";
}

TEST_F(ConcatenateNodeTest, SameTopicArrivingTwiceReplacesTheEarlierCloud)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  start(params);

  // Act
  publish_cloud(0, base_stamp_sec);
  publish_cloud(0, base_stamp_sec + 0.005);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  // The second cloud is inside the collector's window, so it takes the first one's slot
  // instead of being added beside it: one cloud's worth of points, carrying the later stamp.
  EXPECT_EQ(cloud.width, num_points);
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(base_stamp_sec + 0.005));
  EXPECT_EQ(info.source_info.at(0).status, SourcePointCloudInfo::STATUS_OK);
  EXPECT_EQ(rclcpp::Time(info.source_info.at(0).header.stamp), to_time(base_stamp_sec + 0.005));
  EXPECT_EQ(info.source_info.at(1).status, SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(info.source_info.at(2).status, SourcePointCloudInfo::STATUS_TIMEOUT);
}

TEST_F(ConcatenateNodeTest, FourthConcurrentGroupDisplacesTheOldestCollector)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  start(params);

  // Act
  // Four reference times a second apart, so none of them share a collector, against the
  // three collectors the node keeps.
  for (int group = 0; group < 4; ++group) {
    publish_cloud(0, base_stamp_sec + group);
  }
  const auto clouds = await_concatenated_clouds(3);

  // Assert
  // The oldest group is reset to make room for the fourth, and is
  // dropped without ever being published. Four groups in, three clouds out.
  ASSERT_EQ(clouds.size(), 3u);
  EXPECT_EQ(rclcpp::Time(clouds.at(0).header.stamp), to_time(base_stamp_sec + 1));
  EXPECT_EQ(rclcpp::Time(clouds.at(1).header.stamp), to_time(base_stamp_sec + 2));
  EXPECT_EQ(rclcpp::Time(clouds.at(2).header.stamp), to_time(base_stamp_sec + 3));
  wait_out_the_timeout();
  EXPECT_EQ(concatenated_clouds_.size(), 3u) << "the displaced group must not appear later";
}

// ---------------------------------------------------------------------------------------
// ConcatenatedPointCloudInfo. Covers ConcatenationInfoManager, driven by combine_pointclouds().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, InfoHeaderMirrorsConcatenatedCloudHeader)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(rclcpp::Time(info.header.stamp), rclcpp::Time(cloud.header.stamp));
  EXPECT_EQ(info.header.frame_id, cloud.header.frame_id);
}

TEST_F(ConcatenateNodeTest, InfoReportsSuccessWhenEverySourceArrives)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);

  // Assert
  EXPECT_TRUE(await_concatenation_info().concatenation_success);
}

TEST_F(ConcatenateNodeTest, InfoListsSourcesInInputTopicsOrder)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto info = await_concatenation_info();

  // Assert
  ASSERT_EQ(info.source_info.size(), num_sensors);
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_EQ(info.source_info.at(i).topic, input_topics_.at(i));
    EXPECT_EQ(info.source_info.at(i).status, SourcePointCloudInfo::STATUS_OK);
  }
}

TEST_F(ConcatenateNodeTest, InfoSegmentsPartitionTheConcatenatedCloud)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto info = await_concatenation_info();

  // Assert
  const std::vector<std::pair<uint32_t, uint32_t>> expected_segments = {
    {0, num_points}, {num_points, num_points}, {2 * num_points, num_points}};
  EXPECT_EQ(get_sorted_source_segments(info), expected_segments);
}

TEST_F(ConcatenateNodeTest, InfoSourceHeaderKeepsOriginalStampButOutputFrame)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec);
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    const auto & source = info.source_info.at(i);
    EXPECT_EQ(rclcpp::Time(source.header.stamp), to_time(stamps.at(i)));
    EXPECT_EQ(source.header.frame_id, output_frame);
  }
}

TEST_F(ConcatenateNodeTest, InfoReportsAdvancedStrategyWithItsReferenceWindow)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "advanced";
  start(params);
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(info.matching_strategy, ConcatenatedPointCloudInfo::STRATEGY_ADVANCED);
  // set_config() stores the raw memcpy of two builtin_interfaces/Time values, i.e. the
  // window [timestamp - noise_window, timestamp + noise_window].
  ASSERT_EQ(info.matching_strategy_config.size(), 16u);
  int32_t min_sec = 0;
  uint32_t min_nsec = 0;
  int32_t max_sec = 0;
  uint32_t max_nsec = 0;
  std::memcpy(&min_sec, info.matching_strategy_config.data() + 0, 4);
  std::memcpy(&min_nsec, info.matching_strategy_config.data() + 4, 4);
  std::memcpy(&max_sec, info.matching_strategy_config.data() + 8, 4);
  std::memcpy(&max_nsec, info.matching_strategy_config.data() + 12, 4);
  EXPECT_NEAR(min_sec + min_nsec * 1e-9, base_stamp_sec - noise_window, 1e-6);
  EXPECT_NEAR(max_sec + max_nsec * 1e-9, base_stamp_sec + noise_window, 1e-6);
}

TEST_F(ConcatenateNodeTest, InfoReportsNaiveStrategyWithoutAnyConfig)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "naive";
  params.is_motion_compensated = false;
  start(params);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(info.matching_strategy, ConcatenatedPointCloudInfo::STRATEGY_NAIVE);
  // set_config() is only called for the advanced strategy, so this stays empty.
  EXPECT_TRUE(info.matching_strategy_config.empty());
}

// ---------------------------------------------------------------------------------------
// Synchronized clouds. Covers the synchronized-cloud branch of combine_pointclouds() and
// publish_clouds().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, SynchronizedCloudKeepsSensorFrameWhenConfiguredTo)
{
  // Arrange
  NodeParams params;
  params.keep_input_frame_in_synchronized_pointcloud = true;
  start(params);
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    const auto cloud = await_synchronized_cloud(i);
    EXPECT_EQ(cloud.header.frame_id, sensor_frames.at(i));
    EXPECT_EQ(cloud.width, num_points);
    EXPECT_EQ(cloud.point_step, xyzirc_point_step);
    EXPECT_TRUE(positions_match(
      read_points(cloud), expected_points_in_sensor_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, SynchronizedCloudUsesOutputFrameWhenConfiguredTo)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  params.keep_input_frame_in_synchronized_pointcloud = false;
  start(params);

  // Act
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    const auto cloud = await_synchronized_cloud(i);
    EXPECT_EQ(cloud.header.frame_id, output_frame);
    EXPECT_TRUE(positions_match(read_points(cloud), expected_points_in_output_frame(i, 0.0)))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, SynchronizedCloudIsStampedWithTheConcatenatedStamp)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();

  // Assert
  // Not the stamp of the cloud it came from: every synchronized cloud carries the oldest.
  const auto expected_oldest = to_time(*std::min_element(stamps.begin(), stamps.end()));
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_EQ(rclcpp::Time(await_synchronized_cloud(i).header.stamp), expected_oldest)
      << "sensor " << i;
  }
}

// ---------------------------------------------------------------------------------------
// A source that never arrives. Covers the CloudCollector timeout timer.
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, PublishesRemainingSourcesAfterTimeout)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_clouds_from(base_stamp_sec, {0, 1});
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(cloud.width, 2 * num_points);
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(stamps.front()));
  for (size_t i : {0u, 1u}) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i),
      expected_points_in_output_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

TEST_F(ConcatenateNodeTest, InfoMarksAMissingSourceAsTimedOut)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_clouds_from(base_stamp_sec, {0, 1});
  await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_FALSE(info.concatenation_success);
  const auto & missing = info.source_info.at(2);
  EXPECT_EQ(missing.topic, input_topics_.at(2));
  EXPECT_EQ(missing.status, SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(missing.idx_begin, 0u);
  EXPECT_EQ(missing.length, 0u);
  EXPECT_EQ(missing.header.frame_id, "");
}

// ---------------------------------------------------------------------------------------
// Empty inputs.
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, PublishesAnEmptyCloudWhenEverySourceIsEmpty)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec, Layout::xyzircaedt, true);
  const auto cloud = await_concatenated_cloud();

  // Assert
  EXPECT_EQ(cloud.width, 0u);
  EXPECT_EQ(cloud.row_step, 0u);
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(stamps.front()));
  EXPECT_EQ(cloud.header.frame_id, output_frame);
  // combine_pointclouds() forces the XYZIRC layout even with nothing to concatenate.
  EXPECT_EQ(cloud.point_step, xyzirc_point_step);
  EXPECT_EQ(cloud.fields.size(), 6u);
}

TEST_F(ConcatenateNodeTest, InfoReportsSuccessWhenEverySourceIsEmpty)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec, Layout::xyzircaedt, true);
  const auto info = await_concatenation_info();

  // Assert
  // An all-empty concatenation still counts as successful, because
  // every source reported STATUS_OK, with length 0.
  EXPECT_TRUE(info.concatenation_success);
  for (const auto & source : info.source_info) {
    EXPECT_EQ(source.status, SourcePointCloudInfo::STATUS_OK);
    EXPECT_EQ(source.length, 0u);
  }
}

// ---------------------------------------------------------------------------------------
// Naive matching. Covers NaiveMatchingPolicy::match().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, NaiveStrategyGroupsCloudsByArrivalTimeIgnoringTheirStamps)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "naive";
  params.is_motion_compensated = false;
  start(params);
  // Stamps far outside any advanced noise window: the advanced strategy would put each of
  // these in a collector of its own.
  const std::vector<double> stamps = {base_stamp_sec, base_stamp_sec + 5.0, base_stamp_sec + 11.0};

  // Act
  for (size_t i = 0; i < num_sensors; ++i) {
    publish_cloud(i, stamps.at(i));
  }
  const auto cloud = await_concatenated_cloud();

  // Assert
  EXPECT_EQ(cloud.width, num_sensors * num_points);
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(stamps.front()));
  EXPECT_EQ(concatenated_clouds_.size(), 1u);
}

// ---------------------------------------------------------------------------------------
// Clouds whose timestamp went backwards. Covers the drop-late and rosbag_length branches of
// publish_clouds().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, DropsCloudOlderThanTheLastPublishedOne)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  params.publish_previous_but_late_pointcloud = false;
  params.rosbag_length = 10.0;
  start(params);
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();
  ASSERT_EQ(concatenated_clouds_.size(), 1u);
  concatenated_clouds_.clear();
  concatenation_infos_.clear();

  // Act
  // One second earlier, i.e. well inside rosbag_length, so this counts as a late cloud
  // rather than a rosbag loop.
  publish_all_clouds(base_stamp_sec - 1.0);
  wait_out_the_timeout();

  // Assert
  EXPECT_TRUE(concatenated_clouds_.empty());
}

TEST_F(ConcatenateNodeTest, PublishesInfoEvenForACloudItDrops)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  params.publish_previous_but_late_pointcloud = false;
  params.rosbag_length = 10.0;
  start(params);
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();
  concatenated_clouds_.clear();
  concatenation_infos_.clear();
  const double late = base_stamp_sec - 1.0;

  // Act
  publish_all_clouds(late);
  const auto info = await_concatenation_info();

  // Assert
  EXPECT_EQ(rclcpp::Time(info.header.stamp), to_time(late));
  EXPECT_TRUE(info.concatenation_success);
  EXPECT_TRUE(concatenated_clouds_.empty());
}

TEST_F(ConcatenateNodeTest, PublishesCloudWhenTimeJumpsBackFurtherThanRosbagLength)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  params.publish_previous_but_late_pointcloud = false;
  params.rosbag_length = 10.0;
  start(params);
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();
  concatenated_clouds_.clear();
  concatenation_infos_.clear();
  const double looped = base_stamp_sec - 15.0;

  // Act
  publish_all_clouds(looped);
  const auto cloud = await_concatenated_cloud();

  // Assert
  EXPECT_EQ(rclcpp::Time(cloud.header.stamp), to_time(looped));
  EXPECT_EQ(cloud.width, num_sensors * num_points);
}

// ---------------------------------------------------------------------------------------
// Odometry as the velocity source. Covers odom_callback() -> process_odometry().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, OdometryDrivesMotionCompensationLikeTwistDoes)
{
  // Arrange
  NodeParams params;
  params.input_twist_topic_type = "odom";
  start(params);
  publish_odometry(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto cloud = await_concatenated_cloud();
  const auto info = await_concatenation_info();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_TRUE(positions_match(
      get_segment_of(info, cloud, i),
      expected_points_in_output_frame(i, calculate_motion_shift_x(i))))
      << "sensor " << i;
  }
}

// ---------------------------------------------------------------------------------------
// Diagnostics. Covers check_concat_status().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, DiagnosticsIsNamedAfterTheNode)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.hardware_id, node_name_);
  EXPECT_EQ(status.name, node_name_ + ": " + in_namespace("/" + node_name_));
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsOkWhenEverySourceArrives)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "OK");
  EXPECT_EQ(get_diagnostic_value(status, "Pointcloud concatenation succeeded"), "True");
}

TEST_F(ConcatenateNodeTest, DiagnosticsListsItsKeysInAFixedOrder)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "advanced";
  start(params);
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);

  // Assert
  EXPECT_EQ(get_diagnostic_keys(await_diagnostic()), expected_diagnostic_keys(input_topics_));
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsTheAdvancedReferenceWindow)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "advanced";
  start(params);
  publish_twist(base_stamp_sec);

  // Act
  const auto stamps = publish_all_clouds(base_stamp_sec);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(
    get_diagnostic_value(status, "Concatenated pointcloud timestamp"),
    format_seconds(stamps.front()));
  EXPECT_EQ(
    get_diagnostic_value(status, "Minimum reference timestamp"),
    format_seconds(base_stamp_sec - noise_window));
  EXPECT_EQ(
    get_diagnostic_value(status, "Maximum reference timestamp"),
    format_seconds(base_stamp_sec + noise_window));
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsAnArrivalTimestampForTheNaiveStrategy)
{
  // Arrange
  NodeParams params;
  params.matching_strategy = "naive";
  params.is_motion_compensated = false;
  start(params);

  // Act
  publish_all_clouds(base_stamp_sec);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_TRUE(has_diagnostic_key(status, "First pointcloud arrival timestamp"));
  EXPECT_FALSE(has_diagnostic_key(status, "Minimum reference timestamp"));
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsErrorWhenASourceIsMissing)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_clouds_from(base_stamp_sec, {0, 1});
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is published but misses some topics");
  EXPECT_EQ(get_diagnostic_value(status, "Concatenated: " + input_topics_.at(2)), "False");
  EXPECT_EQ(get_diagnostic_value(status, "Pointcloud concatenation succeeded"), "False");
  // A missing topic contributes no "Timestamp:" or "Latency (ms):" entry at all.
  EXPECT_FALSE(has_diagnostic_key(status, "Timestamp: " + input_topics_.at(2)));
  EXPECT_FALSE(has_diagnostic_key(status, "Latency (ms): " + input_topics_.at(2)));
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsErrorWhenTheConcatenatedCloudIsEmpty)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec, Layout::xyzircaedt, true);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is empty");
}

TEST_F(ConcatenateNodeTest, DiagnosticsReportsErrorWhenACloudIsDroppedAsLate)
{
  // Arrange
  NodeParams params;
  params.is_motion_compensated = false;
  params.publish_previous_but_late_pointcloud = false;
  params.rosbag_length = 10.0;
  start(params);
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();
  diagnostics_.clear();
  const double late = base_stamp_sec - 1.0;

  // Act
  publish_all_clouds(late);
  const auto status = await_diagnostic();

  // Assert
  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "Concatenated pointcloud was dropped due to its timestamp is earlier than the latest "
    "published one");
  // The diagnostic still describes the cloud that was dropped.
  EXPECT_EQ(
    get_diagnostic_value(status, "Concatenated pointcloud timestamp"), format_seconds(late));
}

// ---------------------------------------------------------------------------------------
// Debug topics. Covers publish_debug_message().
// ---------------------------------------------------------------------------------------

TEST_F(ConcatenateNodeTest, PublishesProcessingAndCyclicTimeOnDebugTopics)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();

  // Assert
  ASSERT_FALSE(processing_times_.empty());
  ASSERT_FALSE(cyclic_times_.empty());
  EXPECT_GE(processing_times_.front().data, 0.0);
  EXPECT_GE(cyclic_times_.front().data, 0.0);
}

TEST_F(ConcatenateNodeTest, PublishesPipelineLatencyPerInputTopic)
{
  // Arrange
  start(NodeParams{});
  publish_twist(base_stamp_sec);

  // Act
  publish_all_clouds(base_stamp_sec);
  await_concatenated_cloud();

  // Assert
  for (size_t i = 0; i < num_sensors; ++i) {
    EXPECT_FALSE(pipeline_latencies_.at(i).empty()) << "no latency for sensor " << i;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
