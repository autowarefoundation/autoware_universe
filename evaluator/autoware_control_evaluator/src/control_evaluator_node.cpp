// Copyright 2025 TIER IV, Inc.
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

#include "autoware/control_evaluator/control_evaluator_node.hpp"

#include "autoware/control_evaluator/metrics/metrics_utils.hpp"

#include <autoware/boundary_departure_checker/conversion.hpp>
#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <nlohmann/json.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace control_diagnostics
{
namespace bg = boost::geometry;

ControlEvaluatorNode::ControlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  // planning_factor subscribers
  std::vector<std::string> stop_deviation_modules_list =
    declare_parameter<std::vector<std::string>>(
      "planning_factor_metrics.stop_deviation.module_list");
  stop_deviation_modules_ = std::unordered_set<std::string>(
    stop_deviation_modules_list.begin(), stop_deviation_modules_list.end());

  const std::string topic_prefix =
    declare_parameter<std::string>("planning_factor_metrics.topic_prefix");
  for (const auto & module_name : stop_deviation_modules_) {
    planning_factors_sub_.emplace(
      module_name, autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>(
                     this, topic_prefix + module_name));
    stop_deviation_accumulators_.emplace(module_name, Accumulator<double>());
    stop_deviation_abs_accumulators_.emplace(module_name, Accumulator<double>());
  }

  // Publisher
  processing_time_pub_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  metrics_pub_ = create_publisher<MetricArrayMsg>("~/metrics", 1);

  // Parameters
  output_metrics_ = declare_parameter<bool>("output_metrics");
  distance_filter_thr_m_ = declare_parameter<double>("object_metrics.distance_filter_thr_m");

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ControlEvaluatorNode::onTimer, this));
}

ControlEvaluatorNode::~ControlEvaluatorNode()
{
  if (!output_metrics_) {
    return;
  }

  try {
    // generate json data
    using json = nlohmann::json;
    json j;
    for (Metric metric : metrics_) {
      if (metric_accumulators_[static_cast<size_t>(metric)].count() == 0) {
        continue;
      }
      const std::string base_name = metric_to_str.at(metric) + "/";
      j[base_name + "min"] = metric_accumulators_[static_cast<size_t>(metric)].min();
      j[base_name + "max"] = metric_accumulators_[static_cast<size_t>(metric)].max();
      j[base_name + "mean"] = metric_accumulators_[static_cast<size_t>(metric)].mean();
      j[base_name + "count"] = metric_accumulators_[static_cast<size_t>(metric)].count();
      j[base_name + "description"] = metric_descriptions.at(metric);
    }
    // processing json for stop_deviation
    j["stop_deviation/description"] = metric_descriptions.at(Metric::stop_deviation);
    for (const auto & [module_name, stop_deviation_accumulator] : stop_deviation_accumulators_) {
      if (stop_deviation_accumulator.count() == 0) {
        continue;
      }
      const std::string base_name = "stop_deviation/" + module_name + "/";
      j[base_name + "min"] = stop_deviation_accumulator.min();
      j[base_name + "max"] = stop_deviation_accumulator.max();
      j[base_name + "mean"] = stop_deviation_accumulator.mean();
      j[base_name + "count"] = stop_deviation_accumulator.count();
    }
    j["stop_deviation_abs/description"] = metric_descriptions.at(Metric::stop_deviation_abs);
    for (const auto & [module_name, stop_deviation_abs_accumulator] :
         stop_deviation_abs_accumulators_) {
      if (stop_deviation_abs_accumulator.count() == 0) {
        continue;
      }
      const std::string base_name = "stop_deviation_abs/" + module_name + "/";
      j[base_name + "min"] = stop_deviation_abs_accumulator.min();
      j[base_name + "max"] = stop_deviation_abs_accumulator.max();
      j[base_name + "mean"] = stop_deviation_abs_accumulator.mean();
      j[base_name + "count"] = stop_deviation_abs_accumulator.count();
    }

    // get output folder
    const std::string output_folder_str =
      rclcpp::get_logging_directory().string() + "/autoware_metrics";
    if (!std::filesystem::exists(output_folder_str)) {
      if (!std::filesystem::create_directories(output_folder_str)) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to create directories: %s", output_folder_str.c_str());
        return;
      }
    }

    // get time stamp
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm * local_time = std::localtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d-%H-%M-%S");
    std::string cur_time_str = oss.str();

    // Write metrics .json to file
    const std::string output_file_str =
      output_folder_str + "/autoware_control_evaluator-" + cur_time_str + ".json";
    std::ofstream f(output_file_str);
    if (f.is_open()) {
      f << j.dump(4);
      f.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", output_file_str.c_str());
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in ControlEvaluatorNode destructor: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in ControlEvaluatorNode destructor" << std::endl;
  }
}

void ControlEvaluatorNode::getRouteData()
{
  // route
  {
    const auto msg = route_subscriber_.take_data();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_.setRoute(*msg);
      }
    }
  }

  // map
  {
    const auto msg = vector_map_subscriber_.take_data();
    if (msg) {
      route_handler_.setMap(*msg);
    }
  }
}

void ControlEvaluatorNode::AddMetricMsg(
  const Metric & metric, const double & metric_value, const bool & accumulate_metric)
{
  MetricMsg metric_msg;
  metric_msg.name = metric_to_str.at(metric);
  metric_msg.value = std::to_string(metric_value);
  metrics_msg_.metric_array.push_back(metric_msg);
  if (output_metrics_ && accumulate_metric) {
    metric_accumulators_[static_cast<size_t>(metric)].add(metric_value);
  }
}

void ControlEvaluatorNode::AddLaneletInfoMsg(const Pose & ego_pose)
{
  const auto current_lanelets = metrics::utils::get_current_lanes(route_handler_, ego_pose);
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);

  const std::string base_name = "ego_lane_info/";
  MetricMsg metric_msg;

  {
    metric_msg.name = base_name + "lane_id";
    metric_msg.value = std::to_string(current_lane.id());
    metrics_msg_.metric_array.push_back(metric_msg);
  }

  {
    metric_msg.name = base_name + "s";
    metric_msg.value = std::to_string(arc_coordinates.length);
    metrics_msg_.metric_array.push_back(metric_msg);
  }

  {
    metric_msg.name = base_name + "t";
    metric_msg.value = std::to_string(arc_coordinates.distance);
    metrics_msg_.metric_array.push_back(metric_msg);
  }
}

void ControlEvaluatorNode::AddBoundaryDistanceMetricMsg(
  const PathWithLaneId & behavior_path, const Pose & ego_pose)
{
  const auto current_lanelets = metrics::utils::get_current_lanes(route_handler_, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  const auto current_vehicle_footprint = autoware_utils::transform_vector(
    local_vehicle_footprint, autoware_utils::pose2transform(ego_pose));

  if (behavior_path.left_bound.size() >= 1) {
    LineString2d left_boundary;
    for (const auto & p : behavior_path.left_bound) left_boundary.push_back(Point2d(p.x, p.y));
    double distance_to_left_boundary =
      metrics::utils::calc_distance_to_line(current_vehicle_footprint, left_boundary);

    if (metrics::utils::is_point_left_of_line(ego_pose.position, behavior_path.left_bound)) {
      distance_to_left_boundary *= -1.0;
    }
    const Metric metric_left = Metric::left_boundary_distance;
    AddMetricMsg(metric_left, distance_to_left_boundary);
  }

  if (behavior_path.right_bound.size() >= 1) {
    LineString2d right_boundary;
    for (const auto & p : behavior_path.right_bound) right_boundary.push_back(Point2d(p.x, p.y));
    double distance_to_right_boundary =
      metrics::utils::calc_distance_to_line(current_vehicle_footprint, right_boundary);

    if (!metrics::utils::is_point_left_of_line(ego_pose.position, behavior_path.right_bound)) {
      distance_to_right_boundary *= -1.0;
    }
    const Metric metric_right = Metric::right_boundary_distance;
    AddMetricMsg(metric_right, distance_to_right_boundary);
  }
}

void ControlEvaluatorNode::AddUncrossableBoundaryDistanceMetricMsg(const Pose & ego_pose)
{
  namespace bdc_utils = autoware::boundary_departure_checker::utils;

  constexpr auto search_dist_offset{5.0};  // Extra range to include slightly farther boundaries

  const auto search_distance =
    std::max(vehicle_info_.max_longitudinal_offset_m, vehicle_info_.max_lateral_offset_m) +
    search_dist_offset;

  const auto nearby_uncrossable_lines_opt = bdc_utils::get_uncrossable_linestrings_near_pose(
    route_handler_.getLaneletMapPtr(), ego_pose, search_distance);

  if (!nearby_uncrossable_lines_opt) {
    return;
  }

  const auto & nearby_uncrossable_lines = *nearby_uncrossable_lines_opt;

  const auto transformed_pose = autoware_utils::pose2transform(ego_pose);
  const auto local_fp = vehicle_info_.createFootprint();
  const auto current_fp = autoware_utils::transform_vector(local_fp, transformed_pose);
  const auto side = bdc_utils::get_footprint_sides(current_fp, false, false);

  auto nearest_left = search_distance;
  auto nearest_right = search_distance;

  auto is_overlapping{false};
  for (const auto & nearby_ls : nearby_uncrossable_lines) {
    LineString2d boundary;
    const auto & basic_ls = nearby_ls.basicLineString();
    boundary.reserve(basic_ls.size());
    for (size_t idx = 0; idx + 1 < basic_ls.size(); ++idx) {
      const auto segment = bdc_utils::to_segment_2d(basic_ls[idx], basic_ls[idx + 1]);

      is_overlapping = !boost::geometry::disjoint(current_fp, segment);

      if (is_overlapping) {
        nearest_left = 0.0;
        nearest_right = 0.0;
        break;
      }

      const auto dist_to_left = boost::geometry::distance(segment, side.left);
      const auto dist_to_right = boost::geometry::distance(segment, side.right);
      if (dist_to_left < dist_to_right) {
        nearest_left = std::min(dist_to_left, nearest_left);
      } else {
        nearest_right = std::min(dist_to_right, nearest_right);
      }
    }
    if (is_overlapping) {
      break;
    }
  }
  const Metric metric_left = Metric::left_uncrossable_boundary_distance;
  AddMetricMsg(metric_left, nearest_left);

  const Metric metric_right = Metric::right_uncrossable_boundary_distance;
  AddMetricMsg(metric_right, nearest_right);
}

void ControlEvaluatorNode::AddKinematicStateMetricMsg(
  const Odometry & odom, const AccelWithCovarianceStamped & accel_stamped)
{
  AddMetricMsg(Metric::velocity, odom.twist.twist.linear.x);

  const auto & acc = accel_stamped.accel.accel.linear.x;
  AddMetricMsg(Metric::acceleration, acc);

  const auto jerk = [&]() {
    if (!prev_acc_stamped_.has_value()) {
      prev_acc_stamped_ = accel_stamped;
      return 0.0;
    }
    const auto t = static_cast<double>(accel_stamped.header.stamp.sec) +
                   static_cast<double>(accel_stamped.header.stamp.nanosec) * 1e-9;
    const auto prev_t = static_cast<double>(prev_acc_stamped_.value().header.stamp.sec) +
                        static_cast<double>(prev_acc_stamped_.value().header.stamp.nanosec) * 1e-9;
    const auto dt = t - prev_t;
    if (dt < std::numeric_limits<double>::epsilon()) return 0.0;

    const auto prev_acc = prev_acc_stamped_.value().accel.accel.linear.x;
    prev_acc_stamped_ = accel_stamped;
    return (acc - prev_acc) / dt;
  }();
  AddMetricMsg(Metric::jerk, jerk);
}

void ControlEvaluatorNode::AddSteeringMetricMsg(const SteeringReport & steering_status)
{
  // steering angle
  double cur_steering_angle = steering_status.steering_tire_angle;
  double cur_steering_angle_abs = std::abs(cur_steering_angle);
  const double cur_t = static_cast<double>(steering_status.stamp.sec) +
                       static_cast<double>(steering_status.stamp.nanosec) * 1e-9;
  AddMetricMsg(Metric::steering_angle, cur_steering_angle);
  AddMetricMsg(Metric::steering_angle_abs, cur_steering_angle_abs);

  if (!prev_steering_angle_timestamp_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  // d_t
  const double dt = cur_t - prev_steering_angle_timestamp_.value();
  if (dt < std::numeric_limits<double>::epsilon()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  // steering rate
  const double steering_rate = (cur_steering_angle - prev_steering_angle_.value()) / dt;
  AddMetricMsg(Metric::steering_rate, steering_rate);

  // steering acceleration
  if (!prev_steering_rate_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    prev_steering_rate_ = steering_rate;
    return;
  }
  const double steering_acceleration = (steering_rate - prev_steering_rate_.value()) / dt;
  AddMetricMsg(Metric::steering_acceleration, steering_acceleration);

  prev_steering_angle_timestamp_ = cur_t;
  prev_steering_angle_ = cur_steering_angle;
  prev_steering_rate_ = steering_rate;
}

void ControlEvaluatorNode::AddLateralDeviationMetricMsg(
  const Trajectory & traj, const Point & ego_point)
{
  const double metric_value = metrics::calcLateralDeviation(traj, ego_point);
  const double metric_value_abs = std::abs(metric_value);

  AddMetricMsg(Metric::lateral_deviation, metric_value);
  AddMetricMsg(Metric::lateral_deviation_abs, metric_value_abs);
}

void ControlEvaluatorNode::AddYawDeviationMetricMsg(const Trajectory & traj, const Pose & ego_pose)
{
  const double metric_value = metrics::calcYawDeviation(traj, ego_pose);
  const double metric_value_abs = std::abs(metric_value);

  AddMetricMsg(Metric::yaw_deviation, metric_value);
  AddMetricMsg(Metric::yaw_deviation_abs, metric_value_abs);
}

void ControlEvaluatorNode::AddGoalDeviationMetricMsg(const Odometry & odom)
{
  const Pose ego_pose = odom.pose.pose;
  const double longitudinal_deviation_value =
    metrics::calcLongitudinalDeviation(route_handler_.getGoalPose(), ego_pose.position);
  const double longitudinal_deviation_value_abs = std::abs(longitudinal_deviation_value);
  const double lateral_deviation_value =
    metrics::calcLateralDeviation(route_handler_.getGoalPose(), ego_pose.position);
  const double lateral_deviation_value_abs = std::abs(lateral_deviation_value);
  const double yaw_deviation_value =
    metrics::calcYawDeviation(route_handler_.getGoalPose(), ego_pose);
  const double yaw_deviation_value_abs = std::abs(yaw_deviation_value);

  const bool is_ego_stopped_near_goal =
    std::abs(longitudinal_deviation_value) < 3.0 && std::abs(odom.twist.twist.linear.x) < 0.001;

  AddMetricMsg(
    Metric::goal_longitudinal_deviation, longitudinal_deviation_value, is_ego_stopped_near_goal);
  AddMetricMsg(Metric::goal_lateral_deviation, lateral_deviation_value, is_ego_stopped_near_goal);
  AddMetricMsg(Metric::goal_yaw_deviation, yaw_deviation_value, is_ego_stopped_near_goal);
  AddMetricMsg(
    Metric::goal_longitudinal_deviation_abs, longitudinal_deviation_value_abs,
    is_ego_stopped_near_goal);
  AddMetricMsg(
    Metric::goal_lateral_deviation_abs, lateral_deviation_value_abs, is_ego_stopped_near_goal);
  AddMetricMsg(Metric::goal_yaw_deviation_abs, yaw_deviation_value_abs, is_ego_stopped_near_goal);
}

void ControlEvaluatorNode::AddStopDeviationMetricMsg(const Odometry & odom)
{
  const auto get_min_distance_signed =
    [](const PlanningFactorArray::ConstSharedPtr & planning_factors) -> std::optional<double> {
    std::optional<double> min_distance = std::nullopt;
    for (const auto & factor : planning_factors->factors) {
      if (factor.behavior == PlanningFactor::STOP) {
        for (const auto & control_point : factor.control_points) {
          const auto cur_distance = control_point.distance;
          if (!min_distance || std::abs(cur_distance) < std::abs(*min_distance)) {
            min_distance = cur_distance;
          }
        }
      }
    }
    return min_distance;
  };

  // get min_distance from each module
  std::vector<std::pair<std::string, double>> min_distances;
  for (auto & [module_name, planning_factor_sub_] : planning_factors_sub_) {
    const auto planning_factors = planning_factor_sub_.take_data();
    if (
      !planning_factors || planning_factors->factors.empty() ||
      stop_deviation_modules_.count(module_name) == 0) {
      continue;
    }
    const auto min_distance = get_min_distance_signed(planning_factors);
    if (min_distance) {
      min_distances.emplace_back(module_name, *min_distance);
    }
  }
  if (min_distances.empty()) {
    return;
  }

  // find the stop decision closest to the ego only, the other stop decisions are not accumulated
  const auto min_distance_pair = std::min_element(
    min_distances.begin(), min_distances.end(),
    [](const auto & a, const auto & b) { return std::abs(a.second) < std::abs(b.second); });

  const auto [closest_module_name, closest_min_distance] = *min_distance_pair;
  const bool is_ego_stopped_near_stop_decision =
    std::abs(closest_min_distance) < 3.0 && std::abs(odom.twist.twist.linear.x) < 0.001;
  if (output_metrics_ && is_ego_stopped_near_stop_decision) {
    stop_deviation_accumulators_[closest_module_name].add(closest_min_distance);
    stop_deviation_abs_accumulators_[closest_module_name].add(std::abs(closest_min_distance));
  }

  // add metrics for each module
  for (const auto & [module_name, min_distance] : min_distances) {
    MetricMsg metric_msg;
    metric_msg.name = "stop_deviation/" + module_name;
    metric_msg.value = std::to_string(min_distance);
    metrics_msg_.metric_array.push_back(metric_msg);

    MetricMsg metric_msg_abs;
    metric_msg_abs.name = "stop_deviation_abs/" + module_name;
    metric_msg_abs.value = std::to_string(std::abs(min_distance));
    metrics_msg_.metric_array.push_back(metric_msg_abs);
  }
}

void ControlEvaluatorNode::AddObjectMetricMsg(
  const Odometry & odom, const PredictedObjects & objects)
{
  if (objects.objects.empty()) {
    return;
  }

  const auto ego_polygon = [&]() -> autoware_utils::Polygon2d {
    const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info_.createFootprint();
    const autoware_utils::LinearRing2d ego_footprint = autoware_utils::transform_vector(
      local_ego_footprint, autoware_utils::pose2transform(odom.pose.pose));

    autoware_utils::Polygon2d ego_polygon;
    ego_polygon.outer() = ego_footprint;
    bg::correct(ego_polygon);
    return ego_polygon;
  }();

  double minimum_distance = std::numeric_limits<double>::max();
  for (const auto & object : objects.objects) {
    const double center_distance = autoware_utils::calc_distance2d(
      odom.pose.pose.position, object.kinematics.initial_pose_with_covariance.pose.position);
    if (center_distance > distance_filter_thr_m_) {
      continue;
    }

    const auto object_polygon = autoware_utils::to_polygon2d(object);
    const auto distance = bg::distance(ego_polygon, object_polygon);
    if (distance < minimum_distance) {
      minimum_distance = distance;
    }
  }

  if (minimum_distance == std::numeric_limits<double>::max()) {
    return;
  }

  AddMetricMsg(Metric::closest_object_distance, minimum_distance);
}

void ControlEvaluatorNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  const auto odom = odometry_sub_.take_data();
  if (odom) {
    const Pose ego_pose = odom->pose.pose;

    // add planning_factor related metrics
    AddStopDeviationMetricMsg(*odom);

    // add object related metrics
    const auto objects = objects_sub_.take_data();
    if (objects) {
      AddObjectMetricMsg(*odom, *objects);
    }

    // add kinematic info
    const auto acc = accel_sub_.take_data();
    if (acc) {
      AddKinematicStateMetricMsg(*odom, *acc);
    }

    // add deviation metrics
    const auto traj = traj_sub_.take_data();
    if (traj && !traj->points.empty()) {
      AddLateralDeviationMetricMsg(*traj, ego_pose.position);
      AddYawDeviationMetricMsg(*traj, ego_pose);
    }

    getRouteData();
    if (route_handler_.isHandlerReady()) {
      // add goal deviation metrics
      AddLaneletInfoMsg(ego_pose);
      AddGoalDeviationMetricMsg(*odom);

      // add boundary distance metrics
      const auto behavior_path = behavior_path_subscriber_.take_data();
      if (behavior_path) {
        AddBoundaryDistanceMetricMsg(*behavior_path, ego_pose);
      }
      AddUncrossableBoundaryDistanceMetricMsg(ego_pose);
    }
  }

  // add steering metrics
  const auto steering_status = steering_sub_.take_data();
  if (steering_status) {
    AddSteeringMetricMsg(*steering_status);
  }

  // Publish metrics
  metrics_msg_.stamp = now();
  metrics_pub_->publish(metrics_msg_);
  metrics_msg_ = MetricArrayMsg{};

  // Publish processing time
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  processing_time_pub_->publish(processing_time_msg);
}
}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::ControlEvaluatorNode)
