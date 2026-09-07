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

#include "in_lane_mrm_planner_node.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <cstdint>
#include <memory>

namespace autoware::in_lane_mrm_planner
{

namespace
{
enum class StatusReasonCode : int32_t {
  PUBLISHED_OK = 0,
  WAITING_MAP = 1,
  WAITING_ROUTE = 2,
  WAITING_ODOMETRY = 3,
  WAITING_ACCEL = 4,
  PLAN_FAILED_UPDATE_CURRENT_LANELET = 10,
  PLAN_FAILED_BACKWARD_LANELETS = 11,
  PLAN_FAILED_FORWARD_LANELETS = 12,
  PLAN_FAILED_INVALID_S_RANGE = 13,
  PLAN_FAILED_GENERATE_PATH = 14,
  PLAN_FAILED_NO_OUTPUT = 15,
  VALIDATION_FAILED_POINT_COUNT = 20,
  VALIDATION_FAILED_NON_FINITE = 21,
  LATCHED_OUTPUT_PUBLISHED = 30,
  LATCHED_WITHOUT_CANDIDATE = 31,
  UNKNOWN = 99,
};

int to_reason_code(const PathPlanner::PlanFailureCode code)
{
  using Code = PathPlanner::PlanFailureCode;
  switch (code) {
    case Code::UPDATE_CURRENT_LANELET_FAILED:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_UPDATE_CURRENT_LANELET);
    case Code::BACKWARD_LANELETS_FAILED:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_BACKWARD_LANELETS);
    case Code::FORWARD_LANELETS_FAILED:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_FORWARD_LANELETS);
    case Code::INVALID_S_RANGE:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_INVALID_S_RANGE);
    case Code::LANELET_SEQUENCE_EMPTY:
    case Code::NO_PATH_POINTS:
    case Code::TRAJECTORY_BUILD_FAILED:
    case Code::TRAJECTORY_TOO_SHORT_AFTER_CROP:
    case Code::FINALIZED_PATH_EMPTY:
    case Code::GENERATE_PATH_FAILED:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_GENERATE_PATH);
    case Code::NONE:
    default:
      return static_cast<int>(StatusReasonCode::PLAN_FAILED_NO_OUTPUT);
  }
}

int to_reason_code(const InLaneMrmTrajectoryValidator::FailureCode code)
{
  using Code = InLaneMrmTrajectoryValidator::FailureCode;
  switch (code) {
    case Code::INSUFFICIENT_POINT_COUNT:
      return static_cast<int>(StatusReasonCode::VALIDATION_FAILED_POINT_COUNT);
    case Code::NON_FINITE_VALUES:
      return static_cast<int>(StatusReasonCode::VALIDATION_FAILED_NON_FINITE);
    case Code::NONE:
    default:
      return static_cast<int>(StatusReasonCode::UNKNOWN);
  }
}
}  // namespace

InLaneMrmPlannerNode::InLaneMrmPlannerNode(const rclcpp::NodeOptions & options)
: Node("in_lane_mrm_planner", options),
  param_listener_(
    std::make_shared<::in_lane_mrm_planner::ParamListener>(get_node_parameters_interface())),
  params_(param_listener_->get_params()),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  time_keeper_(std::make_shared<autoware_utils_debug::TimeKeeper>()),
  velocity_planner_(params_),
  trajectory_validator_(params_),
  route_subscriber_(this, "~/input/route", rclcpp::QoS{1}.transient_local()),
  vector_map_subscriber_(this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()),
  kinematic_state_subscriber_(this, "~/input/kinematic_state"),
  acceleration_subscriber_(this, "~/input/acceleration"),
  objects_subscriber_(this, "~/input/objects"),
  trigger_subscriber_(this, "~/input/trigger")
{
  path_planner_ =
    std::make_unique<PathPlanner>(get_logger(), get_clock(), time_keeper_, params_, vehicle_info_);
  trajectory_planner_ =
    std::make_unique<InLaneMrmTrajectoryPlanner>(*path_planner_, params_, vehicle_info_);

  trajectory_smoother_.initialize(this, params_);
  trajectory_modifier_.initialize(this, vehicle_info_, params_);

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_debug_status_ = create_publisher<Float32MultiArrayStamped>("~/debug/planner_status", 1);

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&InLaneMrmPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "InLaneMrmPlannerNode started.");
}

void InLaneMrmPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto cycle_start = now();
  DebugStatus status{};

  const auto input_data = take_data();
  status.data_ready = is_data_ready(input_data);
  status.trigger_active = input_data.trigger_ptr && input_data.trigger_ptr->data;
  status.is_latched = trajectory_latcher_.is_latched();
  status.has_latest_candidate = trajectory_latcher_.has_latest_candidate();
  if (input_data.odometry_ptr) {
    status.odom_vx = input_data.odometry_ptr->twist.twist.linear.x;
  }
  path_planner_->set_planner_data(input_data.lanelet_map_bin_ptr, input_data.route_ptr);

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  if (!status.data_ready) {
    if (!input_data.lanelet_map_bin_ptr) {
      status.reason_code = static_cast<int>(StatusReasonCode::WAITING_MAP);
    } else if (!input_data.route_ptr) {
      status.reason_code = static_cast<int>(StatusReasonCode::WAITING_ROUTE);
    } else if (!input_data.odometry_ptr) {
      status.reason_code = static_cast<int>(StatusReasonCode::WAITING_ODOMETRY);
    } else if (!input_data.acceleration_ptr) {
      status.reason_code = static_cast<int>(StatusReasonCode::WAITING_ACCEL);
    } else {
      status.reason_code = static_cast<int>(StatusReasonCode::UNKNOWN);
    }
    status.cycle_time_ms = (now() - cycle_start).seconds() * 1e3;
    publish_debug_status(status);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for map, route, odometry, or acceleration.");
    return;
  }

  const auto & odom = *input_data.odometry_ptr;
  const auto & accel = *input_data.acceleration_ptr;
  PredictedObjects empty_objects;
  const auto & live_objects = input_data.objects_ptr ? *input_data.objects_ptr : empty_objects;

  const bool trigger_active = input_data.trigger_ptr && input_data.trigger_ptr->data;
  const auto trigger_edges = trigger_edge_detector_.update(trigger_active);
  if (trigger_edges.rising) {
    objects_latcher_.latch(live_objects, params_.latch.use_latched_objects);
    trajectory_latcher_.latch();
  }
  if (trigger_edges.falling) {
    objects_latcher_.unlatch();
    trajectory_latcher_.unlatch();
  }

  std::optional<Trajectory> trajectory_to_publish;

  if (trajectory_latcher_.is_latched()) {
    trajectory_to_publish = trajectory_latcher_.output();
    status.plan_ok = true;
    status.validation_ok = true;
    status.reason_code = trajectory_to_publish
                           ? static_cast<int>(StatusReasonCode::LATCHED_OUTPUT_PUBLISHED)
                           : static_cast<int>(StatusReasonCode::LATCHED_WITHOUT_CANDIDATE);
    if (trajectory_to_publish) {
      status.published_points = trajectory_to_publish->points.size();
    }
  } else {
    const auto planned_traj = trajectory_planner_->plan(odom);
    if (planned_traj) {
      status.plan_ok = true;
      auto traj = *planned_traj;
      status.planned_points = traj.points.size();
      trajectory_smoother_.smooth(traj.points, odom.pose.pose);
      trajectory_modifier_.set_objects(objects_latcher_.objects_for_planning(live_objects));
      trajectory_modifier_.apply(traj.points, odom, accel);
      velocity_planner_.apply(traj.points, odom, accel);
      trajectory_modifier_.publish_planning_factor();

      const auto validation = trajectory_validator_.validate(traj.points);
      if (validation.ok) {
        status.validation_ok = true;
        traj.header.stamp = now();
        traj.header.frame_id = odom.header.frame_id;
        trajectory_latcher_.update_candidate(trajectory_selector_.select(traj));
        trajectory_to_publish = trajectory_latcher_.output();
        if (trajectory_to_publish) {
          status.published_points = trajectory_to_publish->points.size();
          status.reason_code = static_cast<int>(StatusReasonCode::PUBLISHED_OK);
        } else {
          status.reason_code = static_cast<int>(StatusReasonCode::PLAN_FAILED_NO_OUTPUT);
        }
      } else {
        status.reason_code = to_reason_code(validation.code);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Trajectory validation failed: %s",
          validation.reason.c_str());
      }
    } else {
      status.reason_code = to_reason_code(path_planner_->last_failure_code());
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Trajectory planning failed.");
    }
  }

  if (trajectory_to_publish) {
    auto published = *trajectory_to_publish;
    published.header.stamp = now();
    if (published.header.frame_id.empty()) {
      published.header.frame_id = odom.header.frame_id;
    }
    pub_trajectory_->publish(published);
  }

  status.is_latched = trajectory_latcher_.is_latched();
  status.has_latest_candidate = trajectory_latcher_.has_latest_candidate();
  status.cycle_time_ms = (now() - cycle_start).seconds() * 1e3;
  publish_debug_status(status);
}

InLaneMrmPlannerNode::InputData InLaneMrmPlannerNode::take_data()
{
  InputData input_data;
  input_data.route_ptr = route_ptr_;
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;
  input_data.odometry_ptr = odometry_ptr_;
  input_data.acceleration_ptr = acceleration_ptr_;
  input_data.objects_ptr = objects_ptr_;
  input_data.trigger_ptr = trigger_ptr_;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received empty route; ignoring.");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = kinematic_state_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    objects_ptr_ = msg;
  }
  input_data.objects_ptr = objects_ptr_;

  if (const auto msg = trigger_subscriber_.take_data()) {
    trigger_ptr_ = msg;
  }
  input_data.trigger_ptr = trigger_ptr_;

  return input_data;
}

bool InLaneMrmPlannerNode::is_data_ready(const InputData & input_data) const
{
  return input_data.lanelet_map_bin_ptr && input_data.route_ptr && input_data.odometry_ptr &&
         input_data.acceleration_ptr;
}

void InLaneMrmPlannerNode::update_params()
{
  params_ = param_listener_->get_params();
  path_planner_->update_params(params_);
  velocity_planner_.update_params(params_);
  trajectory_validator_.update_params(params_);
}

void InLaneMrmPlannerNode::publish_debug_status(const DebugStatus & status)
{
  Float32MultiArrayStamped msg;
  msg.stamp = now();
  msg.data = {
    static_cast<float>(status.reason_code),
    status.trigger_active ? 1.0F : 0.0F,
    status.is_latched ? 1.0F : 0.0F,
    status.has_latest_candidate ? 1.0F : 0.0F,
    status.data_ready ? 1.0F : 0.0F,
    status.plan_ok ? 1.0F : 0.0F,
    status.validation_ok ? 1.0F : 0.0F,
    static_cast<float>(status.planned_points),
    static_cast<float>(status.published_points),
    static_cast<float>(status.cycle_time_ms),
    static_cast<float>(status.odom_vx),
  };
  pub_debug_status_->publish(msg);
}

}  // namespace autoware::in_lane_mrm_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::in_lane_mrm_planner::InLaneMrmPlannerNode)
