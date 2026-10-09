// Copyright 2026 Xinchen Lin
// Licensed under the Apache License, Version 2.0.

#include "autoware/camp_diffusion_adapter/adapter.hpp"

#include <autoware/diffusion_planner/utils/planning_factor_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_camp_selector/msg/camp_candidate_pool.hpp>
#include <autoware_camp_selector/msg/camp_selection.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::camp_diffusion_adapter
{
using autoware_camp_selector::msg::CampCandidatePool;
using autoware_camp_selector::msg::CampSelection;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class CampDiffusionAdapterNode : public rclcpp::Node
{
public:
  explicit CampDiffusionAdapterNode(const rclcpp::NodeOptions & options)
  : Node("camp_diffusion_adapter", options), generator_uuid_(autoware_utils_uuid::generate_uuid())
  {
    const auto params = read_planner_params();
    if (!std::isfinite(params.planning_frequency_hz) || params.planning_frequency_hz <= 0.0) {
      throw std::invalid_argument("planning_frequency_hz must be finite and positive");
    }
    if (params.build_only) {
      throw std::invalid_argument("the CAMP runtime adapter requires build_only=false");
    }
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    adapter_ = std::make_unique<CampDiffusionAdapter>(
      params, vehicle_info, read_parameter<std::string>("fixed_weight_model_path", ""));
    start_guidance_service_ = guidance_service(
      "~/service/set_start_guidance_enabled", &CampDiffusionAdapter::set_start_guidance_enabled);
    stop_guidance_service_ = guidance_service(
      "~/service/set_stop_guidance_enabled", &CampDiffusionAdapter::set_stop_guidance_enabled);
    centerline_guidance_service_ = guidance_service(
      "~/service/set_centerline_guidance_enabled",
      &CampDiffusionAdapter::set_centerline_guidance_enabled);
    planning_factor_enable_stop_ = read_parameter<bool>("planning_factor.enable_stop", false);
    planning_factor_enable_slowdown_ =
      read_parameter<bool>("planning_factor.enable_slowdown", false);
    planning_factor_config_.stop_velocity_threshold =
      read_parameter<double>("planning_factor.stop_velocity_threshold", 0.1);
    planning_factor_config_.stop_keep_duration_threshold =
      read_parameter<double>("planning_factor.stop_keep_duration_threshold", 1.0);
    planning_factor_config_.slowdown_accel_threshold =
      read_parameter<double>("planning_factor.slowdown_accel_threshold", -0.3);
    planning_factor_interface_ =
      std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
        this, "diffusion_planner");

    pool_pub_ = create_publisher<CampCandidatePool>("~/output/candidate_pool", 1);
    candidates_pub_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    turn_pub_ = create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators", 1);
    objects_pub_ = create_publisher<PredictedObjects>("~/output/predicted_objects", 1);
    accepted_pub_ = create_publisher<CampSelection>("~/output/accepted_selection", 1);

    odometry_sub_ = create_subscription<Odometry>(
      "~/input/odometry", 1,
      [this](Odometry::ConstSharedPtr message) { on_odometry(std::move(message)); });
    acceleration_sub_ = create_subscription<AccelWithCovarianceStamped>(
      "~/input/acceleration", 1, [this](AccelWithCovarianceStamped::ConstSharedPtr message) {
        acceleration_ = std::move(message);
      });
    objects_sub_ = create_subscription<TrackedObjects>(
      "~/input/tracked_objects", 1,
      [this](TrackedObjects::ConstSharedPtr message) { objects_ = std::move(message); });
    turns_sub_ = create_subscription<TurnIndicatorsReport>(
      "~/input/turn_indicators", 1,
      [this](TurnIndicatorsReport::ConstSharedPtr message) { turns_ = std::move(message); });
    signals_sub_ = create_subscription<TrafficLightGroupArray>(
      "~/input/traffic_signals", 1, [this](TrafficLightGroupArray::ConstSharedPtr message) {
        traffic_signals_.push_back(std::move(message));
      });
    route_sub_ = create_subscription<LaneletRoute>(
      "~/input/route", rclcpp::QoS(1).transient_local(),
      [this](LaneletRoute::ConstSharedPtr message) {
        try {
          if (route_ && *route_ != *message) {
            reset_for_boundary("route changed");
          }
          route_ = std::move(message);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Route reset failed: %s", error.what());
        }
      });
    map_sub_ = create_subscription<LaneletMapBin>(
      "~/input/vector_map", rclcpp::QoS(1).transient_local(),
      [this](LaneletMapBin::ConstSharedPtr message) { on_map(*message); });
    selection_sub_ = create_subscription<CampSelection>(
      "~/input/selection", 1,
      [this](CampSelection::ConstSharedPtr message) { on_selection(*message); });
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(params.planning_frequency_hz).period(),
      [this] { on_timer(); });
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr guidance_service(
    const std::string & name, void (CampDiffusionAdapter::*setter)(bool))
  {
    return create_service<std_srvs::srv::SetBool>(
      name, [this, setter](
              std_srvs::srv::SetBool::Request::SharedPtr request,
              std_srvs::srv::SetBool::Response::SharedPtr response) {
        try {
          (adapter_.get()->*setter)(request->data);
          response->success = true;
          response->message = request->data ? "enabled" : "disabled";
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
        }
      });
  }

  template <typename T>
  T read_parameter(const std::string & name, const T & default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  diffusion_planner::DiffusionPlannerParams read_planner_params()
  {
    diffusion_planner::DiffusionPlannerParams params{};
    params.model_type = read_parameter<std::string>("model.type", "single_step");
    params.base_model_directory = read_parameter<std::string>("model.base_model_directory", "");
    params.args_filename =
      read_parameter<std::string>("model.args_filename", "diffusion_planner.param.json");
    params.single_step_model_filename = read_parameter<std::string>(
      "model.single_step_model.onnx_model_filename", "diffusion_planner.onnx");
    params.encoder_model_filename = read_parameter<std::string>(
      "model.multi_step_model.encoder_onnx_model_filename", "diffusion_planner_encoder.onnx");
    params.decoder_model_filename = read_parameter<std::string>(
      "model.multi_step_model.decoder_onnx_model_filename", "diffusion_planner_decoder.onnx");
    params.turn_indicator_model_filename = read_parameter<std::string>(
      "model.multi_step_model.turn_indicator_onnx_model_filename",
      "diffusion_planner_turn_indicator.onnx");
    params.dpm_solver_steps = read_parameter<int>("model.multi_step_model.dpm_solver_steps", 10);
    params.backend = read_parameter<std::string>("model.backend", "tensorrt");
    params.trt_precision = read_parameter<std::string>("model.precision", "fp32");
    params.use_cuda_graph = read_parameter<bool>("model.use_cuda_graph", true);
    params.plugins_path = read_parameter<std::string>("plugins_path", "");
    params.build_only = read_parameter<bool>("build_only", false);
    params.planning_frequency_hz = read_parameter<double>("planning_frequency_hz", 10.0);
    params.ignore_neighbors = read_parameter<bool>("ignore_neighbors", false);
    params.remap_unsupported_objects_to_pedestrian =
      read_parameter<bool>("remap_unsupported_objects_to_pedestrian", false);
    params.traffic_light_group_msg_timeout_seconds =
      read_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
    params.batch_size = read_parameter<int>("batch_size", 8);
    params.temperature_list =
      read_parameter<std::vector<double>>("temperature", {0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    params.velocity_smoothing_window = read_parameter<int64_t>("velocity_smoothing_window", 8);
    params.stopping_threshold = read_parameter<double>("stopping_threshold", 0.3);
    params.turn_indicator_keep_offset =
      static_cast<float>(read_parameter<double>("turn_indicator_keep_offset", -1.25));
    params.turn_indicator_hold_duration =
      read_parameter<double>("turn_indicator_hold_duration", 0.0);
    params.shift_x = read_parameter<bool>("shift_x", false);
    params.delay_step = read_parameter<int64_t>("delay_step", 0);
    params.line_string_max_step_m = read_parameter<double>("line_string_max_step_m", 5.0);
    params.use_time_interpolation = read_parameter<bool>("use_time_interpolation", false);
    params.object_motion_resampling.enable =
      read_parameter<bool>("object_motion_resampling.enable", true);
    params.object_motion_resampling.max_extrapolation_time =
      read_parameter<double>("object_motion_resampling.max_extrapolation_time", 0.5);
    params.ego_snap_to_prev_trajectory.enable =
      read_parameter<bool>("ego_snap_to_prev_trajectory.enable", false);
    params.ego_snap_to_prev_trajectory.max_position_error_m =
      read_parameter<double>("ego_snap_to_prev_trajectory.max_position_error_m", 0.3);
    params.ego_snap_to_prev_trajectory.max_yaw_error_deg =
      read_parameter<double>("ego_snap_to_prev_trajectory.max_yaw_error_deg", 5.0);
    params.ego_snap_to_prev_trajectory.max_search_segment_count =
      read_parameter<int64_t>("ego_snap_to_prev_trajectory.max_search_segment_count", 5);
    params.start_guidance_reference_distance_m =
      read_parameter<double>("guidance.start_guidance.reference_distance_m", 10.0);
    params.start_guidance_max_scale =
      read_parameter<double>("guidance.start_guidance.max_scale", 30.0);
    params.stop_guidance_stop_acceleration_mps2 =
      read_parameter<double>("guidance.stop_guidance.stop_acceleration_mps2", 1.0);
    params.centerline_guidance_start_time_s =
      read_parameter<double>("guidance.centerline_guidance.start_time_s", 2.0);
    return params;
  }

  void reset_for_boundary(const char * reason)
  {
    faulted_ = true;
    adapter_->reset_episode();
    pending_pool_id_.reset();
    last_frame_time_.reset();
    traffic_signals_.clear();
    faulted_ = false;
    RCLCPP_INFO(get_logger(), "Reset CAMP/DP episode: %s", reason);
  }

  void on_odometry(Odometry::ConstSharedPtr message)
  {
    const auto previous = odometry_;
    odometry_ = std::move(message);
    try {
      const rclcpp::Time frame_time(odometry_->header.stamp);
      if (
        (previous && frame_time < rclcpp::Time(previous->header.stamp)) ||
        (last_frame_time_ && frame_time < *last_frame_time_)) {
        // Invalidate the old pool now: selection can arrive before the next timer,
        // even if another odometry message restores the old timestamp first.
        reset_for_boundary("odometry clock moved backwards");
      }
    } catch (const std::exception & error) {
      faulted_ = true;
      RCLCPP_ERROR(get_logger(), "Odometry reset failed: %s", error.what());
    }
  }

  void on_map(const LaneletMapBin & message)
  {
    try {
      const auto loaded_map =
        autoware::experimental::lanelet2_utils::from_autoware_map_msgs(message);
      faulted_ = true;
      adapter_->set_map(loaded_map);
      pending_pool_id_.reset();
      last_frame_time_.reset();
      traffic_signals_.clear();
      faulted_ = false;
      RCLCPP_INFO(get_logger(), "Loaded map and reset CAMP/DP episode");
    } catch (const std::exception & error) {
      faulted_ = true;
      RCLCPP_ERROR(get_logger(), "Map setup failed: %s", error.what());
    }
  }

  void on_timer()
  {
    if (
      faulted_ || !adapter_->ready() || !odometry_ || !acceleration_ || !objects_ || !turns_ ||
      !route_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for loaded model/map and odometry, acceleration, objects, route, turn reports");
      return;
    }
    if (pending_pool_id_) {
      // Inference can exceed the timer period. Let the executor receive the
      // selector's answer before prepare() replaces the matching original pool.
      if (std::chrono::steady_clock::now() < selection_deadline_) return;
      RCLCPP_WARN(
        get_logger(), "CAMP selection timed out for pool %llu",
        static_cast<unsigned long long>(*pending_pool_id_));
      pending_pool_id_.reset();
    }
    try {
      const rclcpp::Time frame_time(odometry_->header.stamp);
      if (last_frame_time_ && frame_time < *last_frame_time_) {
        reset_for_boundary("odometry clock moved backwards");
      }
      // Retain signal updates while inputs or the matching selection are pending.
      auto signals =
        std::exchange(traffic_signals_, std::vector<TrafficLightGroupArray::ConstSharedPtr>{});
      const auto frame = adapter_->core().create_frame_context(
        odometry_, acceleration_, objects_, signals, turns_, route_, now());
      if (!frame) {
        return;
      }
      last_frame_time_ = frame->frame_time;
      const auto prepared = adapter_->prepare(*frame, frame->frame_time, generator_uuid_);
      pending_pool_id_ = prepared.pool.pool_id;
      selection_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      pool_pub_->publish(prepared.pool);
      candidates_pub_->publish(prepared.pool.candidates);
    } catch (const std::exception & error) {
      // Upstream may already have committed its generator history; no rollback is claimed here.
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAMP preparation failed: %s", error.what());
    }
  }

  void on_selection(const CampSelection & message)
  {
    if (faulted_ || !pending_pool_id_ || message.pool_id != *pending_pool_id_) {
      return;
    }
    try {
      const auto selected = adapter_->select(message);
      if (!selected) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "CAMP rejected stale or inconsistent selection");
        return;
      }
      trajectory_pub_->publish(selected->trajectory);
      turn_pub_->publish(selected->turn_indicators_command);
      objects_pub_->publish(selected->predicted_objects);
      publish_planning_factor(selected->trajectory);
      accepted_pub_->publish(selected->selection);
      // This records successful publication of the selected plan, not a controller execution ack.
      if (!adapter_->confirm_published_selection(message.header, message.selected_index)) {
        throw std::runtime_error("published selection could not be retained for continuity");
      }
      pending_pool_id_.reset();
      RCLCPP_DEBUG(get_logger(), "Published CAMP original candidate %u", message.selected_index);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAMP output failed: %s", error.what());
    }
  }

  void publish_planning_factor(const Trajectory & trajectory)
  {
    using autoware_internal_planning_msgs::msg::PlanningFactor;
    using autoware_internal_planning_msgs::msg::SafetyFactorArray;
    const auto & points = trajectory.points;
    const auto detection =
      diffusion_planner::detect_planning_factors(points, planning_factor_config_);
    if (planning_factor_enable_stop_ && detection.stop) {
      const auto & stop = *detection.stop;
      planning_factor_interface_->add(
        points, stop.ego_pose, stop.stop_pose, PlanningFactor::STOP, SafetyFactorArray{});
    }
    if (planning_factor_enable_slowdown_ && detection.slowdown) {
      const auto & slowdown = *detection.slowdown;
      planning_factor_interface_->add(
        points, slowdown.ego_pose, slowdown.start_pose, slowdown.end_pose,
        PlanningFactor::SLOW_DOWN, SafetyFactorArray{}, true, slowdown.start_velocity,
        slowdown.end_velocity);
    }
    planning_factor_interface_->publish();
  }

  std::unique_ptr<CampDiffusionAdapter> adapter_;
  unique_identifier_msgs::msg::UUID generator_uuid_;
  std::optional<rclcpp::Time> last_frame_time_;
  std::optional<std::uint64_t> pending_pool_id_;
  std::chrono::steady_clock::time_point selection_deadline_;
  bool faulted_{false};
  Odometry::ConstSharedPtr odometry_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_;
  TrackedObjects::ConstSharedPtr objects_;
  TurnIndicatorsReport::ConstSharedPtr turns_;
  LaneletRoute::ConstSharedPtr route_;
  std::vector<TrafficLightGroupArray::ConstSharedPtr> traffic_signals_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<TurnIndicatorsReport>::SharedPtr turns_sub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr signals_sub_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<CampSelection>::SharedPtr selection_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_guidance_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stop_guidance_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr centerline_guidance_service_;
  bool planning_factor_enable_stop_{false};
  bool planning_factor_enable_slowdown_{false};
  diffusion_planner::PlanningFactorDetectionConfig planning_factor_config_;
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  rclcpp::Publisher<CampCandidatePool>::SharedPtr pool_pub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr candidates_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_pub_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr objects_pub_;
  rclcpp::Publisher<CampSelection>::SharedPtr accepted_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace autoware::camp_diffusion_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // Single-threaded callbacks serialize generation, episode resets and selection acceptance.
    rclcpp::spin(std::make_shared<autoware::camp_diffusion_adapter::CampDiffusionAdapterNode>(
      rclcpp::NodeOptions{}));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("camp_diffusion_adapter"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
