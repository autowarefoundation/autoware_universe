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

#ifndef AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_CORE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_CORE_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/inference/guidance/centerline_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/start_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/stop_guidance.hpp"
#include "autoware/diffusion_planner/inference/inference.hpp"
#include "autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"

#include <Eigen/Dense>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{

using autoware::diffusion_planner::AgentData;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using preprocess::TrafficSignalStamped;
using std_msgs::msg::Float32MultiArray;
using unique_identifier_msgs::msg::UUID;
using utils::ObservationNormalization;
using utils::StateNormalization;
using InputDataMap = std::unordered_map<std::string, std::vector<float>>;
using AgentPoses = std::vector<std::vector<std::vector<Eigen::Matrix4d>>>;

struct VehicleSpec
{
  double wheel_base;
  double vehicle_length;
  double vehicle_width;
  double base_link_to_center;

  explicit VehicleSpec(const VehicleInfo & info)
  : wheel_base(info.wheel_base_m),
    vehicle_length(info.front_overhang_m + info.wheel_base_m + info.rear_overhang_m),
    vehicle_width(info.left_overhang_m + info.wheel_tread_m + info.right_overhang_m),
    base_link_to_center((info.front_overhang_m + info.wheel_base_m - info.rear_overhang_m) / 2.0)
  {
  }
};

struct PlannerOutput
{
  Trajectory trajectory;
  CandidateTrajectories candidate_trajectories;
  PredictedObjects predicted_objects;
  TurnIndicatorsCommand turn_indicators_command;
  Float32MultiArray denoising_steps;
  std::unordered_map<std::string, std::vector<bool>> guidance_triggered;
};

struct FrameContext
{
  nav_msgs::msg::Odometry ego_kinematic_state;
  geometry_msgs::msg::AccelWithCovarianceStamped ego_acceleration;
  Eigen::Matrix4d ego_to_map_transform;
  std::vector<AgentHistory> ego_centric_neighbor_histories;
  rclcpp::Time frame_time;
};

struct DiffusionPlannerParams
{
  std::string model_type;
  std::string single_step_model_path;
  std::string encoder_model_path;
  std::string decoder_model_path;
  std::string turn_indicator_model_path;
  std::string args_path;
  std::string plugins_path;
  std::string backend;
  std::string trt_precision;
  bool use_cuda_graph;
  bool build_only;
  double planning_frequency_hz;
  bool ignore_neighbors;
  double traffic_light_group_msg_timeout_seconds;
  int batch_size;
  std::vector<double> temperature_list;
  int64_t velocity_smoothing_window;
  double stopping_threshold;
  float turn_indicator_keep_offset;
  double turn_indicator_hold_duration;
  bool shift_x;
  int64_t delay_step;
  double line_string_max_step_m;
  bool use_time_interpolation;
  HistoryAlignmentParams object_history;
  int dpm_solver_steps;
  double start_guidance_reference_distance_m;
  double start_guidance_max_scale;
  double stop_guidance_stop_acceleration_mps2;
  double centerline_guidance_start_time_s;
};

/**
 * @class DiffusionPlannerCore
 * @brief Core logic class for the diffusion-based trajectory planner.
 *
 * This class contains all the business logic for trajectory planning,
 * independent of ROS infrastructure. It handles:
 * - Frame context creation from sensor and environment data
 * - Input data preparation for inference
 * - Model inference execution
 * - Data normalization
 *
 * By separating this from the ROS node, we enable:
 * - Direct testing with rosbag data without ROS runtime
 * - Deterministic and reproducible tests
 * - Better unit testing capabilities
 */
class DiffusionPlannerCore
{
public:
  explicit DiffusionPlannerCore(
    const DiffusionPlannerParams & params, const VehicleInfo & vehicle_info);

  /**
   * @brief Load TensorRT model and normalization statistics.
   *
   * @throws std::runtime_error if args_path or model paths are invalid, if the
   *         model version is incompatible, or if TensorRT engine setup fails.
   */
  void load_model();

  /**
   * @brief Update parameters without losing internal state.
   *
   * @param params New parameters to apply
   */
  void update_params(const DiffusionPlannerParams & params);

  /**
   * @brief Prepare frame context for inference.
   *
   * @param ego_kinematic_state Current ego vehicle odometry
   * @param ego_acceleration Current ego vehicle acceleration
   * @param objects Tracked objects in the scene
   * @param traffic_signals Traffic signal information
   * @param turn_indicators Current turn indicator state
   * @param route_ptr Route information
   * @param current_time Current timestamp
   * @return FrameContext containing preprocessed data, or nullopt if data is incomplete
   */
  std::optional<FrameContext> create_frame_context(
    const std::shared_ptr<const Odometry> & ego_kinematic_state,
    const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
    const std::shared_ptr<const TrackedObjects> & objects,
    const std::vector<
      std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
      traffic_signals,
    const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
    const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time);

  /**
   * @brief Build model input tensors from frame context.
   *
   * @param frame_context Preprocessed frame context
   * @return Map of input data for the model
   */
  InputDataMap create_input_data(const FrameContext & frame_context);

  /**
   * @brief Set the lanelet map context.
   *
   * @param lanelet_map_ptr Shared pointer to lanelet map
   */
  void set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr);

  /**
   * @brief Check if the model is loaded.
   *
   * @return true if model is loaded, false otherwise
   */
  bool is_model_loaded() const { return diffusion_planner_inference_ != nullptr; }

  /**
   * @brief Check if the map is loaded.
   *
   * @return true if map is loaded, false otherwise
   */
  bool is_map_loaded() const { return lane_segment_context_ != nullptr; }

  /**
   * @brief Enable or disable start guidance.
   *
   * @param enabled Whether start guidance should be enabled
   */
  void set_start_guidance_enabled(bool enabled);

  /**
   * @brief Enable or disable stop guidance.
   *
   * @param enabled Whether stop guidance should be enabled
   */
  void set_stop_guidance_enabled(bool enabled);

  /**
   * @brief Enable or disable centerline guidance.
   *
   * @param enabled Whether centerline guidance should be enabled
   */
  void set_centerline_guidance_enabled(bool enabled);

  /**
   * @brief Get the observation normalization.
   *
   * @return Reference to observation normalization
   */
  const ObservationNormalization & get_observation_normalization() const
  {
    return observation_normalization_;
  }

  /**
   * @brief Run inference on the input data.
   *
   * @param input_data_map Input data for inference
   * @return Inference result with predictions, turn indicator logits, and denoising steps
   */
  InferenceResult run_inference(const InputDataMap & input_data_map);

  /**
   * @brief Create all planner output messages from raw inference outputs.
   *
   * Parses raw predictions, creates ego trajectory (batch 0), candidate trajectories
   * for all batches, predicted objects for neighbor agents, and turn indicator command.
   *
   * @param inference_output Successful inference output.
   * @param frame_context Context of the current frame.
   * @param timestamp The ROS time stamp for the messages.
   * @param generator_uuid The unique identifier for the planner instance.
   * @return PlannerOutput containing all output messages.
   */
  PlannerOutput create_planner_output(
    const InferenceOutput & inference_output, const FrameContext & frame_context,
    const rclcpp::Time & timestamp, const UUID & generator_uuid);

  /**
   * @brief Get the first traffic light on the route for debugging.
   *
   * @param frame_context Context of the current frame
   * @return Traffic light group message
   */
  autoware_perception_msgs::msg::TrafficLightGroup get_first_traffic_light_on_route(
    const FrameContext & frame_context) const;

  /**
   * @brief Count valid elements in input data for diagnostics.
   *
   * @param input_data_map Input data map
   * @param data_key Key for the data to count (e.g., "lanes", "route_lanes", "polygons")
   * @return Count of valid elements
   */
  int64_t count_valid_elements(
    const InputDataMap & input_data_map, const std::string & data_key) const;

  /**
   * @brief Get current route pointer.
   *
   * @return Shared pointer to current route
   */
  const LaneletRoute::ConstSharedPtr & get_route() const { return route_ptr_; }

private:
  // Parameters
  DiffusionPlannerParams params_;
  VehicleSpec vehicle_spec_;

  ObservationNormalization observation_normalization_;
  StateNormalization state_normalization_;

  // Inference engine
  std::unique_ptr<Inference> diffusion_planner_inference_{nullptr};
  std::shared_ptr<StartGuidance> start_guidance_{nullptr};
  std::shared_ptr<StopGuidance> stop_guidance_{nullptr};
  std::shared_ptr<CenterlineGuidance> centerline_guidance_{nullptr};
  bool start_guidance_enabled_{false};
  bool stop_guidance_enabled_{false};
  bool centerline_guidance_enabled_{false};

  // Postprocessing
  std::vector<postprocess::TurnIndicatorManager> turn_indicator_managers_;

  /**
   * @brief Resize the per-trajectory turn indicator managers to the current batch size and
   *        apply the latest hold duration / keep offset parameters to each of them.
   */
  void sync_turn_indicator_managers();

  // History data
  std::deque<nav_msgs::msg::Odometry> ego_history_;
  std::deque<TurnIndicatorsReport> turn_indicators_history_;
  AgentData agent_data_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_;
  std::vector<std::vector<std::vector<Eigen::Matrix4d>>> last_agent_poses_map_;

  // Lanelet map
  LaneletRoute::ConstSharedPtr route_ptr_;
  std::unique_ptr<preprocess::LaneSegmentContext> lane_segment_context_;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__DIFFUSION_PLANNER_CORE_HPP_
