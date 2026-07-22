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

#include "autoware/tensorrt_oneplanner/oneplanner_node.hpp"

#include "autoware/tensorrt_oneplanner/detail.hpp"
#include "autoware/tensorrt_oneplanner/dimensions.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/diffusion_planner/dimensions.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;

struct SegmentTensorSummary
{
  std::size_t active_segments{0};
  std::size_t active_points{0};
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  double max_forward_x{0.0};
  double min_forward_x{0.0};
  double mean_width{0.0};
  double max_width{0.0};
  std::size_t tl_green_segments{0};
  std::size_t tl_yellow_segments{0};
  std::size_t tl_red_segments{0};
  std::size_t tl_white_segments{0};
  std::size_t tl_none_segments{0};
  std::string head_points;
};

SegmentTensorSummary summarize_segment_tensor(
  const std::vector<float> & tensor, const int64_t max_segments)
{
  using autoware::diffusion_planner::LB_X;
  using autoware::diffusion_planner::LB_Y;
  using autoware::diffusion_planner::LINE_TYPE_LEFT_START;
  using autoware::diffusion_planner::POINTS_PER_SEGMENT;
  using autoware::diffusion_planner::RB_X;
  using autoware::diffusion_planner::RB_Y;
  using autoware::diffusion_planner::SEGMENT_POINT_DIM;
  using autoware::diffusion_planner::TRAFFIC_LIGHT_GREEN;
  using autoware::diffusion_planner::TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT;
  using autoware::diffusion_planner::TRAFFIC_LIGHT_RED;
  using autoware::diffusion_planner::TRAFFIC_LIGHT_WHITE;
  using autoware::diffusion_planner::TRAFFIC_LIGHT_YELLOW;
  using autoware::diffusion_planner::X;
  using autoware::diffusion_planner::Y;

  SegmentTensorSummary summary;
  bool have_point = false;
  double width_sum = 0.0;
  std::ostringstream head;
  head << std::fixed << std::setprecision(2);

  for (int64_t segment = 0; segment < max_segments; ++segment) {
    bool segment_active = false;
    bool emitted_segment_head = false;
    int active_points_in_segment = 0;

    for (int64_t point = 0; point < POINTS_PER_SEGMENT; ++point) {
      const std::size_t base =
        static_cast<std::size_t>((segment * POINTS_PER_SEGMENT + point) * SEGMENT_POINT_DIM);
      if (base + SEGMENT_POINT_DIM > tensor.size()) {
        break;
      }

      bool point_active = false;
      for (int64_t dim = 0; dim < SEGMENT_POINT_DIM; ++dim) {
        if (std::abs(tensor[base + dim]) > std::numeric_limits<float>::epsilon()) {
          point_active = true;
          break;
        }
      }
      if (!point_active) {
        continue;
      }

      const double x = tensor[base + X];
      const double y = tensor[base + Y];
      const double lb_x = tensor[base + LB_X];
      const double lb_y = tensor[base + LB_Y];
      const double rb_x = tensor[base + RB_X];
      const double rb_y = tensor[base + RB_Y];
      const double width = std::hypot(lb_x - rb_x, lb_y - rb_y);

      if (!have_point) {
        summary.min_x = summary.max_x = x;
        summary.min_y = summary.max_y = y;
        summary.min_forward_x = summary.max_forward_x = x;
        have_point = true;
      } else {
        summary.min_x = std::min(summary.min_x, x);
        summary.max_x = std::max(summary.max_x, x);
        summary.min_y = std::min(summary.min_y, y);
        summary.max_y = std::max(summary.max_y, y);
        summary.min_forward_x = std::min(summary.min_forward_x, x);
        summary.max_forward_x = std::max(summary.max_forward_x, x);
      }

      summary.active_points++;
      width_sum += width;
      summary.max_width = std::max(summary.max_width, width);
      segment_active = true;
      ++active_points_in_segment;

      if (!emitted_segment_head && point < 3) {
        head << " s" << segment << "p" << point << "=(" << x << ", " << y << ")";
        if (point == 2) {
          emitted_segment_head = true;
        }
      }
    }

    if (!segment_active) {
      continue;
    }

    summary.active_segments++;
    const std::size_t tl_base = static_cast<std::size_t>(segment * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
    if (tl_base + LINE_TYPE_LEFT_START <= tensor.size()) {
      const double green = tensor[tl_base + TRAFFIC_LIGHT_GREEN];
      const double yellow = tensor[tl_base + TRAFFIC_LIGHT_YELLOW];
      const double red = tensor[tl_base + TRAFFIC_LIGHT_RED];
      const double white = tensor[tl_base + TRAFFIC_LIGHT_WHITE];
      const double none = tensor[tl_base + TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT];
      if (green > 0.5) {
        summary.tl_green_segments++;
      } else if (yellow > 0.5) {
        summary.tl_yellow_segments++;
      } else if (red > 0.5) {
        summary.tl_red_segments++;
      } else if (white > 0.5) {
        summary.tl_white_segments++;
      } else if (none > 0.5) {
        summary.tl_none_segments++;
      }
    }
  }

  if (summary.active_points > 0) {
    summary.mean_width = width_sum / static_cast<double>(summary.active_points);
  }
  summary.head_points = head.str();
  return summary;
}
}  // namespace

OnePlannerNode::OnePlannerNode(const rclcpp::NodeOptions & options)
: Node("tensorrt_oneplanner", options),
  tf_buffer_(this->get_clock()),
  generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  core_ = std::make_unique<OnePlannerCore>(params_, vehicle_info_);
  core_->load_normalization();

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  load_models();

  // Publishers
  pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", rclcpp::QoS(1));
  pub_trajectories_ =
    this->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "~/output/trajectories", rclcpp::QoS(1));
  pub_turn_indicators_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "~/output/turn_indicators", rclcpp::QoS(1));
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", rclcpp::QoS(1));

  diagnostics_inference_ = std::make_unique<DiagnosticsInterface>(this, "oneplanner_inference");

  // Subscriptions
  sub_map_ = this->create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&OnePlannerNode::on_map, this, std::placeholders::_1));
  cloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&OnePlannerNode::on_pointcloud, this, std::placeholders::_1));

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engines were built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void OnePlannerNode::set_up_params()
{
  // BEV encoder (lidar branch) params
  bev_params_.onnx_path = this->declare_parameter<std::string>("bev_encoder.onnx_path", "");
  bev_params_.engine_path = this->declare_parameter<std::string>("bev_encoder.engine_path", "");
  bev_params_.trt_precision =
    this->declare_parameter<std::string>("bev_encoder.trt_precision", "fp16");
  bev_params_.densification_world_frame_id =
    this->declare_parameter<std::string>("densification_world_frame_id", "map");
  bev_params_.densification_num_past_frames =
    this->declare_parameter<int64_t>("densification_num_past_frames", 1);
  bev_params_.cloud_capacity = this->declare_parameter<int64_t>("cloud_capacity", 2000000);
  bev_params_.max_points_per_voxel = this->declare_parameter<int64_t>("max_points_per_voxel", 10);
  bev_params_.voxels_num = this->declare_parameter<std::vector<int64_t>>("voxels_num");
  const auto point_cloud_range_double =
    this->declare_parameter<std::vector<double>>("point_cloud_range");
  bev_params_.point_cloud_range =
    std::vector<float>(point_cloud_range_double.begin(), point_cloud_range_double.end());
  const auto voxel_size_double = this->declare_parameter<std::vector<double>>("voxel_size");
  bev_params_.voxel_size = std::vector<float>(voxel_size_double.begin(), voxel_size_double.end());
  bev_params_.out_size_factor = this->declare_parameter<int64_t>("out_size_factor", 8);
  bev_params_.bev_feature_channels = this->declare_parameter<int64_t>("bev_feature_channels", 512);
  bev_params_.use_intensity = this->declare_parameter<bool>("use_intensity", true);

  // Planner head params
  planner_onnx_path_ = this->declare_parameter<std::string>("planner.onnx_path", "");
  plugins_path_ = this->declare_parameter<std::string>("plugins_path", "");
  bev_params_.plugins_path = plugins_path_;

  params_.args_path = this->declare_parameter<std::string>("args_path", "");
  params_.temperature = this->declare_parameter<double>("temperature", 0.0);
  params_.enable_warm_start = this->declare_parameter<bool>("enable_warm_start", true);
  params_.velocity_smoothing_window =
    this->declare_parameter<int64_t>("velocity_smoothing_window", 8);
  params_.stopping_threshold = this->declare_parameter<double>("stopping_threshold", 0.3);
  params_.turn_indicator_keep_offset =
    this->declare_parameter<float>("turn_indicator_keep_offset", -1.25f);
  params_.turn_indicator_hold_duration =
    this->declare_parameter<double>("turn_indicator_hold_duration", 1.0);
  params_.shift_x = this->declare_parameter<bool>("shift_x", false);
  params_.delay_step = this->declare_parameter<int64_t>("delay_step", 0);
  params_.line_string_max_step_m = this->declare_parameter<double>("line_string_max_step_m", 5.0);
  params_.use_time_interpolation = this->declare_parameter<bool>("use_time_interpolation", false);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
  debug_tensor_logging_ = this->declare_parameter<bool>("debug_tensor_logging", false);
}

void OnePlannerNode::load_models()
{
  bev_encoder_ = std::make_unique<BevEncoder>(bev_params_, stream_);
  planner_inference_ = std::make_unique<PlannerInference>(
    planner_onnx_path_, plugins_path_, /*batch_size=*/1, bev_encoder_->bev_feature_d(),
    bev_encoder_->bev_channels(), bev_encoder_->bev_size(), stream_);
}

void OnePlannerNode::on_map(const LaneletMapBin::ConstSharedPtr map_msg)
{
  const auto lanelet_map_ptr =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  core_->set_map(lanelet_map_ptr);
}

void OnePlannerNode::log_debug_tensors(
  const InputDataMap & input_data_map, const std::vector<float> & prediction,
  const autoware_planning_msgs::msg::Trajectory & published_trajectory,
  const nav_msgs::msg::Odometry & ego_kinematic_state, int64_t num_input_points)
{
  constexpr double dt = 0.1;  // OnePlanner prediction time step [s]
  const int64_t output_t = autoware::diffusion_planner::OUTPUT_T;
  const int64_t pose_dim = autoware::diffusion_planner::POSE_DIM;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);

  // --- 1. Ground-truth current velocity from odometry (raw, un-normalized) ---
  const auto & lin = ego_kinematic_state.twist.twist.linear;
  const double odom_speed = std::hypot(lin.x, lin.y);
  ss << "\n  [odom] vx=" << lin.x << " vy=" << lin.y << " speed=" << odom_speed
     << " m/s | input_points=" << num_input_points;

  // --- 2. Normalized ego_current_state fed to the model ---
  //     layout: [x, y, cos, sin, vx, vy, ax, ay, steer, yaw_rate]
  const auto ecs_it = input_data_map.find("ego_current_state");
  if (ecs_it != input_data_map.end() && ecs_it->second.size() >= 10) {
    const auto & e = ecs_it->second;
    ss << "\n  [ego_current_state normalized] x=" << e[0] << " y=" << e[1] << " cos=" << e[2]
       << " sin=" << e[3] << " vx=" << e[4] << " vy=" << e[5] << " ax=" << e[6] << " ay=" << e[7]
       << " steer=" << e[8] << " yaw_rate=" << e[9];
  }

  // --- 2b. Warm-started sampled_trajectories fed to the model ---
  {
    const auto delay_it = input_data_map.find("delay");
    const auto sampled_it = input_data_map.find("sampled_trajectories");
    if (
      delay_it != input_data_map.end() && !delay_it->second.empty() &&
      sampled_it != input_data_map.end() && !sampled_it->second.empty())
    {
      const auto & sampled = sampled_it->second;
      constexpr int64_t sampled_steps = autoware::diffusion_planner::OUTPUT_T + 1;
      constexpr int64_t pose_dim_local = autoware::diffusion_planner::POSE_DIM;
      const int64_t available_steps =
        static_cast<int64_t>(sampled.size()) / pose_dim_local;
      const int64_t steps = std::min(sampled_steps, available_steps);
      size_t active_steps = 0;
      size_t duplicate_steps = 0;
      int64_t first_active_step = -1;
      int64_t last_active_step = -1;
      float min_x_norm = std::numeric_limits<float>::max();
      float max_x_norm = std::numeric_limits<float>::lowest();
      float min_y_norm = std::numeric_limits<float>::max();
      float max_y_norm = std::numeric_limits<float>::lowest();
      std::ostringstream sampled_head;
      sampled_head << std::fixed << std::setprecision(2);
      float prev_x_raw = 0.0f;
      float prev_y_raw = 0.0f;
      bool have_prev = false;
      for (int64_t t = 0; t < steps; ++t) {
        const size_t base = static_cast<size_t>(t * pose_dim_local);
        const float x_norm = sampled[base + 0];
        const float y_norm = sampled[base + 1];
        const float cos_yaw = sampled[base + 2];
        const float sin_yaw = sampled[base + 3];
        const float x_raw = x_norm * detail::kEgoPositionStd + detail::kEgoPositionXMean;
        const float y_raw = y_norm * detail::kEgoPositionYStd + detail::kEgoPositionYMean;
        const bool active =
          std::abs(x_norm) > std::numeric_limits<float>::epsilon() ||
          std::abs(y_norm) > std::numeric_limits<float>::epsilon() ||
          std::abs(cos_yaw) > std::numeric_limits<float>::epsilon() ||
          std::abs(sin_yaw) > std::numeric_limits<float>::epsilon();
        if (active) {
          ++active_steps;
          if (first_active_step < 0) {
            first_active_step = t;
          }
          last_active_step = t;
          min_x_norm = std::min(min_x_norm, x_norm);
          max_x_norm = std::max(max_x_norm, x_norm);
          min_y_norm = std::min(min_y_norm, y_norm);
          max_y_norm = std::max(max_y_norm, y_norm);
          if (have_prev && std::hypot(x_raw - prev_x_raw, y_raw - prev_y_raw) < 0.01f) {
            ++duplicate_steps;
          }
          prev_x_raw = x_raw;
          prev_y_raw = y_raw;
          have_prev = true;
        }
        if (t < 5) {
          sampled_head << "[" << t << ":x=" << x_norm << " y=" << y_norm << " -> raw=(" << x_raw
                       << ", " << y_raw << ") c=" << cos_yaw << " s=" << sin_yaw << "] ";
        }
      }
      ss << "\n  [sampled_trajectories] delay_step=" << delay_it->second.front()
         << " active_steps=" << active_steps << "/" << steps
         << " first_active=" << first_active_step << " last_active=" << last_active_step
         << " duplicate_step_pairs=" << duplicate_steps;
      if (delay_it->second.front() <= 0.0f && active_steps <= 1) {
        ss << " | warm_start=weak(delay_step<=0 and <=1 active step)";
      } else if (active_steps > 0 && duplicate_steps + 1 >= active_steps) {
        ss << " | warm_start=collapsed(trajectory steps nearly identical)";
      }
      if (active_steps > 0) {
        ss << " | x_norm=[" << min_x_norm << ", " << max_x_norm << "]"
           << " y_norm=[" << min_y_norm << ", " << max_y_norm << "]"
           << " x_raw=[" << (min_x_norm * detail::kEgoPositionStd + detail::kEgoPositionXMean)
           << ", " << (max_x_norm * detail::kEgoPositionStd + detail::kEgoPositionXMean) << "]"
           << " y_raw=[" << (min_y_norm * detail::kEgoPositionYStd + detail::kEgoPositionYMean)
           << ", " << (max_y_norm * detail::kEgoPositionYStd + detail::kEgoPositionYMean)
           << "]";
      }
      ss << "\n    head " << sampled_head.str();
    } else {
      ss << "\n  [sampled_trajectories] <absent>";
    }
  }

  // --- 3. Speed-limit tensors: how many segments carry a limit and how large ---
  const auto & norm_map = core_->get_normalization_map();
  const auto summarize_speed_limit = [&](const std::string & key) {
    const auto it = input_data_map.find(key);
    if (it == input_data_map.end() || it->second.empty()) {
      ss << "\n  [" << key << "] <absent>";
      return;
    }
    // Recover the raw m/s scale from the normalization std (mean is 0 for speed limits).
    float std_dev = 1.0f;
    const auto nm = norm_map.find(key);
    if (nm != norm_map.end() && !nm->second.second.empty()) {
      std_dev = nm->second.second[0];
    }
    size_t nonzero = 0;
    float max_norm = 0.0f;
    double sum_norm = 0.0;
    for (const float v : it->second) {
      if (std::abs(v) > std::numeric_limits<float>::epsilon()) {
        ++nonzero;
        sum_norm += v;
        max_norm = std::max(max_norm, v);
      }
    }
    const double mean_norm = nonzero > 0 ? sum_norm / static_cast<double>(nonzero) : 0.0;
    ss << "\n  [" << key << "] nonzero=" << nonzero << "/" << it->second.size()
       << " max=" << max_norm << " (~" << max_norm * std_dev << " m/s)"
       << " mean_nonzero=" << mean_norm << " (~" << mean_norm * std_dev << " m/s)";
  };
  summarize_speed_limit("lanes_speed_limit");
  summarize_speed_limit("route_lanes_speed_limit");

  const auto summarize_segment_family = [&](const std::string & key, const int64_t max_segments) {
    const auto it = input_data_map.find(key);
    if (it == input_data_map.end() || it->second.empty()) {
      ss << "\n  [" << key << "] <absent>";
      return;
    }
    const auto summary = summarize_segment_tensor(it->second, max_segments);
    ss << "\n  [" << key << "] active_segments=" << summary.active_segments << "/" << max_segments
       << " active_points=" << summary.active_points
       << " x_range=[" << summary.min_x << ", " << summary.max_x << "]"
       << " y_range=[" << summary.min_y << ", " << summary.max_y << "]"
       << " forward_x_range=[" << summary.min_forward_x << ", " << summary.max_forward_x << "]"
       << " mean_width=" << summary.mean_width << " max_width=" << summary.max_width
       << " tl(g/y/r/w/none)=" << summary.tl_green_segments << "/" << summary.tl_yellow_segments
       << "/" << summary.tl_red_segments << "/" << summary.tl_white_segments << "/"
       << summary.tl_none_segments;
    if (!summary.head_points.empty()) {
      ss << "\n    head" << summary.head_points;
    }
  };
  summarize_segment_family("lanes", autoware::diffusion_planner::NUM_SEGMENTS_IN_LANE);
  summarize_segment_family("route_lanes", autoware::diffusion_planner::NUM_SEGMENTS_IN_ROUTE);

  // --- 4. Raw model output speed (ego frame, before smoothing / force-stop) ---
  //     prediction is ego-only, layout [t][x, y, cos, sin] in meters.
  if (static_cast<int64_t>(prediction.size()) >= output_t * pose_dim) {
    double prev_x = 0.0;
    double prev_y = 0.0;
    double sum_v = 0.0;
    double max_v = 0.0;
    std::ostringstream head;
    head << std::fixed << std::setprecision(2);
    for (int64_t t = 0; t < output_t; ++t) {
      const double x = prediction[t * pose_dim + 0];
      const double y = prediction[t * pose_dim + 1];
      const double v = std::hypot(x - prev_x, y - prev_y) / dt;
      if (t < 10) head << v << " ";
      sum_v += v;
      max_v = std::max(max_v, v);
      prev_x = x;
      prev_y = y;
    }
    const double end_x = prediction[(output_t - 1) * pose_dim + 0];
    const double end_y = prediction[(output_t - 1) * pose_dim + 1];
    // A trajectory whose 8s endpoint is still ~at the ego origin is a degenerate "STOP"
    // output; anything travelling meaningfully forward is a "GO".
    const double endpoint_dist = std::hypot(end_x, end_y);
    const char * verdict = endpoint_dist < 1.0 ? "STOP" : "GO";
    ss << "\n  [model output] step_speed[0..9]= " << head.str()
       << "| mean=" << (sum_v / static_cast<double>(output_t)) << " max=" << max_v
       << " m/s | endpoint(ego)=(" << end_x << ", " << end_y << ") dist=" << endpoint_dist
       << " => " << verdict << " over " << output_t * dt << "s";
  }

  // --- 5. Published trajectory speed (AFTER create_ego_trajectory: velocity smoothing +
  //     force-stop). Comparing this against [model output] isolates whether OnePlanner's own
  //     postprocessing (force-stop/smoothing) is what kills the speed, versus a downstream
  //     node (trajectory_modifier / trajectory_optimizer / control). ---
  {
    const auto & pts = published_trajectory.points;
    double max_pub = 0.0;
    double sum_pub = 0.0;
    size_t n_zero = 0;
    std::ostringstream pub_head;
    pub_head << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < pts.size(); ++i) {
      const double v = pts[i].longitudinal_velocity_mps;
      if (i < 10) pub_head << v << " ";
      if (std::abs(v) < 1e-3) ++n_zero;
      sum_pub += v;
      max_pub = std::max(max_pub, v);
    }
    const double mean_pub = pts.empty() ? 0.0 : sum_pub / static_cast<double>(pts.size());
    ss << "\n  [published traj] vel[0..9]= " << pub_head.str() << "| points=" << pts.size()
       << " mean=" << mean_pub << " max=" << max_pub << " m/s zero_pts=" << n_zero;
  }

  RCLCPP_INFO_STREAM_THROTTLE(
    get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS, "[oneplanner-debug]" << ss.str());
}

void OnePlannerNode::on_pointcloud(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("processing_time");

  diagnostics_inference_->clear();
  const rclcpp::Time current_time(get_clock()->now());

  if (!core_->is_map_loaded()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS, "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Take data from subscribers
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();

  const auto frame_context = core_->create_frame_context(
    ego_kinematic_state, ego_acceleration, traffic_signals, turn_indicators_ptr, route_ptr,
    current_time);

  if (!frame_context) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Missing input data. ego_kinematic_state: "
        << (ego_kinematic_state ? "true" : "false")
        << ", ego_acceleration: " << (ego_acceleration ? "true" : "false")
        << ", route: " << (core_->get_route() ? "true" : "false")
        << ", turn_indicators: " << (turn_indicators_ptr ? "true" : "false"));
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "No input data available for inference");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Guard against an empty pointcloud (e.g. a transient sensor glitch): the voxelization CUDA
  // kernels assert on a positive point count and would otherwise crash the whole node.
  if (msg_ptr->width * msg_ptr->height == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Received an empty pointcloud, skipping this frame.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Empty pointcloud");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run the BEV encoder on the incoming pointcloud
  if (!bev_encoder_->encode(msg_ptr, tf_buffer_)) {
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, "BEV encoder inference failed");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Build and normalize planner inputs
  auto input_data_map = core_->create_input_data(*frame_context);
  autoware::diffusion_planner::preprocess::normalize_input_data(
    input_data_map, core_->get_normalization_map());
  if (!autoware::diffusion_planner::utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run the planner head (consumes the BEV feature map directly on-device)
  const auto inference_result = planner_inference_->infer(input_data_map);
  if (!inference_result.outputs) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << inference_result.error_msg);
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, inference_result.error_msg);
    diagnostics_inference_->publish(current_time);
    return;
  }
  const auto & [prediction, turn_indicator_logit] = inference_result.outputs.value();

  const rclcpp::Time frame_time(frame_context->frame_time);
  OnePlannerOutput planner_output;
  try {
    planner_output = core_->create_planner_output(
      prediction, turn_indicator_logit, *frame_context, frame_time, generator_uuid_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  if (debug_tensor_logging_) {
    log_debug_tensors(
      input_data_map, prediction, planner_output.trajectory, *ego_kinematic_state,
      static_cast<int64_t>(msg_ptr->width) * static_cast<int64_t>(msg_ptr->height));
  }

  pub_trajectory_->publish(planner_output.trajectory);
  pub_trajectories_->publish(planner_output.candidate_trajectories);
  pub_turn_indicators_->publish(planner_output.turn_indicator_command);

  diagnostics_inference_->publish(frame_time);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

}  // namespace autoware::tensorrt_oneplanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_oneplanner::OnePlannerNode)
