// Copyright 2026 Xinchen Lin. Licensed under the Apache License, Version 2.0.
#include <autoware/camp_diffusion_adapter/adapter.hpp>
#include <autoware/camp_diffusion_adapter/input_chain_audit.hpp>
#include <autoware/diffusion_planner/conversion/lanelet.hpp>
#include <autoware/diffusion_planner/dimensions.hpp>
#include <autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace autoware::camp_diffusion_adapter
{
static_assert(
  camp::MAX_NUM_AGENTS == dp::MAX_NUM_AGENTS && camp::OUTPUT_T == dp::OUTPUT_T &&
    camp::POSE_DIM == dp::POSE_DIM && camp::NUM_SEGMENTS_IN_LANE == dp::NUM_SEGMENTS_IN_LANE &&
    camp::NUM_SEGMENTS_IN_ROUTE == dp::NUM_SEGMENTS_IN_ROUTE &&
    camp::POINTS_PER_SEGMENT == dp::POINTS_PER_SEGMENT &&
    camp::SEGMENT_POINT_DIM == dp::SEGMENT_POINT_DIM && dp::TRAFFIC_LIGHT_GREEN == 8 &&
    dp::TRAFFIC_LIGHT_WHITE == 11,
  "upstream DP tensor layout differs from the preserved CAMP adapter contract");
CampDiffusionAdapter::CampDiffusionAdapter(
  dp::DiffusionPlannerParams params, dp::VehicleInfo vehicle, const std::string & path)
: params_(std::move(params)),
  vehicle_(std::move(vehicle)),
  model_(camp::load_camp_fixed_weight_model(path))
{
  camp::validate_fixed_dp_contract(
    model_.candidate_pool_k, params_.batch_size, params_.temperature_list,
    params_.ego_snap_to_prev_trajectory.enable, params_.shift_x, params_.model_type);
  const auto args = std::filesystem::path(params_.base_model_directory) / params_.args_filename;
  state_normalization_ = dp::utils::load_state_normalization(args.string());
  reset_episode();
}

void CampDiffusionAdapter::reset_episode()
{
  // load_model alone does not clear upstream observation/turn/light histories.
  // Clear pending/continuity even if reconstruction/loading throws.
  pending_.reset();
  selected_ready_.reset();
  ledger_.reset_episode();
  core_ = std::make_unique<dp::DiffusionPlannerCore>(params_, vehicle_);
  core_->resolve_model_paths();
  core_->set_start_guidance_enabled(start_guidance_enabled_);
  core_->set_stop_guidance_enabled(stop_guidance_enabled_);
  core_->set_centerline_guidance_enabled(centerline_guidance_enabled_);
  core_->load_model();
  if (map_) core_->set_map(map_);
}

void CampDiffusionAdapter::set_start_guidance_enabled(bool enabled)
{
  core_->set_start_guidance_enabled(enabled);
  start_guidance_enabled_ = enabled;
}
void CampDiffusionAdapter::set_stop_guidance_enabled(bool enabled)
{
  core_->set_stop_guidance_enabled(enabled);
  stop_guidance_enabled_ = enabled;
}
void CampDiffusionAdapter::set_centerline_guidance_enabled(bool enabled)
{
  core_->set_centerline_guidance_enabled(enabled);
  centerline_guidance_enabled_ = enabled;
}

void CampDiffusionAdapter::set_map(const lanelet::LaneletMapConstPtr & map)
{
  if (!map) throw std::invalid_argument("map pointer is null");
  const auto internal = dp::convert_to_internal_lanelet_map(map, params_.line_string_max_step_m);
  std::vector<camp::CampLaneBoundary> boundaries;
  boundaries.reserve(internal.lane_segments.size());
  for (const auto & segment : internal.lane_segments) {
    boundaries.push_back({segment.left_boundary, segment.right_boundary});
  }
  map_ = map;
  boundaries_ = std::move(boundaries);
  reset_episode();
}

PreparedTick CampDiffusionAdapter::prepare(
  const dp::FrameContext & frame, const rclcpp::Time & timestamp, const dp::UUID & generator_uuid)
{
  pending_.reset();
  selected_ready_.reset();
  ledger_.discard_pending();
  if (!ready()) throw std::runtime_error("DP model and map are required");
  auto input_data = core_->create_input_data(frame);
  if (!dp::utils::check_input_map(input_data))
    throw std::invalid_argument("upstream input-map validation failed");
  std::optional<dp::InputDataMap> audited_raw;
  if (input_chain_audit_requested()) audited_raw = input_data;
  auto raw = camp::capture_raw_tensor_context(input_data);
  // Keep CAMP geometry in original units; normalize the generator inputs exactly as upstream.
  dp::preprocess::normalize_input_data(input_data, core_->get_observation_normalization());
  if (!dp::utils::check_input_map(input_data))
    throw std::invalid_argument("normalized upstream input-map validation failed");
  if (audited_raw)
    record_actual_input_chain_once(
      frame, *audited_raw, input_data, core_->get_observation_normalization(), state_normalization_,
      (std::filesystem::path(params_.base_model_directory) / params_.args_filename).string(),
      params_.batch_size);
  const auto inference = core_->run_inference(input_data);  // Exactly one unchanged generator call.
  if (!inference) throw std::runtime_error(inference.error());
  const auto & inferred = inference.value();
  auto predictions = inferred.is_denormalized ? inferred.outputs.first
                                              : dp::postprocess::denormalize_prediction(
                                                  inferred.outputs.first, state_normalization_);
  const auto expected_size = static_cast<std::size_t>(params_.batch_size) * camp::MAX_NUM_AGENTS *
                             camp::OUTPUT_T * camp::POSE_DIM;
  if (
    predictions.size() != expected_size ||
    inferred.outputs.second.size() !=
      static_cast<std::size_t>(params_.batch_size) * dp::TURN_INDICATOR_OUTPUT_DIM ||
    !std::all_of(
      inferred.outputs.second.begin(), inferred.outputs.second.end(),
      [](float x) { return std::isfinite(x); }) ||
    !std::all_of(predictions.begin(), predictions.end(), [](float x) { return std::isfinite(x); }))
    throw std::invalid_argument("inference prediction/logit shape or finite-state contract failed");

  camp::CampAtomMaterializationInput atoms_input;
  atoms_input.denormalized_predictions = predictions;
  atoms_input.batch_size = params_.batch_size;
  atoms_input.agent_count = camp::MAX_NUM_AGENTS;
  atoms_input.tensor_context = {
    std::move(raw.lanes), std::move(raw.route_lanes), std::move(raw.route_speed_limits),
    raw.route_has_traffic_light};
  atoms_input.ego_to_map = frame.ego_to_map_transform;
  atoms_input.lane_boundaries = &boundaries_;
  const dp::VehicleSpec spec(vehicle_);
  atoms_input.ego_wheelbase_m = spec.wheel_base;
  atoms_input.ego_length_m = spec.vehicle_length;
  atoms_input.ego_width_m = spec.vehicle_width;
  atoms_input.origin_seconds = frame.frame_time.seconds();
  for (std::size_t i = 0;
       i < std::min(camp::kCampActorCount, frame.ego_centric_neighbor_histories.size()); ++i) {
    const auto & history = frame.ego_centric_neighbor_histories.at(i);
    if (history.empty()) continue;
    const auto & shape = history.get_latest_state().original_info.shape.dimensions;
    if (std::isfinite(shape.x) && shape.x > 0 && std::isfinite(shape.y) && shape.y > 0)
      atoms_input.actor_shapes.at(i) = {true, shape.x, shape.y};
  }
  if (ledger_.previous()) {
    atoms_input.previous_plan =
      camp::CampPreviousPlan{ledger_.previous()->origin_seconds, ledger_.previous()->states};
  }
  auto materialized = camp::materialize_camp_atoms(atoms_input, model_.transition_component_scales);
  const auto expected =
    camp::rank_camp_candidates(model_, materialized.status, materialized.raw_atoms);
  auto poses = dp::postprocess::parse_predictions(predictions, frame.ego_to_map_transform);
  // Preserves every original row's delay prefix and per-row turn manager history.
  // Upstream commits generator history inside this call; public APIs offer no rollback.
  auto output = core_->create_planner_output(inferred, frame, timestamp, generator_uuid);
  if (output.candidate_trajectories.candidate_trajectories.size() != model_.candidate_pool_k)
    throw std::runtime_error("upstream output count differs from fixed K");
  PreparedTick prepared;
  prepared.pool.header = output.trajectory.header;
  prepared.pool.candidates = std::move(output.candidate_trajectories);
  for (const auto & candidate : prepared.pool.candidates.candidate_trajectories) {
    if (candidate.header != prepared.pool.header || candidate.points.empty())
      throw std::runtime_error("upstream candidate headers/points are inconsistent");
  }
  for (std::size_t i = 0; i < camp::kCampAtomCount; ++i) {
    using Pool = autoware_camp_selector::msg::CampCandidatePool;
    switch (materialized.status.at(i)) {
      case camp::CampAtomStatus::Observed:
        prepared.pool.atom_status.at(i) = Pool::OBSERVED;
        break;
      case camp::CampAtomStatus::NotApplicable:
        prepared.pool.atom_status.at(i) = Pool::NOT_APPLICABLE;
        break;
      case camp::CampAtomStatus::TypedMissing:
        prepared.pool.atom_status.at(i) = Pool::TYPED_MISSING;
        break;
    }
  }
  for (const auto & row : materialized.raw_atoms)
    prepared.pool.raw_atoms.insert(prepared.pool.raw_atoms.end(), row.begin(), row.end());
  prepared.pool.pool_id =
    ledger_.stage(atoms_input.origin_seconds, std::move(materialized.candidate_world_plans));
  pending_.emplace(Pending{prepared, frame, std::move(poses), expected});
  return prepared;
}

std::optional<SelectedTick> CampDiffusionAdapter::select(
  const autoware_camp_selector::msg::CampSelection & selection)
{
  if (!pending_) return std::nullopt;
  const auto & pool = pending_->tick.pool;
  const auto row = selection.selected_index;
  if (
    selection.pool_id != pool.pool_id || selection.header != pool.header ||
    !ledger_.candidate(selection.pool_id, row) || row != pending_->expected.selected_index ||
    selection.costs != pending_->expected.costs ||
    selection.candidate != pool.candidates.candidate_trajectories.at(row))
    return std::nullopt;
  SelectedTick result;
  result.trajectory.header = selection.candidate.header;
  result.trajectory.points = selection.candidate.points;
  result.turn_indicators_command = selection.candidate.turn_indicators_command;
  result.predicted_objects = dp::postprocess::create_predicted_objects(
    pending_->poses, pending_->frame.ego_centric_neighbor_histories,
    rclcpp::Time(selection.header.stamp), row);
  result.selection = selection;
  selected_ready_ = row;
  return result;
}

bool CampDiffusionAdapter::confirm_published_selection(
  const std_msgs::msg::Header & header, std::uint32_t row)
{
  if (!pending_ || selected_ready_ != row || pending_->tick.pool.header != header) return false;
  const bool committed = ledger_.confirm_published(pending_->tick.pool.pool_id, row);
  if (committed) {
    pending_.reset();
    selected_ready_.reset();
  }
  return committed;
}
}  // namespace autoware::camp_diffusion_adapter
