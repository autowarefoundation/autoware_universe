// Copyright 2021 TIER IV, Inc.
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

#include "autoware/behavior_path_side_shift_module/scene.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_side_shift_module/utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::get_point;
using geometry_msgs::msg::Point;

SideShiftModule::SideShiftModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SideShiftParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{parameters}
{
}

void SideShiftModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  debug_data_.path_shifter.reset();
  debug_marker_.markers.clear();
  start_pose_reset_request_ = false;
  requested_lateral_offset_ = 0.0;
  inserted_lateral_offset_ = 0.0;
  inserted_shift_line_ = ShiftLine{};
  shift_status_ = SideShiftStatus::BEFORE_SHIFT;
  prev_output_ = ShiftedPath{};
  prev_shift_line_ = ShiftLine{};
  path_shifter_ = PathShifter{};
  resetPathCandidate();
  resetPathReference();
}

void SideShiftModule::processOnEntry()
{
  // write me... (Don't initialize variables, otherwise lateral offset gets zero on entry.)
  start_pose_reset_request_ = false;
}

void SideShiftModule::processOnExit()
{
  // write me...
  initVariables();
}

void SideShiftModule::setParameters(const std::shared_ptr<SideShiftParameters> & parameters)
{
  parameters_ = parameters;
}

bool SideShiftModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  // If the desired offset has a non-zero value, return true as we want to execute the plan.
  const bool has_request = std::fabs(requested_lateral_offset_) >= 1.0e-4;
  RCLCPP_DEBUG_STREAM(
    getLogger(), "ESS::isExecutionRequested() : " << std::boolalpha << has_request);

  return has_request;
}

bool SideShiftModule::isExecutionReady() const
{
  return true;  // TODO(Horibe) is it ok to say "always safe"?
}

bool SideShiftModule::isReadyForNextRequest(
  const double & min_request_time_sec, bool override_requests) const noexcept
{
  rclcpp::Time current_time = clock_->now();
  const auto interval_from_last_request_sec = current_time - last_requested_shift_change_time_;

  if (interval_from_last_request_sec.seconds() >= min_request_time_sec && !override_requests) {
    last_requested_shift_change_time_ = current_time;
    return true;
  }

  return false;
}

bool SideShiftModule::canTransitSuccessState()
{
  // Never return the FAILURE. When the desired offset is zero and the vehicle is in the original
  // drivable area,this module can stop the computation and return SUCCESS.
  constexpr double ZERO_THRESHOLD = 1.0e-4;

  const auto isOffsetDiffAlmostZero = [this]() noexcept { return true; }();

  const bool no_offset_diff = isOffsetDiffAlmostZero;
  const bool no_request = std::fabs(requested_lateral_offset_) < ZERO_THRESHOLD;

  const auto no_shifted_plan = [&]() {
    if (prev_output_.shift_length.empty()) {
      return true;
    } else {
      const auto max_planned_shift_length = std::max_element(
        prev_output_.shift_length.begin(), prev_output_.shift_length.end(),
        [](double a, double b) { return std::abs(a) < std::abs(b); });
      return std::abs(*max_planned_shift_length) < 0.01;
    }
  }();

  RCLCPP_DEBUG(
    getLogger(), "ESS::updateState() : no_request = %d, no_shifted_plan = %d", no_request,
    no_shifted_plan);

  if (no_request && no_shifted_plan && no_offset_diff) {
    return true;
  }

  // Check if inserted_shift_line_ is valid (has been set with actual shift line)
  const bool has_valid_shift_line =
    std::abs(inserted_shift_line_.end_shift_length) > ZERO_THRESHOLD ||
    inserted_shift_line_.start_idx != inserted_shift_line_.end_idx;

  if (!has_valid_shift_line) {
    // No valid shift line has been inserted yet, stay in BEFORE_SHIFT
    shift_status_ = SideShiftStatus::BEFORE_SHIFT;
    return false;
  }

  const auto & current_lanes = utils::getCurrentLanes(planner_data_);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & inserted_shift_line_start_pose = inserted_shift_line_.start;
  const auto & inserted_shift_line_end_pose = inserted_shift_line_.end;
  const double self_to_shift_line_start_arc_length =
    autoware::behavior_path_planner::utils::getSignedDistance(
      current_pose, inserted_shift_line_start_pose, current_lanes);
  const double self_to_shift_line_end_arc_length =
    autoware::behavior_path_planner::utils::getSignedDistance(
      current_pose, inserted_shift_line_end_pose, current_lanes);

  RCLCPP_DEBUG(
    getLogger(), "SideShift: start_arc=%.2f, end_arc=%.2f", self_to_shift_line_start_arc_length,
    self_to_shift_line_end_arc_length);

  if (self_to_shift_line_start_arc_length >= 0) {
    shift_status_ = SideShiftStatus::BEFORE_SHIFT;
  } else if (self_to_shift_line_start_arc_length < 0 && self_to_shift_line_end_arc_length > 0) {
    shift_status_ = SideShiftStatus::SHIFTING;
  } else {
    shift_status_ = SideShiftStatus::AFTER_SHIFT;
  }
  return false;
}

void SideShiftModule::updateData()
{
  if (
    planner_data_->lateral_offset != nullptr &&
    planner_data_->lateral_offset->stamp != latest_lateral_offset_stamp_) {
    if (isReadyForNextRequest(parameters_->shift_request_time_limit)) {
      lateral_offset_change_request_ = true;
      requested_lateral_offset_ = planner_data_->lateral_offset->lateral_offset;
      latest_lateral_offset_stamp_ = planner_data_->lateral_offset->stamp;
    }
  }

  if (getCurrentStatus() != ModuleStatus::RUNNING && getCurrentStatus() != ModuleStatus::IDLE) {
    return;
  }

  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_line = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftLines()) {
      max_dist = std::max(max_dist, autoware_utils::calc_distance2d(getEgoPose(), pnt.start));
    }
    return max_dist;
  }();

  const auto reference_pose = prev_output_.shift_length.empty()
                                ? planner_data_->self_odometry->pose.pose
                                : utils::getUnshiftedEgoPose(getEgoPose(), prev_output_);
  if (getPreviousModuleOutput().reference_path.points.empty()) {
    return;
  }
  const auto centerline_path = utils::calcCenterLinePath(
    planner_data_, reference_pose, longest_dist_to_shift_line,
    getPreviousModuleOutput().reference_path);

  constexpr double resample_interval = 1.0;
  const auto backward_extened_path = extendBackwardLength(getPreviousModuleOutput().path);
  reference_path_ = utils::resamplePathWithSpline(backward_extened_path, resample_interval);

  path_shifter_.setPath(reference_path_);

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
  }

  // For current_lanes with desired length
  current_lanelets_ = route_handler->getLaneletSequence(
    current_lane, reference_pose, p.backward_path_length, p.forward_path_length);

  const size_t nearest_idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
  path_shifter_.removeBehindShiftLineAndSetBaseOffset(nearest_idx);
}

void SideShiftModule::replaceShiftLine()
{
  auto shift_lines = path_shifter_.getShiftLines();
  if (shift_lines.size() > 0) {
    shift_lines.clear();
  }

  const auto new_sl = calcShiftLine();

  // if no conflict, then add the new point.
  shift_lines.push_back(new_sl);
  const bool new_sl_is_same_with_previous =
    new_sl.end_shift_length == prev_shift_line_.end_shift_length;

  if (!new_sl_is_same_with_previous) {
    prev_shift_line_ = new_sl;
  }

  // set to path_shifter
  path_shifter_.setShiftLines(shift_lines);
  lateral_offset_change_request_ = false;
  inserted_lateral_offset_ = requested_lateral_offset_;
  inserted_shift_line_ = new_sl;

  RCLCPP_DEBUG(
    getLogger(), "SideShift: replaceShiftLine called. start_idx=%lu, end_idx=%lu, end_shift=%.2f",
    new_sl.start_idx, new_sl.end_idx, new_sl.end_shift_length);

  return;
}

BehaviorModuleOutput SideShiftModule::plan()
{
  // Replace shift line only when change is requested and not currently shifting
  if (
    lateral_offset_change_request_ && ((shift_status_ == SideShiftStatus::BEFORE_SHIFT) ||
                                       (shift_status_ == SideShiftStatus::AFTER_SHIFT))) {
    replaceShiftLine();
  }

  // Refine path
  ShiftedPath shifted_path;
  path_shifter_.generate(&shifted_path);

  if (shifted_path.path.points.empty()) {
    RCLCPP_ERROR(getLogger(), "Generated shift_path has no points");
    return {};
  }

  // Reset orientation
  setOrientation(&shifted_path.path);

  auto output = adjustDrivableArea(shifted_path);
  output.reference_path = getPreviousModuleOutput().reference_path;

  prev_output_ = shifted_path;
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);

  if (parameters_->publish_debug_marker) {
    setDebugMarkersVisualization();
  } else {
    debug_marker_.markers.clear();
  }

  return output;
}

CandidateOutput SideShiftModule::planCandidate() const
{
  auto path_shifter_local = path_shifter_;

  path_shifter_local.addShiftLine(calcShiftLine());

  // Refine path
  ShiftedPath shifted_path;
  path_shifter_local.generate(&shifted_path);

  // Reset orientation
  setOrientation(&shifted_path.path);

  return CandidateOutput(shifted_path.path);
}

BehaviorModuleOutput SideShiftModule::planWaitingApproval()
{
  // Refine path
  ShiftedPath shifted_path;
  path_shifter_.generate(&shifted_path);

  // Reset orientation
  setOrientation(&shifted_path.path);

  auto output = adjustDrivableArea(shifted_path);

  output.reference_path = getPreviousModuleOutput().reference_path;

  path_candidate_ = std::make_shared<PathWithLaneId>(planCandidate().path_candidate);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  prev_output_ = shifted_path;

  return output;
}

// can be moved to utils
ShiftLine SideShiftModule::calcShiftLine() const
{
  const auto & p = parameters_;
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  const double dist_to_start =
    std::max(p->min_distance_to_start_shifting, ego_speed * p->time_to_start_shifting);

  const double dist_to_end = [&]() {
    const double shift_length =
      requested_lateral_offset_ - getClosestShiftLength(prev_output_, getEgoPose().position);
    const double jerk_shifting_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
      shift_length, p->shifting_lateral_jerk, std::max(ego_speed, p->min_shifting_speed));
    const double shifting_distance = std::max(jerk_shifting_distance, p->min_shifting_distance);
    const double dist_to_end = dist_to_start + shifting_distance;
    RCLCPP_DEBUG(
      getLogger(), "min_distance_to_start_shifting = %f, dist_to_start = %f, dist_to_end = %f",
      parameters_->min_distance_to_start_shifting, dist_to_start, dist_to_end);
    return dist_to_end;
  }();

  const double limited_offset = calcMaxLateralOffset(requested_lateral_offset_);

  const size_t nearest_idx = planner_data_->findEgoIndex(reference_path_.points);
  ShiftLine shift_line;
  shift_line.end_shift_length = limited_offset;
  shift_line.start_idx = utils::getIdxByArclength(reference_path_, nearest_idx, dist_to_start);
  shift_line.start = reference_path_.points.at(shift_line.start_idx).point.pose;
  shift_line.end_idx = utils::getIdxByArclength(reference_path_, nearest_idx, dist_to_end);
  shift_line.end = reference_path_.points.at(shift_line.end_idx).point.pose;

  return shift_line;
}

BehaviorModuleOutput SideShiftModule::adjustDrivableArea(const ShiftedPath & path) const
{
  BehaviorModuleOutput out;
  const auto & p = planner_data_->parameters;

  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto itr = std::minmax_element(path.shift_length.begin(), path.shift_length.end());

  constexpr double threshold = 0.1;
  constexpr double margin = 0.5;
  const double left_offset = std::max(
    *itr.second + (*itr.first > threshold ? margin : 0.0), dp.drivable_area_left_bound_offset);
  const double right_offset = -std::min(
    *itr.first - (*itr.first < -threshold ? margin : 0.0), -dp.drivable_area_right_bound_offset);

  // crop path which is too long here
  auto output_path = path.path;
  const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(output_path.points);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  output_path.points = autoware::motion_utils::cropPoints(
    output_path.points, current_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  auto drivable_lanes = utils::generateDrivableLanes(current_lanelets_);

  if (parameters_->drivable_area_check_mode == 2) {
    const auto & route_handler = planner_data_->route_handler;
    const auto itr = std::minmax_element(path.shift_length.begin(), path.shift_length.end());
    const double max_left_shift = *itr.second;
    const double max_right_shift = *itr.first;

    for (auto & dl : drivable_lanes) {
      if (dl.middle_lanes.empty()) continue;
      const auto & current_lane = dl.middle_lanes.front();

      if (max_left_shift > 0.1) {
        const auto left_lane = route_handler->getLeftLanelet(current_lane, true, false);
        if (left_lane) {
          dl.left_lane = *left_lane;
        }
      }
      if (max_right_shift < -0.1) {
        const auto right_lane = route_handler->getRightLanelet(current_lane, true, false);
        if (right_lane) {
          dl.right_lane = *right_lane;
        }
      }
    }
  }

  const auto shorten_lanes = utils::cutOverlappedLanes(output_path, drivable_lanes);
  const auto expanded_lanes =
    utils::expandLanelets(shorten_lanes, left_offset, right_offset, dp.drivable_area_types_to_skip);

  {  // for new architecture
    // NOTE: side shift module is not launched with other modules. Therefore, drivable_lanes can be
    // assigned without combining.
    out.path = output_path;
    out.reference_path = getPreviousModuleOutput().reference_path;
    out.drivable_area_info.drivable_lanes = expanded_lanes;
    out.drivable_area_info.is_already_expanded = true;
  }

  return out;
}

PathWithLaneId SideShiftModule::extendBackwardLength(const PathWithLaneId & original_path) const
{
  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftLines()) {
      max_dist = std::max(max_dist, calc_distance2d(getEgoPose(), pnt.start));
    }
    return max_dist;
  }();

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length = std::max(
    planner_data_->parameters.backward_path_length, longest_dist_to_shift_point + extra_margin);

  const auto & prev_reference = getPreviousModuleOutput().path;
  const size_t orig_ego_idx = findNearestIndex(original_path.points, getEgoPose().position);
  const size_t prev_ego_idx = findNearestSegmentIndex(
    prev_reference.points, get_point(original_path.points.at(orig_ego_idx)));

  size_t clip_idx = 0;
  for (size_t i = 0; i < prev_ego_idx; ++i) {
    if (backward_length > calcSignedArcLength(prev_reference.points, clip_idx, prev_ego_idx)) {
      break;
    }
    clip_idx = i;
  }

  PathWithLaneId extended_path{};
  {
    extended_path.points.insert(
      extended_path.points.end(), prev_reference.points.begin() + clip_idx,
      prev_reference.points.begin() + prev_ego_idx);
  }

  {
    extended_path.points.insert(
      extended_path.points.end(), original_path.points.begin() + orig_ego_idx,
      original_path.points.end());
  }

  return extended_path;
}

/**
 * @brief Calculate the maximum allowable lateral offset based on lane boundaries.
 *
 * This function limits the requested lateral offset to prevent the vehicle from
 * departing the drivable area (lanelet boundaries).
 *
 * @param requested_offset The requested lateral offset [m] (positive: left, negative: right)
 * @return The limited lateral offset that keeps the vehicle within lane boundaries
 *
 * Mode 0: No check - returns requested offset as-is
 * Mode 1: Current lane only - limits offset to current lane boundaries
 * Mode 2: Current lane + neighbor lanes - allows shifting into same-direction adjacent lanes
 */
double SideShiftModule::calcMaxLateralOffset(const double requested_offset) const
{
  // Mode 0: No boundary check, return requested offset directly
  if (parameters_->drivable_area_check_mode == 0) {
    return requested_offset;
  }

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;
  const double margin = parameters_->drivable_area_margin;
  const double vehicle_half_width = p.vehicle_width / 2.0;

  const size_t nearest_idx = planner_data_->findEgoIndex(reference_path_.points);
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  // Calculate distance to start shifting to skip checking the path before shifting
  const double dist_to_start = std::max(
    parameters_->min_distance_to_start_shifting, ego_speed * parameters_->time_to_start_shifting);

  // Estimate shifting distance to determine check horizon
  const double shift_length = std::abs(requested_offset);
  const double jerk_shifting_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    shift_length, parameters_->shifting_lateral_jerk,
    std::max(ego_speed, parameters_->min_shifting_speed));
  const double shifting_distance =
    std::max(jerk_shifting_distance, parameters_->min_shifting_distance);

  // Check up to end of shift + buffer (e.g. 20m or 2s) to avoid restricting the shift based on
  // far-future road conditions
  const double check_distance = dist_to_start + shifting_distance + std::max(20.0, ego_speed * 2.0);

  const size_t start_idx = utils::getIdxByArclength(reference_path_, nearest_idx, dist_to_start);
  const size_t end_idx = utils::getIdxByArclength(reference_path_, nearest_idx, check_distance);

  double safe_left_limit = 100.0;
  double safe_right_limit = -100.0;
  bool found_valid_limit = false;

  // Check range: from shift start point to end of check distance
  for (size_t i = start_idx; i < std::min(end_idx, reference_path_.points.size()); ++i) {
    const auto & pt = reference_path_.points.at(i);
    const auto & pose = pt.point.pose;

    lanelet::ConstLanelet lane;
    if (!route_handler->getClosestLaneletWithinRoute(pose, &lane)) {
      continue;
    }

    const lanelet::BasicPoint2d target_point(pose.position.x, pose.position.y);

    // For mode 2, check if adjacent lanes in the same direction exist
    bool allow_left = false;
    bool allow_right = false;
    lanelet::ConstLanelet left_lane_val;
    lanelet::ConstLanelet right_lane_val;
    bool has_left = false;
    bool has_right = false;

    if (parameters_->drivable_area_check_mode == 2) {
      // Get adjacent lanes (same direction only)
      const auto l = route_handler->getLeftLanelet(lane, true, false);
      const auto r = route_handler->getRightLanelet(lane, true, false);
      if (l) {
        allow_left = true;
        has_left = true;
        left_lane_val = *l;
      }
      if (r) {
        allow_right = true;
        has_right = true;
        right_lane_val = *r;
      }
    }

    if (!lanelet::geometry::inside(lane, target_point)) {
      bool is_safe = false;
      if (has_left && lanelet::geometry::inside(left_lane_val, target_point)) is_safe = true;
      if (has_right && lanelet::geometry::inside(right_lane_val, target_point)) is_safe = true;

      if (!is_safe) {
        // If the reference path point is outside the lane, we cannot guarantee safety.
        // Clamp to 0.0 to avoid further deviation.
        safe_left_limit = 0.0;
        safe_right_limit = 0.0;
        found_valid_limit = true;
        continue;
      }
    }

    const auto left_bound = lane.leftBound2d();
    const auto right_bound = lane.rightBound2d();

    const double dist_to_left = lanelet::geometry::distance2d(left_bound, target_point);
    const double dist_to_right = lanelet::geometry::distance2d(right_bound, target_point);

    // Add a small extra buffer (e.g. 5cm) to account for vehicle corner deviation on curves
    const double extra_buffer = 0.05;
    double current_max_left = dist_to_left - vehicle_half_width - margin - extra_buffer;
    if (allow_left) {
      const double dist_to_far_left =
        lanelet::geometry::distance2d(left_lane_val.leftBound2d(), target_point);
      current_max_left = dist_to_far_left - vehicle_half_width - margin - extra_buffer;
    }

    double current_max_right = -(dist_to_right - vehicle_half_width - margin - extra_buffer);
    if (allow_right) {
      const double dist_to_far_right =
        lanelet::geometry::distance2d(right_lane_val.rightBound2d(), target_point);
      current_max_right = -(dist_to_far_right - vehicle_half_width - margin - extra_buffer);
    }

    safe_left_limit = std::min(safe_left_limit, std::max(0.0, current_max_left));
    safe_right_limit = std::max(safe_right_limit, std::min(0.0, current_max_right));
    found_valid_limit = true;
  }

  if (!found_valid_limit) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000, "SideShift: No valid lane found for limit check. Returning 0.0");
    return 0.0;
  }

  // Clamp the requested offset to the safe limits
  double result = 0.0;
  if (requested_offset > 0.0) {
    result = std::min(requested_offset, safe_left_limit);
  } else {
    result = std::max(requested_offset, safe_right_limit);
  }

  // Ignore very small offsets
  if (std::abs(result) < 1e-3) {
    result = 0.0;
  }

  RCLCPP_DEBUG_THROTTLE(
    getLogger(), *clock_, 1000, "SideShift: req=%.2f, safe_l=%.2f, safe_r=%.2f, res=%.2f, mode=%d",
    requested_offset, safe_left_limit, safe_right_limit, result,
    parameters_->drivable_area_check_mode);

  return result;
}

void SideShiftModule::setDebugMarkersVisualization() const
{
  using marker_utils::createShiftLineMarkerArray;

  debug_marker_.markers.clear();

  const auto add = [this](const MarkerArray & added) {
    autoware_utils::append_marker_array(added, &debug_marker_);
  };

  const auto add_shift_line_marker = [this, add](
                                       const auto & ns, auto r, auto g, auto b, double w = 0.1) {
    add(createShiftLineMarkerArray(
      debug_data_.path_shifter->getShiftLines(), debug_data_.path_shifter->getBaseOffset(), ns, r,
      g, b, w));
  };

  if (debug_data_.path_shifter) {
    add_shift_line_marker("side_shift_shift_points", 0.7, 0.7, 0.7, 0.4);
  }
}
}  // namespace autoware::behavior_path_planner
