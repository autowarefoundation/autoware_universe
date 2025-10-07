// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "autoware/freespace_planning_algorithms/astar_search.hpp"

#include "autoware/freespace_planning_algorithms/abstract_algorithm.hpp"
#include "autoware/freespace_planning_algorithms/kinematic_bicycle_model.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <queue>
#include <utility>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
using autoware_utils::calc_distance2d;

double calcReedsSheppDistance(const Pose & p1, const Pose & p2, double radius)
{
  const auto rs_space = ReedsSheppStateSpace(radius);
  const ReedsSheppStateSpace::StateXYT pose0{
    p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  const ReedsSheppStateSpace::StateXYT pose1{
    p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

Pose calcRelativePose(const Pose & base_pose, const Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param)
: AbstractPlanningAlgorithm(
    planner_common_param, std::make_shared<rclcpp::Clock>(RCL_ROS_TIME), collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param, const rclcpp::Clock::SharedPtr & clock)
: AbstractPlanningAlgorithm(planner_common_param, clock, collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  final_segment_threshold_ = astar_param_.final_segment_threshold;
  extra_steering_penalty_factor_ = astar_param_.extra_steering_penalty_factor;
  yaw_weight_ = astar_param_.yaw_weight;
  distance_to_goal_extension_weight_ = astar_param_.distance_to_goal_extension_weight;
  reparking_forward_first_weight_ = astar_param_.reparking_forward_first_weight;
  reparking_deviation_penalty_ = astar_param_.reparking_deviation_penalty;
  reparking_alignment_weight_ = astar_param_.reparking_alignment_weight;
  reparking_distance_ = astar_param_.reparking_distance;
  is_reparking_ = false;

  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

void AstarSearch::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);

  // ensure minimum expansion distance is larger then grid cell diagonal length
  min_expansion_dist_ = std::max(astar_param_.expansion_distance, 1.5 * costmap_.info.resolution);
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);
}

void AstarSearch::resetData()
{
  // clearing openlist is necessary because otherwise remaining elements of openlist
  // point to deleted node.
  openlist_ = std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison>();
  const int nb_of_grid_nodes = costmap_.info.width * costmap_.info.height;
  const int total_astar_node_count = nb_of_grid_nodes * planner_common_param_.theta_size;
  graph_.assign(total_astar_node_count, AstarNode{});
  col_free_distance_map_.assign(nb_of_grid_nodes, std::numeric_limits<double>::max());
  shifted_goal_pose_ = {};
}

bool AstarSearch::makePlan(const Pose & start_pose, const Pose & goal_pose)
{
  resetData();

  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  if (detectCollision(start_pose_) || detectCollision(goal_pose_)) {
    throw std::logic_error("Invalid start or goal pose");
  }

  if (is_backward_search_) std::swap(start_pose_, goal_pose_);

  setCollisionFreeDistanceMap();

  is_multiple_goals_ = false;

  setStartNode();

  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
  }

  return true;
}

bool AstarSearch::makePlan(
  const geometry_msgs::msg::Pose & start_pose,
  const std::vector<geometry_msgs::msg::Pose> & goal_candidates)
{
  if (goal_candidates.empty()) return false;

  if (goal_candidates.size() == 1) {
    return makePlan(start_pose, goal_candidates.front());
  }

  resetData();

  start_pose_ = global2local(costmap_, start_pose);

  std::vector<Pose> goals_local;
  for (const auto & goal : goal_candidates) {
    const auto goal_local = global2local(costmap_, goal);
    if (detectCollision(goal_local)) continue;
    goals_local.push_back(goal_local);
  }

  if (detectCollision(start_pose_) || goals_local.empty()) {
    throw std::logic_error("Invalid start or goal pose");
  }

  goal_pose_ = is_backward_search_ ? start_pose_ : goals_local.front();

  setCollisionFreeDistanceMap();

  is_multiple_goals_ = true;

  if (is_backward_search_) {
    double cost_offset = 0.0;
    for (const auto & pose : goals_local) {
      start_pose_ = pose;
      setStartNode(cost_offset);
      cost_offset += multi_goal_backward_cost_offset;
    }
  } else {
    setStartNode();
    alternate_goals_ = goals_local;
  }

  if (!search()) {
    throw std::logic_error("HA* failed to find path to goal");
  }

  return true;
}

void AstarSearch::setReparking(bool is_reparking)
{
  is_reparking_ = is_reparking;
  return;
}

void AstarSearch::setCollisionFreeDistanceMap()
{
  using Entry = std::pair<IndexXY, double>;
  struct CompareEntry
  {
    bool operator()(const Entry & a, const Entry & b) const { return a.second > b.second; }
  };
  std::priority_queue<Entry, std::vector<Entry>, CompareEntry> heap;
  std::vector<bool> closed(col_free_distance_map_.size(), false);
  auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  col_free_distance_map_[indexToId(goal_index)] = 0.0;
  heap.push({IndexXY{goal_index.x, goal_index.y}, 0.0});

  Entry current;
  std::array<int, 3> offsets = {1, 0, -1};
  while (!heap.empty()) {
    current = heap.top();
    heap.pop();
    const int id = indexToId(current.first);
    if (closed[id]) continue;
    closed[id] = true;

    const auto & index = current.first;
    for (const auto & offset_x : offsets) {
      const int x = index.x + offset_x;
      for (const auto & offset_y : offsets) {
        const int y = index.y + offset_y;
        const IndexXY n_index{x, y};
        const double offset = std::abs(offset_x) + std::abs(offset_y);
        if (isOutOfRange(n_index) || isObs(n_index) || offset < 1) continue;
        if (getObstacleEDT(n_index).distance < 0.5 * collision_vehicle_shape_.width) continue;
        const int n_id = indexToId(n_index);
        const double dist = current.second + (sqrt(offset) * costmap_.info.resolution);
        if (closed[n_id] || col_free_distance_map_[n_id] < dist) continue;
        col_free_distance_map_[n_id] = dist;
        heap.push({n_index, dist});
      }
    }
  }
}

void AstarSearch::setStartNode(const double cost_offset)
{
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  // Set start node
  AstarNode * start_node = &graph_[getKey(index)];
  const double initial_cost = estimateCost(start_pose_, index) + cost_offset;
  start_node->set(start_pose_, 0.0, initial_cost, 0, false);
  start_node->dir_distance = 0.0;
  start_node->dist_to_goal = calc_distance2d(start_pose_, goal_pose_);
  start_node->dist_to_obs = getObstacleEDT(index).distance;
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);
}

double AstarSearch::estimateCost(const Pose & pose, const IndexXYT & index) const
{
  double total_cost = col_free_distance_map_[indexToId(index)];
  // Temporarily, until reeds_shepp gets stable.
  if (use_reeds_shepp_) {
    total_cost =
      std::max(total_cost, calcReedsSheppDistance(pose, goal_pose_, avg_turning_radius_));
  }
  return astar_param_.distance_heuristic_weight * total_cost;
}

bool AstarSearch::search()
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // Start A* search
  while (!openlist_.empty()) {
    // Check time and terminate if the search reaches the time limit
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    // Expand minimum cost node
    AstarNode * current_node = openlist_.top();
    openlist_.pop();
    if (current_node->status == NodeStatus::Closed) continue;
    current_node->status = NodeStatus::Closed;

    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    expandNodes(*current_node);
    if (astar_param_.use_back) expandNodes(*current_node, true);
  }

  // Failed to find path
  return false;
}

void AstarSearch::expandNodes(AstarNode & current_node, const bool is_back)
{
  const auto current_pose = node2pose(current_node);
  const double direction = (is_back == is_backward_search_) ? 1.0 : -1.0;
  const double distance = getExpansionDistance(current_node) * direction;

  // Current distance from the goal
  double dist_g = calc_distance2d(current_pose, goal_pose_);

  // Check if the node have already switched direction
  bool already_switched_dir = current_node.reparking_direction_change;

  const double straight_yaw_tolerance = 0.05;  // e.g. ±~3 degrees. Adjust as needed.

  const double node_yaw = tf2::getYaw(current_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose_.orientation);

  // Forward alignment: node_yaw ≈ goal_yaw
  const double yaw_diff_forward = std::fabs(autoware_utils::normalize_radian(node_yaw - goal_yaw));

  // Backward alignment: node_yaw ≈ goal_yaw + π
  const double yaw_diff_backward =
    std::fabs(autoware_utils::normalize_radian(node_yaw - (goal_yaw + M_PI)));

  // "aligned" if either forward or backward difference is small
  bool aligned_with_goal_yaw = false;
  if (yaw_diff_forward < straight_yaw_tolerance || yaw_diff_backward < straight_yaw_tolerance) {
    aligned_with_goal_yaw = true;
  }

  for (int steering_index = -planner_common_param_.turning_steps;
       steering_index <= planner_common_param_.turning_steps; ++steering_index) {
    // Skip expansions that reverse direction with same steering
    if (
      current_node.parent != nullptr && (is_back != current_node.is_back) &&
      steering_index == current_node.steering_index) {
      continue;
    }

    if (is_reparking_ && !already_switched_dir && (dist_g < reparking_distance_)) {
      // if the node aligned with the goal yaw, skip nonzero steering
      if (aligned_with_goal_yaw && std::abs(steering_index) > 0) {
        continue;  // don't allow turning expansions if aligned
      }
    }
    const double steering = static_cast<double>(steering_index) * steering_resolution_;
    const auto next_pose = kinematic_bicycle_model::getPose(
      current_pose, collision_vehicle_shape_.base_length, steering, distance);
    const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    if (isOutOfRange(next_index) || isObs(next_index) || detectCollision(next_index)) {
      continue;
    }

    AstarNode * next_node = &graph_[getKey(next_index)];
    if (next_node->status == NodeStatus::Closed) {
      continue;
    }

    double next_dist_g = calc_distance2d(next_pose, goal_pose_);

    // Check if direction switched
    const bool direction_switched =
      (current_node.parent != nullptr) && (is_back != current_node.is_back);
    bool next_reparking_dir_change =
      (current_node.reparking_direction_change || direction_switched);

    // "pull‐out" phase, ensure next_dist_g > dist_g to keep moving away
    if (is_reparking_ && !already_switched_dir && (dist_g < reparking_distance_)) {
      if (next_dist_g <= dist_g) {
        // skip expansions that don't move us further from the goal
        continue;
      }
    }

    double total_weight = 1.0;
    total_weight += getSteeringCost(steering_index);
    if (is_back) total_weight *= (1.0 + planner_common_param_.reverse_weight);

    double move_cost = current_node.gc + (total_weight * std::abs(distance));
    move_cost += getSteeringChangeCost(steering_index, current_node.steering_index);

    // Obstacle and lateral distance cost
    const auto obs_edt = getObstacleEDT(next_index);
    move_cost += getObsDistanceCost(next_index, obs_edt);
    move_cost += getLatDistanceCost(next_pose);

    // Direction change penalty
    if (direction_switched) {
      move_cost += getDirectionChangeCost(current_node.dir_distance);
    }

    // Re‐parking alignment logic for the final approach
    double yaw_weight = yaw_weight_;
    double distance_to_goal_extension_weight = distance_to_goal_extension_weight_;
    double reparking_forward_first_weight = reparking_forward_first_weight_;
    // is in reparking state
    if (is_reparking_) {
      // first forward then reverse
      if (current_node.parent == nullptr && is_back) {
        move_cost += reparking_forward_first_weight;
      }
      yaw_weight *= reparking_alignment_weight_;
      distance_to_goal_extension_weight *= reparking_alignment_weight_;
    }
    if (is_reparking_ || current_node.dist_to_goal < final_segment_threshold_) {
      // steering param
      double steering_ratio = std::abs(static_cast<double>(steering_index)) /
                              static_cast<double>(planner_common_param_.turning_steps);
      double extra_steering_penalty = extra_steering_penalty_factor_ * steering_ratio;

      // yaw deviation
      double yaw_diff = autoware_utils::normalize_radian(
        tf2::getYaw(goal_pose_.orientation) - tf2::getYaw(next_pose.orientation));
      double yaw_cost = std::abs(yaw_diff) * yaw_weight;
      // reparking reverse phase
      if (next_reparking_dir_change) {
        double deviation_penalty = reparking_deviation_penalty_ * std::abs(yaw_diff);
        move_cost += deviation_penalty;
      }
      // reparking pull-out phase
      if (is_reparking_ && !current_node.reparking_direction_change && !is_back) {
        double align_penalty = 2.0 * reparking_deviation_penalty_ * std::abs(yaw_diff);
        move_cost += align_penalty;
      }

      // alignment
      double dx = next_pose.position.x - goal_pose_.position.x;
      double dy = next_pose.position.y - goal_pose_.position.y;
      double goal_yaw = tf2::getYaw(goal_pose_.orientation);
      double lateral_offset = std::fabs(-std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy);
      double distance_to_goal_extension_cost = lateral_offset * distance_to_goal_extension_weight;

      // total cost
      move_cost += (extra_steering_penalty + distance_to_goal_extension_cost + yaw_cost);
    }

    double total_cost = move_cost + estimateCost(next_pose, next_index);
    // Compare cost
    if (next_node->status == NodeStatus::None || next_node->fc > total_cost) {
      next_node->status = NodeStatus::Open;
      next_node->set(next_pose, move_cost, total_cost, steering_index, is_back);
      next_node->dir_distance =
        std::abs(distance) + (direction_switched ? 0.0 : current_node.dir_distance);
      next_node->dist_to_goal = next_dist_g;
      next_node->dist_to_obs = obs_edt.distance;
      next_node->parent = &current_node;
      next_node->reparking_direction_change = next_reparking_dir_change;
      openlist_.push(next_node);
    }
  }
}

double AstarSearch::getExpansionDistance(const AstarNode & current_node) const
{
  if (!astar_param_.adapt_expansion_distance || max_expansion_dist_ <= min_expansion_dist_) {
    return min_expansion_dist_;
  }
  double exp_dist = std::min(
    current_node.dist_to_goal * dist_to_goal_expansion_factor_,
    current_node.dist_to_obs * dist_to_obs_expansion_factor_);
  return std::clamp(exp_dist, min_expansion_dist_, max_expansion_dist_);
}

double AstarSearch::getSteeringCost(const int steering_index) const
{
  return planner_common_param_.curve_weight *
         (static_cast<double>(abs(steering_index)) / planner_common_param_.turning_steps);
}

double AstarSearch::getSteeringChangeCost(
  const int steering_index, const int prev_steering_index) const
{
  double steering_index_diff = abs(steering_index - prev_steering_index);
  return astar_param_.smoothness_weight * steering_index_diff /
         (2.0 * planner_common_param_.turning_steps);
}

double AstarSearch::getDirectionChangeCost(const double dir_distance) const
{
  return planner_common_param_.direction_change_weight * (1.0 + (1.0 / (1.0 + dir_distance)));
}

double AstarSearch::getObsDistanceCost(const IndexXYT & index, const EDTData & obs_edt) const
{
  if (obs_edt.distance > collision_vehicle_shape_.max_dimension + cost_free_obs_dist) {
    return 0.0;
  }
  const double yaw = index.theta * (2.0 * M_PI / planner_common_param_.theta_size);
  const double base_to_frame_dist = getVehicleBaseToFrameDistance(yaw - obs_edt.angle);
  const double vehicle_to_obs_dist = std::max(obs_edt.distance - base_to_frame_dist, 0.0);
  return astar_param_.obstacle_distance_weight *
         std::max(1.0 - (vehicle_to_obs_dist / cost_free_obs_dist), 0.0);
}

double AstarSearch::getLatDistanceCost(const Pose & pose) const
{
  if (is_multiple_goals_) return 0.0;
  const auto ref_pose = is_backward_search_ ? start_pose_ : goal_pose_;
  const double distance_to_goal = calc_distance2d(pose, ref_pose);
  if (distance_to_goal > near_goal_dist_) return 0.0;
  const double lat_distance = std::abs(calcRelativePose(ref_pose, pose).position.y);
  return astar_param_.goal_lat_distance_weight * lat_distance;
}

void AstarSearch::setPath(const AstarNode & goal_node)
{
  std_msgs::msg::Header header;
  header.stamp = clock_->now();
  header.frame_id = costmap_.header.frame_id;

  // From the goal node to the start node
  const AstarNode * node = &goal_node;

  std::vector<PlannerWaypoint> waypoints;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  if (shifted_goal_pose_) {
    pose.pose = local2global(costmap_, shifted_goal_pose_.get());
    waypoints.push_back({pose, goal_node.is_back});
  }

  const auto interpolate = [this, &waypoints, &pose](const AstarNode & node) {
    if (node.parent == nullptr || !astar_param_.adapt_expansion_distance) return;
    const auto parent_pose = node2pose(*node.parent);
    const double distance_2d = calc_distance2d(node2pose(node), parent_pose);
    const int n = static_cast<int>(distance_2d / min_expansion_dist_);
    for (int i = 1; i < n; ++i) {
      const double dist =
        ((distance_2d * i) / n) * (node.is_back == is_backward_search_ ? 1.0 : -1.0);
      const double steering = node.steering_index * steering_resolution_;
      const auto local_pose = kinematic_bicycle_model::getPose(
        parent_pose, collision_vehicle_shape_.base_length, steering, dist);
      pose.pose = local2global(costmap_, local_pose);
      waypoints.push_back({pose, node.is_back});
    }
  };

  // push astar nodes poses
  while (node != nullptr) {
    pose.pose = local2global(costmap_, node2pose(*node));
    waypoints.push_back({pose, node->is_back});
    interpolate(*node);
    // To the next node
    node = node->parent;
  }

  if (waypoints.empty()) return;

  if (waypoints.size() > 1) waypoints.back().is_back = waypoints.rbegin()[1].is_back;

  if (!is_backward_search_) {
    // Reverse the vector to be start to goal order
    std::reverse(waypoints.begin(), waypoints.end());
  }

  waypoints_.header = header;
  waypoints_.waypoints = waypoints;

  if (!is_backward_search_) return;

  for (size_t i = 0; i < waypoints_.waypoints.size() - 1; ++i) {
    const auto & current = waypoints_.waypoints[i];
    auto & next = waypoints_.waypoints[i + 1];

    if (current.is_back != next.is_back) {
      next.is_back = current.is_back;
      ++i;  // skip next waypoint
    }
  }
}

bool AstarSearch::isGoal(const AstarNode & node) const
{
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;
  const double goal_angle = autoware_utils::deg2rad(planner_common_param_.angle_goal_range / 2.0);

  const auto node_pose = node2pose(node);

  auto checkGoal = [this, &node_pose, &lateral_goal_range, &longitudinal_goal_range, &goal_angle,
                    &is_back = node.is_back](const Pose & pose) {
    const auto node_index = pose2index(costmap_, node_pose, planner_common_param_.theta_size);
    const auto goal_index = pose2index(costmap_, pose, planner_common_param_.theta_size);

    if (node_index == goal_index) return true;

    const auto relative_pose = calcRelativePose(pose, node_pose);

    bool is_behind_goal = relative_pose.position.x <= 0.0;

    if (astar_param_.only_behind_solutions && !is_behind_goal) {
      return false;
    }

    if (
      std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
      std::fabs(relative_pose.position.y) > lateral_goal_range) {
      return false;
    }

    const auto angle_diff =
      autoware_utils::normalize_radian(tf2::getYaw(relative_pose.orientation));
    if (std::abs(angle_diff) > goal_angle) {
      return false;
    }

    const bool is_set_shifted_goal_pose =
      is_backward_search_ ? is_behind_goal == is_back : is_behind_goal != is_back;
    if (is_set_shifted_goal_pose) {
      setShiftedGoalPose(pose, relative_pose.position.y);
    }

    return true;
  };

  bool is_goal_position = false;
  if (checkGoal(goal_pose_)) {
    is_goal_position = true;
  } else {
    for (const auto & alt_goal : alternate_goals_) {
      if (checkGoal(alt_goal)) {
        is_goal_position = true;
        break;
      }
    }
  }

  if (!is_goal_position) {
    return false;
  }

  if (!is_reparking_) {
    return true;
  }

  // ------------------------------------------------------
  //   [Additional Checks under Reparking Mode]
  // ------------------------------------------------------
  // Backtrack from this node to the start node and count:
  //  a) Number of direction changes (EXACTLY 1)
  //  b) Maximum distance from the trajectory to the target

  int direction_switch_count = 0;
  double max_dist_from_goal = 0.0;
  const int required_switch_count = 1;                    // direction_changed occurs exactly once
  const double reparking_distance = reparking_distance_;  // 2.0

  const AstarNode * current = &node;
  bool prev_is_back = current->is_back;

  while (current != nullptr) {
    double dist_g = calc_distance2d(node2pose(*current), goal_pose_);
    if (dist_g > max_dist_from_goal) {
      max_dist_from_goal = dist_g;
    }

    // check whether the direction_changed has been excuted?
    const AstarNode * parent = current->parent;
    if (parent) {
      bool direction_changed = (current->is_back != prev_is_back);
      // direction_changed must occu outside the minimum pull-out distance from goal
      if (direction_changed && dist_g >= reparking_distance) {
        direction_switch_count++;
      }
      prev_is_back = current->is_back;
    }

    current = parent;
  }

  bool exactly_one_switch = (direction_switch_count == required_switch_count);

  bool enough_move_out = (max_dist_from_goal >= reparking_distance);

  if (!exactly_one_switch || !enough_move_out) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("AstarSearch"),
      "Reparking path found but direction_switch_count=%d (need == %d) or didn't move out enough "
      "(%.2f < %.2f) => ignore",
      direction_switch_count, required_switch_count, max_dist_from_goal, reparking_distance);

    return false;
  }

  return true;
}

void AstarSearch::setShiftedGoalPose(const Pose & goal_pose, const double lat_offset) const
{
  tf2::Transform tf;
  tf2::convert(goal_pose, tf);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf);

  Pose lat_pose;
  lat_pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(lat_offset).z(0.0);
  lat_pose.orientation =
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0);

  shifted_goal_pose_ = transformPose(lat_pose, transform);
}

Pose AstarSearch::node2pose(const AstarNode & node) const
{
  Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  pose_local.orientation = autoware_utils::create_quaternion_from_yaw(node.theta);

  return pose_local;
}

}  // namespace autoware::freespace_planning_algorithms
