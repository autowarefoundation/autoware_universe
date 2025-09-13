// Copyright 2019 Autoware Foundation
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

#include "mission_planner.hpp"

#include "service_utils.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/route_checker.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mission_planner_universe
{
namespace
{
std::string route_state_to_string(const uint8_t state)
{
  switch (state) {
      // clang-format off
    case RouteState::UNKNOWN:      return "UNKNOWN";
    case RouteState::INITIALIZING: return "INITIALIZING";
    case RouteState::UNSET:        return "UNSET";
    case RouteState::ROUTING:      return "ROUTING";
    case RouteState::SET:          return "SET";
    case RouteState::REROUTING:    return "REROUTING";
    case RouteState::ARRIVED:      return "ARRIVED";
    case RouteState::ABORTED:      return "ABORTED";
    case RouteState::INTERRUPTED:  return "INTERRUPTED";
    default: return "UNKNOWN(" + std::to_string(static_cast<int>(state)) + ")";
      // clang-format on
  }
}
}  // namespace

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_(
    "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  odometry_(nullptr),
  map_ptr_(nullptr)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  map_frame_ = declare_parameter<std::string>("map_frame");
  reroute_time_threshold_ = declare_parameter<double>("reroute_time_threshold");
  minimum_reroute_length_ = declare_parameter<double>("minimum_reroute_length");
  allow_reroute_in_autonomous_mode_ = declare_parameter<bool>("allow_reroute_in_autonomous_mode");

  planner_ = plugin_loader_.createSharedInstance(
    "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
  planner_->initialize(this);

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS(1), std::bind(&MissionPlanner::on_odometry, this, _1));
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "~/input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&MissionPlanner::on_operation_mode_state, this, _1));
  sub_vector_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", durable_qos, std::bind(&MissionPlanner::on_map, this, _1));
  pub_marker_ = create_publisher<MarkerArray>("~/debug/route_marker", durable_qos);

  // NOTE: The route interface should be mutually exclusive by callback group.
  sub_modified_goal_ = create_subscription<PoseWithUuidStamped>(
    "~/input/modified_goal", durable_qos, std::bind(&MissionPlanner::on_modified_goal, this, _1));
  srv_clear_route = create_service<ClearRoute>(
    "~/clear_route", service_utils::handle_exception(&MissionPlanner::on_clear_route, this));
  srv_set_preferred_lane = create_service<SetPreferredLane>(
    "~/set_preferred_lane",
    service_utils::handle_exception(&MissionPlanner::set_preferred_lane, this));
  srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/set_lanelet_route",
    service_utils::handle_exception(&MissionPlanner::on_set_lanelet_route, this));
  srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/set_waypoint_route",
    service_utils::handle_exception(&MissionPlanner::on_set_waypoint_route, this));
  pub_route_ = create_publisher<LaneletRoute>("~/route", durable_qos);
  pub_state_ = create_publisher<RouteState>("~/state", durable_qos);

  // Route state will be published when the node gets ready for route api after initialization,
  // otherwise the mission planner rejects the request for the API.
  using namespace std::literals::chrono_literals;
  data_check_timer_ =
    rclcpp::create_timer(this, get_clock(), 0.1s, [this] { check_initialization(); });
  is_mission_planner_ready_ = false;

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
}

void MissionPlanner::publish_processing_time(
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void MissionPlanner::print_pose_log(
  const std::string & route_type, const Pose & initial_pose, const Pose & goal_pose)
{
  RCLCPP_INFO(this->get_logger(), "Route set via %s", route_type.c_str());

  const auto & ip = initial_pose.position;
  RCLCPP_INFO(this->get_logger(), "Initial pose - x: %f, y: %f, z: %f", ip.x, ip.y, ip.z);
  const auto & iq = initial_pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "Initial orientation - qx: %f, qy: %f, qz: %f, qw: %f", iq.x, iq.y, iq.z,
    iq.w);

  const auto & gp = goal_pose.position;
  RCLCPP_INFO(this->get_logger(), "Goal pose - x: %f, y: %f, z: %f", gp.x, gp.y, gp.z);
  const auto & gq = goal_pose.orientation;
  RCLCPP_INFO(
    this->get_logger(), "Goal orientation - qx: %f, qy: %f, qz: %f, qw: %f", gq.x, gq.y, gq.z,
    gq.w);
}

void MissionPlanner::check_initialization()
{
  auto logger = get_logger();
  auto clock = *get_clock();

  if (!planner_->ready()) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting lanelet map... Route API is not ready.");
    return;
  }
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(logger, clock, 5000, "waiting odometry... Route API is not ready.");
    return;
  }

  // All data is ready. Now API is available.
  is_mission_planner_ready_ = true;
  RCLCPP_DEBUG(logger, "Route API is ready.");
  change_state(RouteState::UNSET);

  // Stop timer callback.
  data_check_timer_->cancel();
  data_check_timer_ = nullptr;
}

void MissionPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;

  // NOTE: Do not check in the other states as goal may change.
  if (state_.state == RouteState::SET) {
    PoseStamped pose;
    pose.header = odometry_->header;
    pose.pose = odometry_->pose.pose;
    if (arrival_checker_.is_arrived(pose)) {
      change_state(RouteState::ARRIVED);
    }
  }
}

void MissionPlanner::on_operation_mode_state(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

void MissionPlanner::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  map_ptr_ = msg;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_ptr_, lanelet_map_ptr_);
}

Pose MissionPlanner::transform_pose(const Pose & pose, const Header & header)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Pose result;
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, header.frame_id, tf2::TimePointZero);
    tf2::doTransform(pose, result, transform);
    return result;
  } catch (tf2::TransformException & error) {
    throw service_utils::TransformError(error.what());
  }
}

void MissionPlanner::change_state(RouteState::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void MissionPlanner::on_modified_goal(const PoseWithUuidStamped::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received modified goal.");

  if (state_.state != RouteState::SET) {
    RCLCPP_ERROR(get_logger(), "The route hasn't set yet. Cannot reroute.");
    return;
  }
  if (!is_mission_planner_ready_) {
    RCLCPP_ERROR(get_logger(), "The mission planner is not ready.");
    return;
  }
  if (!current_route_) {
    RCLCPP_ERROR(get_logger(), "The route has not set yet.");
    return;
  }
  if (current_route_->uuid != msg->uuid) {
    RCLCPP_ERROR(get_logger(), "Goal uuid is incorrect.");
    return;
  }

  change_state(RouteState::REROUTING);
  const auto route = create_route(*msg);

  if (route.segments.empty()) {
    cancel_route();
    change_state(RouteState::SET);
    RCLCPP_ERROR(get_logger(), "The planned route is empty.");
    return;
  }

  change_route(route);
  change_state(RouteState::SET);
  RCLCPP_INFO(get_logger(), "Changed the route with the modified goal");
}

void MissionPlanner::on_clear_route(
  const ClearRoute::Request::SharedPtr, const ClearRoute::Response::SharedPtr res)
{
  if (!is_mission_planner_ready_) {
    using ResponseCode = autoware_adapi_v1_msgs::msg::ResponseStatus;
    throw service_utils::ServiceException(
      ResponseCode::NO_EFFECT, "The mission planner is not ready.", true);
  }

  change_route();
  change_state(RouteState::UNSET);
  res->status.success = true;
}

void MissionPlanner::set_preferred_lane(
  const SetPreferredLane::Request::SharedPtr req, const SetPreferredLane::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  RCLCPP_INFO_STREAM(
    get_logger(), "Received lane change override request with direction: "
                    << (req->lane_change_direction == 0   ? "LEFT"
                        : req->lane_change_direction == 1 ? "RIGHT"
                                                          : "AUTO"));

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE,
      fmt::format(
        "The lanelet route cannot be set in the current state: {}",
        route_state_to_string(state_.state)));
  }
  if (!is_mission_planner_ready_) {
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
  }
  if (is_reroute && !operation_mode_state_) {
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Reroute is not allowed in autonomous mode.");
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_availability = sub_reroute_availability_.take_data();
    if (!reroute_availability || !reroute_availability->availability) {
      res->status.success = false;
      throw service_utils::ServiceException(
        ResponseCode::ERROR_INVALID_STATE,
        "Cannot reroute as the planner is not in lane following.");
    }
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);

  const DIRECTION override_direction = req->lane_change_direction == 0   ? DIRECTION::MANUAL_LEFT
                                       : req->lane_change_direction == 1 ? DIRECTION::MANUAL_RIGHT
                                                                         : DIRECTION::AUTO;

  lanelet::ConstLanelet closest_lanelet;
  const bool found_closest_lane = planner_->getRouteHandler().getClosestLaneletWithinRoute(
    odometry_->pose.pose, &closest_lanelet);

  if (!found_closest_lane) {
    res->status.success = false;
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "Failed to find closest lanelet.");
  }

  const LaneChangeRequestResult lane_change_request_result =
    manual_lane_change_handler_.process_lane_change_request(closest_lanelet.id(), req);
  auto route = lane_change_request_result.route;

  res->status.message = lane_change_request_result.message;

  if (!lane_change_request_result.success) {
    res->status.success = false;
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    return;
  }

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  // trim route from the current position
  const auto segment_it =
    std::find_if(route.segments.begin(), route.segments.end(), [&](const auto & segment) {
      return std::any_of(
        segment.primitives.begin(), segment.primitives.end(), [&](const auto & primitive) {
          const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
          return lanelet::utils::isInLanelet(route.start_pose, lanelet);
        });
    });

  // erase segments before the current segment
  if (segment_it != route.segments.end()) {
    route.segments.erase(route.segments.begin(), segment_it);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to find the current segment on the new route.");
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    res->status.success = false;
    throw service_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }
  // Generate a new UUID for the route
  boost::uuids::random_generator gen;
  boost::uuids::uuid uuid = gen();
  std::copy(uuid.begin(), uuid.end(), route.uuid.uuid.begin());

  change_route(route, override_direction != DIRECTION::AUTO);
  change_state(RouteState::SET);

  res->status.success = true;
}

std::vector<autoware_planning_msgs::msg::LaneletPrimitive>
MissionPlanner::sortPrimitivesLeftToRight(
  const route_handler::RouteHandler & route_handler,
  autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives)
{
  using Primitive = autoware_planning_msgs::msg::LaneletPrimitive;

  std::deque<Primitive> sorted_primitives;

  auto find_primitive = [&](lanelet::Id id) -> std::optional<Primitive> {
    auto it = std::find_if(
      primitives.begin(), primitives.end(), [&](const Primitive & p) { return p.id == id; });
    if (it != primitives.end()) return *it;
    return std::nullopt;
  };

  lanelet::ConstLanelet current = route_handler.getLaneletsFromId(preferred_primitive.id);
  // Walk left lanes
  for (auto left = route_handler.getLeftLanelet(current, true); left;
       left = route_handler.getLeftLanelet(*left, true)) {
    if (auto match = find_primitive(left->id())) {
      sorted_primitives.push_front(*match);
    }
  }

  sorted_primitives.push_back(preferred_primitive);

  // Walk right lanes
  for (auto right = route_handler.getRightLanelet(current, true); right;
       right = route_handler.getRightLanelet(*right, true, true)) {
    if (auto match = find_primitive(right->id())) {
      sorted_primitives.push_back(*match);
    }
  }

  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> result{
    sorted_primitives.begin(), sorted_primitives.end()};

  return result;
}

void MissionPlanner::on_set_lanelet_route(
  const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoute::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE,
      fmt::format(
        "The lanelet route cannot be set in the current state: {}",
        route_state_to_string(state_.state)));
  }
  if (!is_mission_planner_ready_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
  }
  if (is_reroute && !operation_mode_state_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && !allow_reroute_in_autonomous_mode_ && is_autonomous_driving) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE, "Reroute is not allowed in autonomous mode.");
  }

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_availability = sub_reroute_availability_.take_data();
    if (!reroute_availability || !reroute_availability->availability) {
      throw service_utils::ServiceException(
        ResponseCode::ERROR_INVALID_STATE,
        "Cannot reroute as the planner is not in lane following.");
    }
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  auto route = create_route(*req);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  // trim route from the current position

  const auto segment_it =
    std::find_if(route.segments.begin(), route.segments.end(), [&](const auto & segment) {
      return std::any_of(
        segment.primitives.begin(), segment.primitives.end(), [&](const auto & primitive) {
          const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
          return lanelet::utils::isInLanelet(odometry_->pose.pose, lanelet);
        });
    });

  // erase segments before the current segment
  if (segment_it != route.segments.end()) {
    route.segments.erase(route.segments.begin(), segment_it);
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }

  // --- Sort segments left-to-right at the very beginning ---
  auto route_handler = planner_->getRouteHandler();

  for (auto & segment : route.segments) {
    segment.primitives =
      sortPrimitivesLeftToRight(route_handler, segment.preferred_primitive, segment.primitives);
  }

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  print_pose_log("set_lanelet_route", odometry_->pose.pose, req->goal_pose);
}

void MissionPlanner::on_set_waypoint_route(
  const SetWaypointRoute::Request::SharedPtr req, const SetWaypointRoute::Response::SharedPtr res)
{
  using ResponseCode = autoware_adapi_v1_msgs::srv::SetRoutePoints::Response;
  const auto is_reroute = state_.state == RouteState::SET;

  if (state_.state != RouteState::UNSET && state_.state != RouteState::SET) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_INVALID_STATE,
      fmt::format(
        "The waypoint route cannot be set in the current state: {}",
        route_state_to_string(state_.state)));
  }
  if (!is_mission_planner_ready_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "The mission planner is not ready.");
  }
  if (is_reroute && !operation_mode_state_) {
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_UNREADY, "Operation mode state is not received.");
  }

  const bool is_autonomous_driving =
    operation_mode_state_ ? operation_mode_state_->mode == OperationModeState::AUTONOMOUS &&
                              operation_mode_state_->is_autoware_control_enabled
                          : false;

  if (is_reroute && is_autonomous_driving) {
    const auto reroute_availability = sub_reroute_availability_.take_data();
    if (!reroute_availability || !reroute_availability->availability) {
      throw service_utils::ServiceException(
        ResponseCode::ERROR_INVALID_STATE,
        "Cannot reroute as the planner is not in lane following.");
    }
  }

  change_state(is_reroute ? RouteState::REROUTING : RouteState::ROUTING);
  const auto route = create_route(*req);

  if (route.segments.empty()) {
    cancel_route();
    change_state(is_reroute ? RouteState::SET : RouteState::UNSET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_PLANNER_FAILED, "The planned route is empty.");
  }

  if (is_reroute && is_autonomous_driving && !check_reroute_safety(*current_route_, route)) {
    cancel_route();
    change_state(RouteState::SET);
    throw service_utils::ServiceException(
      ResponseCode::ERROR_REROUTE_FAILED, "New route is not safe. Reroute failed.");
  }

  // Reset manual lane change handler
  manual_lane_change_handler_.reset();

  change_route(route);
  change_state(RouteState::SET);
  res->status.success = true;

  print_pose_log("set_waypoint_route", odometry_->pose.pose, req->goal_pose);
}

void MissionPlanner::change_route()
{
  current_route_ = nullptr;
  planner_->clearRoute();
  arrival_checker_.set_goal();

  // TODO(Takagi, Isamu): publish an empty route here
  // pub_route_->publish();
  // pub_marker_->publish();
}

void MissionPlanner::change_route(const LaneletRoute & route, bool emphasise_goal_lanes)
{
  PoseWithUuidStamped goal;
  goal.header = route.header;
  goal.pose = route.goal_pose;
  goal.uuid = route.uuid;

  current_route_ = std::make_shared<LaneletRoute>(route);
  planner_->updateRoute(route);
  arrival_checker_.set_goal(goal);

  pub_route_->publish(route);
  pub_marker_->publish(planner_->visualize(route, emphasise_goal_lanes));
}

void MissionPlanner::cancel_route()
{
  // Restore planner state that changes with create_route function.
  if (current_route_) {
    planner_->updateRoute(*current_route_);
  }
}

LaneletRoute MissionPlanner::create_route(const SetLaneletRoute::Request & req)
{
  const auto & header = req.header;
  const auto & segments = req.segments;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(header, segments, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(const SetWaypointRoute::Request & req)
{
  const auto & header = req.header;
  const auto & waypoints = req.waypoints;
  const auto & goal_pose = req.goal_pose;
  const auto & uuid = req.uuid;
  const auto & allow_goal_modification = req.allow_modification;

  return create_route(
    header, waypoints, odometry_->pose.pose, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(const PoseWithUuidStamped & msg)
{
  const auto & header = msg.header;
  const auto & goal_pose = msg.pose;
  const auto & uuid = msg.uuid;
  const auto & allow_goal_modification = current_route_->allow_modification;

  // NOTE: Reroute by modifed goal is assumed to be a slight modification near the goal lane.
  //       It is assumed that ego and goal are on the extension of the current route at least.
  //       Therefore, the start pose is the start pose of the current route if it exists.
  //       This prevents the route from becoming shorter due to reroute.
  //       Also, use start pose and waypoints that are on the preferred lanelet of the current route
  //       as much as possible.
  //       For this process, refer to RouteHandler::planPathLaneletsBetweenCheckpoints() or
  //       https://github.com/autowarefoundation/autoware_universe/pull/8238 too.
  const auto & start_pose = current_route_ ? current_route_->start_pose : odometry_->pose.pose;
  std::vector<Pose> waypoints{};
  if (current_route_) {
    waypoints.push_back(odometry_->pose.pose);
  }

  return create_route(header, waypoints, start_pose, goal_pose, uuid, allow_goal_modification);
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
  const UUID & uuid, const bool allow_goal_modification)
{
  LaneletRoute route;
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.start_pose = odometry_->pose.pose;
  route.goal_pose = transform_pose(goal_pose, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

LaneletRoute MissionPlanner::create_route(
  const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
  const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification)
{
  PlannerPlugin::RoutePoints points;
  points.push_back(start_pose);
  for (const auto & waypoint : waypoints) {
    points.push_back(transform_pose(waypoint, header));
  }
  points.push_back(transform_pose(goal_pose, header));

  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;
  return route;
}

bool MissionPlanner::check_reroute_safety(
  const LaneletRoute & original_route, const LaneletRoute & target_route)
{
  if (
    original_route.segments.empty() || target_route.segments.empty() || !map_ptr_ ||
    !lanelet_map_ptr_ || !odometry_) {
    RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Route, map or odometry is not set.");
    return false;
  }

  const auto current_velocity = odometry_->twist.twist.linear.x;

  // if vehicle is stopped, do not check safety
  if (current_velocity < 0.01) {
    return true;
  }

  auto hasSamePrimitives = [](
                             const std::vector<LaneletPrimitive> & original_primitives,
                             const std::vector<LaneletPrimitive> & target_primitives) {
    if (original_primitives.size() != target_primitives.size()) {
      return false;
    }

    for (const auto & primitive : original_primitives) {
      const auto has_same = [&](const auto & p) { return p.id == primitive.id; };
      const bool is_same =
        std::find_if(target_primitives.begin(), target_primitives.end(), has_same) !=
        target_primitives.end();
      if (!is_same) {
        return false;
      }
    }
    return true;
  };

  // =============================================================================================
  // NOTE: the target route is calculated while ego is driving on the original route, so basically
  // the first lane of the target route should be in the original route lanelets. So the common
  // segment interval matches the beginning of the target route. The exception is that if ego is
  // on an intersection lanelet, getClosestLanelet() may not return the same lanelet which exists
  // in the original route. In that case the common segment interval does not match the beginning of
  // the target lanelet
  // =============================================================================================
  const auto start_idx_opt =
    std::invoke([&]() -> std::optional<std::pair<size_t /* original */, size_t /* target */>> {
      for (size_t i = 0; i < original_route.segments.size(); ++i) {
        const auto & original_segment = original_route.segments.at(i).primitives;
        for (size_t j = 0; j < target_route.segments.size(); ++j) {
          const auto & target_segment = target_route.segments.at(j).primitives;
          if (hasSamePrimitives(original_segment, target_segment)) {
            return std::make_pair(i, j);
          }
        }
      }
      return std::nullopt;
    });
  if (!start_idx_opt.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "Check reroute safety failed. Cannot find the start index of the route.");
    return false;
  }
  const auto [start_idx_original, start_idx_target] = start_idx_opt.value();

  // find last idx that matches the target primitives
  size_t end_idx_original = start_idx_original;
  size_t end_idx_target = start_idx_target;
  for (size_t i = 1; i < target_route.segments.size() - start_idx_target; ++i) {
    if (start_idx_original + i > original_route.segments.size() - 1) {
      break;
    }

    const auto & original_primitives =
      original_route.segments.at(start_idx_original + i).primitives;
    const auto & target_primitives = target_route.segments.at(start_idx_target + i).primitives;
    if (!hasSamePrimitives(original_primitives, target_primitives)) {
      break;
    }
    end_idx_original = start_idx_original + i;
    end_idx_target = start_idx_target + i;
  }

  // at the very first transition from main/MRM to MRM/main, the requested route from the
  // route_selector may not begin from ego current lane (because route_selector requests the
  // previous route once, and then replan)
  const bool ego_is_on_first_target_section = std::any_of(
    target_route.segments.front().primitives.begin(),
    target_route.segments.front().primitives.end(), [&](const auto & primitive) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      return lanelet::utils::isInLanelet(target_route.start_pose, lanelet);
    });
  if (!ego_is_on_first_target_section) {
    RCLCPP_ERROR(
      get_logger(),
      "Check reroute safety failed. Ego is not on the first section of target route.");
    return false;
  }

  // if the front of target route is not the front of common segment, it is expected that the front
  // of the target route is conflicting with another lane which is equal to original
  // route[start_idx_original-1]
  double accumulated_length = 0.0;

  if (start_idx_target != 0 && start_idx_original > 1) {
    // compute distance from the current pose to the beginning of the common segment
    const auto current_pose = target_route.start_pose;
    const auto primitives = original_route.segments.at(start_idx_original - 1).primitives;
    lanelet::ConstLanelets start_lanelets;
    for (const auto & primitive : primitives) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      start_lanelets.push_back(lanelet);
    }
    // closest lanelet in start lanelets
    lanelet::ConstLanelet closest_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(start_lanelets, current_pose, &closest_lanelet)) {
      RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Cannot find the closest lanelet.");
      return false;
    }

    const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
    const double dist_to_current_pose = arc_coordinates.length;
    const double lanelet_length = lanelet::utils::getLaneletLength2d(closest_lanelet);
    accumulated_length = lanelet_length - dist_to_current_pose;
  } else {
    // compute distance from the current pose to the end of the current lanelet
    const auto current_pose = target_route.start_pose;
    const auto primitives = original_route.segments.at(start_idx_original).primitives;
    lanelet::ConstLanelets start_lanelets;
    for (const auto & primitive : primitives) {
      const auto lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      start_lanelets.push_back(lanelet);
    }
    // closest lanelet in start lanelets
    lanelet::ConstLanelet closest_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(start_lanelets, current_pose, &closest_lanelet)) {
      RCLCPP_ERROR(get_logger(), "Check reroute safety failed. Cannot find the closest lanelet.");
      return false;
    }

    const auto & centerline_2d = lanelet::utils::to2D(closest_lanelet.centerline());
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
    const auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      centerline_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
    const double dist_to_current_pose = arc_coordinates.length;
    const double lanelet_length = lanelet::utils::getLaneletLength2d(closest_lanelet);
    accumulated_length = lanelet_length - dist_to_current_pose;
  }

  // compute distance from the start_idx+1 to end_idx
  for (size_t i = start_idx_original + 1; i <= end_idx_original; ++i) {
    const auto primitives = original_route.segments.at(i).primitives;
    if (primitives.empty()) {
      break;
    }

    std::vector<double> lanelets_length(primitives.size());
    for (size_t primitive_idx = 0; primitive_idx < primitives.size(); ++primitive_idx) {
      const auto & primitive = primitives.at(primitive_idx);
      const auto & lanelet = lanelet_map_ptr_->laneletLayer.get(primitive.id);
      lanelets_length.at(primitive_idx) = (lanelet::utils::getLaneletLength2d(lanelet));
    }
    accumulated_length += *std::min_element(lanelets_length.begin(), lanelets_length.end());
  }

  // check if the goal is inside of the target terminal lanelet
  const auto & target_end_primitives = target_route.segments.at(end_idx_target).primitives;
  const auto & target_goal = target_route.goal_pose;
  for (const auto & target_end_primitive : target_end_primitives) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(target_end_primitive.id);
    if (lanelet::utils::isInLanelet(target_goal, lanelet)) {
      const auto target_goal_position =
        lanelet::utils::conversion::toLaneletPoint(target_goal.position);
      const double dist_to_goal = lanelet::geometry::toArcCoordinates(
                                    lanelet::utils::to2D(lanelet.centerline()),
                                    lanelet::utils::to2D(target_goal_position).basicPoint())
                                    .length;
      const double target_lanelet_length = lanelet::utils::getLaneletLength2d(lanelet);
      // NOTE: `accumulated_length` here contains the length of the entire target_end_primitive, so
      // the remaining distance from the goal to the end of the target_end_primitive needs to be
      // subtracted.
      const double remaining_dist = target_lanelet_length - dist_to_goal;
      accumulated_length = std::max(accumulated_length - remaining_dist, 0.0);
      break;
    }
  }

  // check safety
  const double safety_length =
    std::max(current_velocity * reroute_time_threshold_, minimum_reroute_length_);
  if (accumulated_length > safety_length) {
    return true;
  }

  RCLCPP_WARN(
    get_logger(),
    "Length of lane where original and B target (= %f) is less than safety length (= %f), so "
    "reroute is not safe.",
    accumulated_length, safety_length);
  return false;
}
}  // namespace autoware::mission_planner_universe

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner_universe::MissionPlanner)
