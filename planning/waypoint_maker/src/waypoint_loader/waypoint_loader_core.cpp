#include "waypoint_loader/waypoint_loader_core.h"

WaypointLoaderNode::WaypointLoaderNode()
: Node("waypoint_loader"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // param
  max_velocity_ = waitForParam<double>(
    this,
    declare_parameter(
      "node_name_for_max_velocity", "/planning/scenario_planning/velocity_smoother"),
    declare_parameter("param_name_for_max_velocity", "max_vel"));

  lane_csv_position_ = declare_parameter(
    "lane_csv_position", "./src/universe/autoware.universe/planning/waypoint_maker/log/");
  bound_margin_ = declare_parameter("bound_margin", 1.0);
  update_rate_ = declare_parameter("update_rate", 10.0);
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  distance_threshold_ = declare_parameter("check_distance", 0.5);
  angle_threshold_ = autoware_utils::deg2rad(declare_parameter("check_angle_deg", 45.0));
  stop_duration_ = declare_parameter("stop_duration", 1.0);

  // subscriber
  file_subscriber_ = create_subscription<std_msgs::msg::String>(
    "/waypoint_loader/input_file_path", 1, std::bind(&WaypointLoaderNode::csvFileCallback, this, std::placeholders::_1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "/input/pose", rclcpp::QoS(1),
    std::bind(&WaypointLoaderNode::onOdometry, this, std::placeholders::_1));
  sub_scenario_ = create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
    "/input/scenario", rclcpp::QoS(1),
    std::bind(&WaypointLoaderNode::onScenario, this, std::placeholders::_1));

  // publish
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  path_pub_ = create_publisher<autoware_planning_msgs::msg::Path>("output/path", rclcpp::QoS{1});
  path_visualization_pub_ = create_publisher<autoware_planning_msgs::msg::Path>(
    "output/path_visualization", rclcpp::QoS{1});
  file_location_pub_ = create_publisher<std_msgs::msg::String>(
    "/file/location", rclcpp::QoS(1).transient_local().reliable());
  waypoint_following_state_pub_ =
    create_publisher<std_msgs::msg::Bool>("output/waypoint_following_state", rclcpp::QoS{1});

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_route_);

  // Init variables
  stop_time_ = {};
  state_ = 0;
  id_flip_ = 0;
  csv_file_ = lane_csv_position_ + "waypoint_default.csv";
  readCsvFile();

  // Callback Groups
  callback_group_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&WaypointLoaderNode::run, this),
    callback_group_subscribers_);

  std_msgs::msg::String file_location;
  file_location.data = lane_csv_position_;
  file_location_pub_->publish(file_location);
}

void WaypointLoaderNode::run()
{
  // if arrived, clear output_trajectory_ and return;
  if (
    route_state_.state ==
    tier4_planning_msgs::msg::RouteState::ARRIVED) {
    ouput_trajectory_.header.stamp = this->now();
    ouput_trajectory_.points.clear();

    return;
  }

  // return if csv file reading fails or is empty
  if (basic_trajectory_.points.empty()) {
    return;
  }

  // publish visualized trajectory
  path_visualization_pub_->publish(convertTrajectoryToPath(basic_trajectory_));

  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::WAYPOINTFOLLOWING) {
    // set start pose
    if (!getEgoVehiclePose(&start_pose_)) {
      RCLCPP_ERROR(get_logger(), "Failed to get ego vehicle pose in map frame.");
      return;
    }

    // generate output trajectory based on start_pose
    if (ouput_trajectory_.points.empty()) {
      ouput_trajectory_ = generateOutputTrajectory(
        basic_trajectory_, start_pose_, distance_threshold_, angle_threshold_);
    }

    // publish path or trajectory if output_trajectory is generated
    ouput_trajectory_.header.stamp = this->now();
    path_pub_->publish(convertTrajectoryToPath(ouput_trajectory_));
  } else {
    // clear ouput_trajectory_ if is not in waypoint following
    ouput_trajectory_.header.stamp = this->now();
    ouput_trajectory_.points.clear();
  }
}

autoware_planning_msgs::msg::Trajectory WaypointLoaderNode::generateOutputTrajectory(
  autoware_planning_msgs::msg::Trajectory input_trajectory,
  geometry_msgs::msg::PoseStamped start_pose, double distance_threshold, double angle_threshold)
{
  if (input_trajectory.points.empty()) {
    RCLCPP_WARN(get_logger(), "Input trajectory is empty.");
    return autoware_planning_msgs::msg::Trajectory();
  }

  // Find the closest point
  size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < input_trajectory.points.size(); ++i) {
    double dist = std::hypot(
      input_trajectory.points.at(i).pose.position.x - start_pose.pose.position.x,
      input_trajectory.points.at(i).pose.position.y - start_pose.pose.position.y);

    if (dist < min_distance) {
      min_distance = dist;
      closest_index = i;
    }
  }

  if (min_distance > distance_threshold) {
    RCLCPP_WARN(rclcpp::get_logger("waypoint_loader"), "Start pose is too far from trajectory.");
    return autoware_planning_msgs::msg::Trajectory();
  }

  // Generate both forward and reversed trajectories
  autoware_planning_msgs::msg::Trajectory forward_trajectory = input_trajectory;
  autoware_planning_msgs::msg::Trajectory reverse_trajectory = input_trajectory;
  std::reverse(reverse_trajectory.points.begin(), reverse_trajectory.points.end());

  // Update orientations for reversed trajectory
  // Note: waypoint loader only works for forward driving
  autoware::motion_utils::insertOrientation(reverse_trajectory.points, true);

  // Check yaw alignment for both
  double yaw_start = tf2::getYaw(start_pose.pose.orientation);
  double yaw_fwd = tf2::getYaw(forward_trajectory.points.at(closest_index).pose.orientation);
  double yaw_rev = tf2::getYaw(reverse_trajectory.points.at(closest_index).pose.orientation);

  double yaw_diff_fwd = std::atan2(std::sin(yaw_start - yaw_fwd), std::cos(yaw_start - yaw_fwd));
  double yaw_diff_rev = std::atan2(std::sin(yaw_start - yaw_rev), std::cos(yaw_start - yaw_rev));

  bool is_fwd_aligned = (std::fabs(yaw_diff_fwd) <= angle_threshold);
  bool is_rev_aligned = (std::fabs(yaw_diff_rev) <= angle_threshold);

  // Choose the suitable trajectory based on direction
  if (is_fwd_aligned) {
    return forward_trajectory;
  } else if (is_rev_aligned) {
    return reverse_trajectory;
  } else {
    return autoware_planning_msgs::msg::Trajectory();
  }
}

autoware_planning_msgs::msg::Path WaypointLoaderNode::convertTrajectoryToPath(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  autoware_planning_msgs::msg::Path output_path;
  output_path.header = trajectory.header;

  for (size_t i = 0; i < trajectory.points.size(); i++) {
    autoware_planning_msgs::msg::PathPoint path_point;
    path_point.longitudinal_velocity_mps = trajectory.points.at(i).longitudinal_velocity_mps;
    path_point.pose = trajectory.points.at(i).pose;

    output_path.points.push_back(path_point);
  }

  generateDrivableArea(
    output_path, vehicle_info.vehicle_length_m, vehicle_info.vehicle_width_m + bound_margin_, true);

  return output_path;
}

void WaypointLoaderNode::generateDrivableArea(
  autoware_planning_msgs::msg::Path & path, const double vehicle_length, const double offset,
  const bool is_driving_forward)
{
  if (path.points.empty()) {
    return;
  }

  // create bound point by calculating offset point
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
  for (const auto & point : path.points) {
    const auto & pose = point.pose;

    const auto left_point = autoware::universe_utils::calcOffsetPose(pose, 0, offset, 0);
    const auto right_point = autoware::universe_utils::calcOffsetPose(pose, 0, -offset, 0);

    left_bound.push_back(left_point.position);
    right_bound.push_back(right_point.position);
  }

  if (is_driving_forward) {
    // add backward offset point to bound
    const auto first_point =
      autoware::universe_utils::calcOffsetPose(path.points.front().pose, -vehicle_length, 0, 0);
    const auto left_first_point =
      autoware::universe_utils::calcOffsetPose(first_point, 0, offset, 0);
    const auto right_first_point =
      autoware::universe_utils::calcOffsetPose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add forward offset point to bound
    const auto last_point =
      autoware::universe_utils::calcOffsetPose(path.points.back().pose, vehicle_length, 0, 0);
    const auto left_last_point = autoware::universe_utils::calcOffsetPose(last_point, 0, offset, 0);
    const auto right_last_point =
      autoware::universe_utils::calcOffsetPose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  } else {
    // add forward offset point to bound
    const auto first_point =
      autoware::universe_utils::calcOffsetPose(path.points.front().pose, vehicle_length, 0, 0);
    const auto left_first_point =
      autoware::universe_utils::calcOffsetPose(first_point, 0, offset, 0);
    const auto right_first_point =
      autoware::universe_utils::calcOffsetPose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add backward offset point to bound
    const auto last_point =
      autoware::universe_utils::calcOffsetPose(path.points.back().pose, -vehicle_length, 0, 0);
    const auto left_last_point = autoware::universe_utils::calcOffsetPose(last_point, 0, offset, 0);
    const auto right_last_point =
      autoware::universe_utils::calcOffsetPose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  }

  if (left_bound.empty() || right_bound.empty()) {
    return;
  }

  // fix intersected bound
  // if bound is intersected, remove them and insert intersection point
  typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
  typedef boost::geometry::model::linestring<BoostPoint> LineString;
  auto modify_bound_intersection = [](const std::vector<geometry_msgs::msg::Point> & bound) {
    const double intersection_check_distance = 10.0;
    std::vector<geometry_msgs::msg::Point> modified_bound;
    size_t i = 0;
    while (i < bound.size() - 1) {
      BoostPoint p1(bound.at(i).x, bound.at(i).y);
      BoostPoint p2(bound.at(i + 1).x, bound.at(i + 1).y);
      LineString p_line;
      p_line.push_back(p1);
      p_line.push_back(p2);
      bool intersection_found = false;
      for (size_t j = i + 2; j < bound.size() - 1; j++) {
        const double distance = autoware::universe_utils::calcDistance2d(bound.at(i), bound.at(j));
        if (distance > intersection_check_distance) {
          break;
        }
        LineString q_line;
        BoostPoint q1(bound.at(j).x, bound.at(j).y);
        BoostPoint q2(bound.at(j + 1).x, bound.at(j + 1).y);
        q_line.push_back(q1);
        q_line.push_back(q2);
        std::vector<BoostPoint> intersection_points;
        boost::geometry::intersection(p_line, q_line, intersection_points);
        if (intersection_points.size() > 0) {
          modified_bound.push_back(bound.at(i));
          geometry_msgs::msg::Point intersection_point;
          intersection_point.x = intersection_points.at(0).x();
          intersection_point.y = intersection_points.at(0).y();
          modified_bound.push_back(intersection_point);
          i = j + 1;
          intersection_found = true;
          break;
        }
      }
      if (!intersection_found) {
        modified_bound.push_back(bound.at(i));
        i++;
      }
    }
    modified_bound.push_back(bound.back());
    return modified_bound;
  };
  std::vector<geometry_msgs::msg::Point> modified_left_bound =
    modify_bound_intersection(left_bound);
  std::vector<geometry_msgs::msg::Point> modified_right_bound =
    modify_bound_intersection(right_bound);

  // set bound to path
  path.left_bound = modified_left_bound;
  path.right_bound = modified_right_bound;
}

void WaypointLoaderNode::readCsvFile()
{
  basic_trajectory_ = waypoint_maker::createWaypointTrajectory(csv_file_, true, max_velocity_);
  publishRoute(basic_trajectory_, id_flip_ = id_flip_ == 0 ? 1 : 0);
}

void WaypointLoaderNode::csvFileCallback(std_msgs::msg::String::ConstSharedPtr msg)
{
  std::string default_csv_file = lane_csv_position_ + "waypoint_default.csv";
  csv_file_ = msg->data;

  if (!std::filesystem::equivalent(csv_file_, default_csv_file)){
    RCLCPP_INFO(get_logger(), "Receive a new file. Reloading");
    readCsvFile();

    std::filesystem::copy(
      csv_file_, default_csv_file, std::filesystem::copy_options::overwrite_existing);
    RCLCPP_INFO_STREAM(this->get_logger(), "The default output csv file has been copied");
  }
}

template <class T>
T WaypointLoaderNode::waitForParam(
  rclcpp::Node * node, const std::string & remote_node_name, const std::string & param_name)
{
  std::chrono::seconds sec(1);

  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

  while (!param_client->wait_for_service(sec)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
      return {};
    }
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000 /* ms */, "waiting for node: %s, param: %s\n",
      remote_node_name.c_str(), param_name.c_str());
  }

  if (param_client->has_parameter(param_name)) {
    return param_client->get_parameter<T>(param_name);
  }

  return {};
}

void WaypointLoaderNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  bool is_empty_output_trajectory = ouput_trajectory_.points.empty();

  // check arrival
  bool is_arrival = false;
  if (!is_empty_output_trajectory) {
    const auto goal = ouput_trajectory_.points.back().pose;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    is_arrival = isArrival(pose, goal);
  }

  // check vehicle stopped
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  bool is_vehicle_stopped = isVehicleStopped(twist);

  // check waypoint complete
  std_msgs::msg::Bool state;
  if (current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::WAYPOINTFOLLOWING) {
    if ((is_arrival || is_empty_output_trajectory) && is_vehicle_stopped) {
      state.data = true;
    }

  } else {
    state.data = false;
  }

  waypoint_following_state_pub_->publish(state);
}

void WaypointLoaderNode::onScenario(const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr msg)
{
  if (msg) {
    current_scenario_ = msg->current_scenario;
  }
}

void WaypointLoaderNode::publishRoute(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const signed short & id_flip)
{
  if (trajectory.points.empty()) {
    return;
  }

  autoware_planning_msgs::msg::LaneletSegment s;
  s.preferred_primitive.id = id_flip;

  autoware_planning_msgs::msg::LaneletRoute route_msg;
  route_msg.header.frame_id = map_frame_;
  route_msg.header.stamp = this->now();

  route_msg.start_pose = trajectory.points.front().pose;
  route_msg.goal_pose = trajectory.points.back().pose;
  route_msg.segments.push_back(s);

  pub_route_->publish(route_msg);
}

bool WaypointLoaderNode::getEgoVehiclePose(geometry_msgs::msg::PoseStamped * ego_vehicle_pose)
{
  geometry_msgs::msg::PoseStamped base_link_origin;
  base_link_origin.header.frame_id = base_link_frame_;

  //  transform base_link frame origin to map_frame to get vehicle positions
  return transformPose(base_link_origin, ego_vehicle_pose, map_frame_);
}

bool WaypointLoaderNode::transformPose(
  const geometry_msgs::msg::PoseStamped & input_pose, geometry_msgs::msg::PoseStamped * output_pose,
  const std::string target_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input_pose, *output_pose, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return false;
  }
}

bool WaypointLoaderNode::isArrival(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Pose & point)
{
  // Check distance
  if (distance_threshold_ < autoware::universe_utils::calcDistance2d(pose, point)) return false;

  // Check angle
  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_point = tf2::getYaw(point.orientation);
  const double yaw_diff = autoware::universe_utils::normalizeRadian(yaw_pose - yaw_point);
  if (angle_threshold_ < std::fabs(yaw_diff)) return false;

  return true;
}

bool WaypointLoaderNode::isVehicleStopped(const geometry_msgs::msg::TwistStamped & twist)
{
  constexpr double squared_stop_velocity = 1e-3 * 1e-3;
  const auto now = this->now();

  double velocity_sq = std::hypot(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);

  if (velocity_sq <= squared_stop_velocity) {  // Vehicle is stopped
    if (!stop_time_) {
      stop_time_ = now;  // Start counting stop duration
      return false;
    }

    // Check if the vehicle has been stopped for the required duration
    if ((now - *stop_time_).seconds() > stop_duration_) {
      return true;
    }
  } else {
    stop_time_.reset();  // Reset stop timer if vehicle moves again
  }

  return false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointLoaderNode>();
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}
