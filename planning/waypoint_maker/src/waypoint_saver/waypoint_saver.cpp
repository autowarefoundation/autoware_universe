#include "waypoint_saver/waypoint_saver.h"

WaypointSaver::WaypointSaver(const rclcpp::NodeOptions & node_options)
: Node("waypoint_saver", node_options), self_pose_listener_{this}
{
  // parameter settings
  output_file_str_ = declare_parameter("output_file", "./test.csv");
  if (output_file_str_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Output file not specified, the results will NOT be saved!"
      "Provide output_file parameter to store the results.");
  }
  interval_ = declare_parameter("save_interval", 0.5);
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  record_ = false;

  // subscriber
  pose_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "/input/pose", 1, std::bind(&WaypointSaver::poseCallBack, this, std::placeholders::_1));

  srv_record_ = create_service<tier4_planning_msgs::srv::SaveWaypoint>(
    "/waypoint_maker/record",
    std::bind(&WaypointSaver::onRecordService, this, std::placeholders::_1, std::placeholders::_2));

  // Check folder if not folder create log folder
  auto index = output_file_str_.rfind("/");
  if (!waypoint_maker::dirExists(output_file_str_.substr(0, index))) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Create log folder");
    mkdir(output_file_str_.substr(0, index).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();
}

WaypointSaver::~WaypointSaver()
{
}

void WaypointSaver::poseCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ptr_ = msg;

  if (record_) {
    outputProcessing(msg->pose.pose, waypoint_maker::mps2kmph(msg->twist.twist.linear.x));
  }
}

void WaypointSaver::outputProcessing(
  const geometry_msgs::msg::Pose & current_pose, const double & velocity) const
{
  std::ofstream ofs(output_file_str_backup_.c_str(), std::ios::app);
  static geometry_msgs::msg::Pose previous_pose;
  static bool receive_once = false;
  // first subscribe
  if (!receive_once) {
    ofs << "x,y,z,yaw,velocity" << std::endl;
    ofs << std::setprecision(10) << std::fixed << current_pose.position.x << ","
        << current_pose.position.y << "," << current_pose.position.z << ","
        << tf2::getYaw(current_pose.orientation) << "," << velocity << std::endl;
    receive_once = true;
    previous_pose = current_pose;
  } else {
    double distance = sqrt(
      pow((current_pose.position.x - previous_pose.position.x), 2) +
      pow((current_pose.position.y - previous_pose.position.y), 2));

    // if car moves [interval] meter
    if (distance > interval_) {
      ofs << std::setprecision(10) << std::fixed << current_pose.position.x << ","
          << current_pose.position.y << "," << current_pose.position.z << ","
          << tf2::getYaw(current_pose.orientation) << "," << velocity << std::endl;

      previous_pose = current_pose;
    }
  }
}

void WaypointSaver::onRecordService(
  const tier4_planning_msgs::srv::SaveWaypoint::Request::SharedPtr request,
  const tier4_planning_msgs::srv::SaveWaypoint::Response::SharedPtr response)
{
  record_ = request->mode;
  interval_ = request->interval;

  if (!odom_ptr_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Odometry pointer is not ready.");
    response->file_name = "";
    response->success = false;
    return;
  }

  if (!record_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Cancel record.");

    try {
      std::filesystem::copy(
        output_file_str_backup_, output_file_str_,
        std::filesystem::copy_options::overwrite_existing);
      RCLCPP_INFO_STREAM(this->get_logger(), "The default output csv file has been copied");
      response->file_name = "";
      response->success = true;
    } catch (const std::filesystem::filesystem_error & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), " Save data is fail, can't find back up file.");
      response->file_name = "";
      response->success = true;
    }
  }

  if (record_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "recording");

    // init output csv file
    auto index = output_file_str_.rfind("/");
    output_file_str_backup_ = output_file_str_.substr(0, index) + "/" +
                              waypoint_maker::currentDateTime() + "_" +
                              output_file_str_.substr(index + 1, output_file_str_.length());

    response->file_name = output_file_str_backup_;
    response->success = true;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointSaver>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
