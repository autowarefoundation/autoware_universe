#include "include/waypoint_loader_panel.hpp"

#include <rviz_common/display_context.hpp>

namespace rviz_plugins
{

WaypointLoaderPanel::WaypointLoaderPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Create the main vertical layout
  auto * layout = new QVBoxLayout(this);

  {
    file_label_ = new QLabel("No folder selected");
    layout->addWidget(file_label_);
  }

  {
    file_list_ = new QListWidget();
    layout->addWidget(file_list_);
  }

  {
    load_button_ = new QPushButton("loading");
    load_button_->setToolTip("loading new trajectory");
    layout->addWidget(load_button_);
  }

  connect(load_button_, &QPushButton::clicked, this, &WaypointLoaderPanel::onLoadSelectedFile);
}

WaypointLoaderPanel::~WaypointLoaderPanel()
{
}

void WaypointLoaderPanel::onInitialize()
{
  if (!raw_node_) {
    raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  srv_record_ = raw_node_->create_service<tier4_planning_msgs::srv::SaveWaypoint>(
    "/waypoint_maker/record",
    std::bind(&WaypointLoaderPanel::onRecordService, this, std::placeholders::_1));

  sub_motion_state_ = raw_node_->create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
    "/api/motion/state", 1,
    std::bind(&WaypointLoaderPanel::onMotionState, this, std::placeholders::_1));

  sub_log_location_ = raw_node_->create_subscription<std_msgs::msg::String>(
    "/file/location", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(raw_node_->get_logger(), "Received file location: %s", msg->data.c_str());

      file_position_ = msg->data;
      loadeFiles(QString::fromStdString(file_position_));
    });

  path_file_pub_ = raw_node_->create_publisher<std_msgs::msg::String>("/path_file", rclcpp::QoS{1});
}

void WaypointLoaderPanel::onRecordService(
  const tier4_planning_msgs::srv::SaveWaypoint::Request::SharedPtr request)
{
  if (!request->mode) {
    if (!file_position_.empty()) {
      loadeFiles(QString::fromStdString(file_position_));
    }
  }
}

void WaypointLoaderPanel::onMotionState(
  const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg)
{
  if (msg->state == autoware_adapi_v1_msgs::msg::MotionState::MOVING) {
    deactivateButton(load_button_);
  } else {
    activateButton(load_button_);
  }
}

void WaypointLoaderPanel::loadeFiles(const QString & folderPath)
{
  QDir dir(folderPath);
  if (!dir.exists()) return;

  file_label_->setText("Folder: " + folderPath);
  file_list_->clear();

  QStringList files = dir.entryList(QDir::Files);
  for (const QString & file : files) {
    QListWidgetItem * item = new QListWidgetItem(file, file_list_);
    item->setData(Qt::UserRole, dir.absoluteFilePath(file));
  }
}

void WaypointLoaderPanel::onLoadSelectedFile()
{
  QListWidgetItem * selectedItem = file_list_->currentItem();
  if (selectedItem) {
    QString filePath = selectedItem->data(Qt::UserRole).toString();
    std_msgs::msg::String path_file;
    path_file.data = filePath.toStdString();
    path_file_pub_->publish(path_file);
    std::cout << "Selected file path: " << filePath.toStdString() << std::endl;
  } else {
    QMessageBox::warning(this, "No Selection", "Please select a file from the list.");
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointLoaderPanel, rviz_common::Panel)