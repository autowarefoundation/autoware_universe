#include "include/waypoint_saver_panel.hpp"

#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QHeaderView>
#include <rviz_common/display_context.hpp>

#include <memory>

namespace rviz_plugins
{
WaypointSaverPanel::WaypointSaverPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Create a label for the spin box
  QLabel * interval_label = new QLabel("Distance Interval:");

  // interval spin box
  {
    interval_input_ = new QDoubleSpinBox();
    interval_input_->setMinimum(0.0);
    interval_input_->setValue(INTERVAL_DEFAULT);
    interval_input_->setSingleStep(0.1);
    interval_input_->setSuffix(" [m]");
  }

  // save button
  {
    save_button_ = new QPushButton("Save");
    save_button_->setToolTip("Recording trajectory.");
    connect(save_button_, SIGNAL(clicked()), SLOT(onClickRecord()));
  }

  // cancel button
  {
    finish_button_ = new QPushButton("Finish");
    finish_button_->setToolTip("Stop recording trajectory.");
    deactivateButton(finish_button_);
    connect(finish_button_, SIGNAL(clicked()), SLOT(onClickCancel()));
  }

  // file lable
  {
    file_label_ = new QLabel("");
  }

  // Create the main vertical layout
  auto * main_layout = new QVBoxLayout(this);

  // Add file_table_ at the top
  main_layout->addWidget(file_label_);

  // Create a grid layout for buttons and the interval input
  auto * bottom_layout = new QGridLayout();
  bottom_layout->addWidget(save_button_, 0, 0);
  bottom_layout->addWidget(interval_label, 0, 1);
  bottom_layout->addWidget(interval_input_, 0, 2);
  bottom_layout->addWidget(finish_button_, 1, 0);

  // Add the bottom layout to the main vertical layout
  main_layout->addLayout(bottom_layout);

  // Set the layout for the panel
  setLayout(main_layout);
}

WaypointSaverPanel::~WaypointSaverPanel()
{
}

void WaypointSaverPanel::onInitialize()
{
  if (!raw_node_) {
    raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  using std::placeholders::_1;
  client_record_ = raw_node_->create_client<tier4_planning_msgs::srv::SaveWaypoint>(
    "/waypoint_maker/record", rmw_qos_profile_services_default);
}

void WaypointSaverPanel::onClickRecord()
{
  // Call the service
  callServiceWithoutResponse(client_record_, true);
}

void WaypointSaverPanel::onClickCancel()
{
  // Call the service
  callServiceWithoutResponse(client_record_, false);
}

void WaypointSaverPanel::callServiceWithoutResponse(
  rclcpp::Client<tier4_planning_msgs::srv::SaveWaypoint>::SharedPtr client, bool && mode)
{
  auto req = std::make_shared<tier4_planning_msgs::srv::SaveWaypoint::Request>();

  req->mode = mode;
  req->interval = interval_input_->value();

  RCLCPP_INFO(raw_node_->get_logger(), "client request");

  if (!client->service_is_ready()) {
    RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
    return;
  }

  client->async_send_request(
    req, [this, mode](rclcpp::Client<tier4_planning_msgs::srv::SaveWaypoint>::SharedFuture result) {
      try {
        auto response = result.get();  // get service response
        if (!response->success) {
          RCLCPP_WARN(raw_node_->get_logger(), "Service call failed.");
          restoreButtonState(mode);
          return;
        }

        // update ui, check Qt in main threads
        QMetaObject::invokeMethod(
          this,
          [this, response, mode]() {
            file_label_->setText(QString::fromStdString(response->file_name));
            updateButtonState(mode);
          },
          Qt::QueuedConnection);

        RCLCPP_INFO(
          raw_node_->get_logger(), "Service call succeeded. File name: %s",
          response->file_name.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(raw_node_->get_logger(), "Service call failed: %s", e.what());
        restoreButtonState(mode);
      }
    });
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::WaypointSaverPanel, rviz_common::Panel)
