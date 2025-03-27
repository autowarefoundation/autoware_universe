#ifndef WAYPOINT_LOADER_PANEL_HPP_
#define WAYPOINT_LOADER_PANEL_HPP_

#include <QDir>
#include <QMessageBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLayout>
#include <qt5/QtWidgets/QListWidget>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>


#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <tier4_planning_msgs/srv/save_waypoint.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <string>

namespace rviz_plugins
{
class WaypointLoaderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointLoaderPanel(QWidget * parent = nullptr);
  ~WaypointLoaderPanel() override;
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickRecord();

protected:
  std::string file_position_;

  QLabel * file_label_{nullptr};
  QListWidget * file_list_{nullptr};
  QPushButton * load_button_{nullptr};

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Service<tier4_planning_msgs::srv::SaveWaypoint>::SharedPtr srv_record_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr sub_motion_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_log_location_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_file_pub_;

  void loadeFiles(const QString & folderPath);
  void onLoadSelectedFile();

  void onRecordService(const tier4_planning_msgs::srv::SaveWaypoint::Request::SharedPtr request);

  void onMotionState(const autoware_adapi_v1_msgs::msg::MotionState::ConstSharedPtr msg);

  static void activateButton(QAbstractButton * button)
  {
    button->setChecked(false);
    button->setEnabled(true);
  }

  static void deactivateButton(QAbstractButton * button)
  {
    button->setChecked(true);
    button->setEnabled(false);
  }
};
}  // namespace rviz_plugins

#endif  // WAYPOINT_LOADER_PANEL_HPP_