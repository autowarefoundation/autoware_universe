#ifndef WAYPOINT_SAVER_PANEL_HPP_
#define WAYPOINT_SAVER_PANEL_HPP_

#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLayout>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <tier4_planning_msgs/srv/save_waypoint.hpp>

#include <cstdint>

namespace rviz_plugins
{
class WaypointSaverPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointSaverPanel(QWidget * parent = nullptr);
  ~WaypointSaverPanel() override;
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickRecord();
  void onClickCancel();

protected:
  static constexpr float INTERVAL_DEFAULT = 0.5;  // [m]

  QDoubleSpinBox * interval_input_{nullptr};
  QPushButton * save_button_{nullptr};
  QPushButton * finish_button_{nullptr};
  QLabel * file_label_{nullptr};

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Client<tier4_planning_msgs::srv::SaveWaypoint>::SharedPtr client_record_;

  void callServiceWithoutResponse(
    rclcpp::Client<tier4_planning_msgs::srv::SaveWaypoint>::SharedPtr client, bool && mode);

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

  void updateButtonState(bool mode)
  {
    if (mode) {
      deactivateButton(save_button_);
      activateButton(finish_button_);
    } else {
      deactivateButton(finish_button_);
      activateButton(save_button_);
    }
  }

  void restoreButtonState(bool mode)
  {
    QMetaObject::invokeMethod(
      this,
      [this, mode]() {
        if (mode) {
          activateButton(save_button_);
          deactivateButton(finish_button_);
        } else {
          activateButton(finish_button_);
          deactivateButton(save_button_);
        }
      },
      Qt::QueuedConnection);
  }
};
}  // namespace rviz_plugins

#endif  // WAYPOINT_SAVER_PANEL_HPP_