#include "autoware/scenario_gate/scenario_gate.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <map>
#include <stdexcept>

namespace autoware::scenario_gate
{

ScenarioGateNode::ScenarioGateNode(const rclcpp::NodeOptions & options)
: Node("scenario_gate", options)
{
  const std::string selector_type = this->declare_parameter<std::string>("selector_type", "Default");

  static const std::map<std::string, std::string> selector_map = {
    {"Default", "autoware::scenario_selector::DefaultScenarioSelector"},
    {"Extra", "autoware::scenario_selector::ExtraScenarioSelector"}};

  if (selector_map.count(selector_type)) {
    selector_info_ = selector_map.at(selector_type);
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown selector_key: %s", selector_type.c_str());
    throw std::runtime_error("Invalid selector_key");
  }

  // Subscriptions
  sub_selector_scenario_ =
    this->create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
      "input/scenario", rclcpp::QoS{1},
      std::bind(&ScenarioGateNode::onSelectorScenario, this, std::placeholders::_1));

  sub_selector_traj_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioGateNode::onSelectorTrajectory, this, std::placeholders::_1));

  // Publishers
  pub_scenario_ = this->create_publisher<autoware_internal_planning_msgs::msg::Scenario>(
    "output/scenario", rclcpp::QoS{1});
  pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1});

  try {
    RCLCPP_INFO(get_logger(), "Loading selector plugin [%s]", selector_info_.c_str());

    loader_ =
      std::make_unique<pluginlib::ClassLoader<autoware::scenario_selector::ScenarioSelectorPlugin>>(
        "autoware_scenario_gate", "autoware::scenario_selector::ScenarioSelectorPlugin");

    selector_plugin_ = loader_->createSharedInstance(selector_info_);

    selector_plugin_->initialize(this);

    RCLCPP_INFO(get_logger(), "Selector plugin [%s] loaded and initialized", selector_info_.c_str());
  } catch (const pluginlib::LibraryLoadException & e) {
    RCLCPP_FATAL(get_logger(), "Failed to load selector plugin: %s", e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Exception while creating selector instance: %s", e.what());
    throw;
  }
}

void ScenarioGateNode::onSelectorScenario(
  const autoware_internal_planning_msgs::msg::Scenario::ConstSharedPtr msg)
{
  pub_scenario_->publish(*msg);
}

void ScenarioGateNode::onSelectorTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  pub_trajectory_->publish(*msg);
}

}  // namespace autoware::scenario_gate

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::scenario_gate::ScenarioGateNode)
