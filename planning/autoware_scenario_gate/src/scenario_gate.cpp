#include "autoware/scenario_gate/scenario_gate.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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
    {"Default", "autoware::scenario_selector::DefaultScenarioSelectorNode"},
    {"Extra", "autoware::scenario_selector::ExtraScenarioSelectorNode"}};

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
      std::make_unique<pluginlib::ClassLoader<autoware::scenario_selector::ScenarioSelectorBase>>(
        "autoware_scenario_gate", "autoware::scenario_selector::ScenarioSelectorBase");

    selector_plugin_ = loader_->createSharedInstance(selector_info_);

    selector_node_raw_ = dynamic_cast<rclcpp::Node *>(selector_plugin_.get());
    if (!selector_node_raw_) {
      RCLCPP_FATAL(
        get_logger(), "Loaded plugin does not inherit rclcpp::Node; cannot spin in-process");
      throw std::runtime_error("Plugin is not an rclcpp::Node");
    }

    selector_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    selector_executor_->add_node(selector_node_raw_->get_node_base_interface());
    selector_spin_thread_ = std::thread([exec = selector_executor_]() { exec->spin(); });

    RCLCPP_INFO(get_logger(), "Selector component loaded and spinning");
  } catch (const pluginlib::LibraryLoadException & e) {
    RCLCPP_FATAL(get_logger(), "Failed to load selector component: %s", e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Exception while creating selector instance: %s", e.what());
    throw;
  }
}

ScenarioGateNode::~ScenarioGateNode()
{
  if (selector_executor_) {
    selector_executor_->cancel();
  }
  if (selector_spin_thread_.joinable()) {
    selector_spin_thread_.join();
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
