// Copyright 2021 Tier IV
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

#include "autoware/dummy_infrastructure/dummy_infrastructure_node.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

namespace autoware::dummy_infrastructure
{
namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

std::optional<InfrastructureCommandArray> findCommand(
  const InfrastructureCommandArray & command_array, const std::string & instrument_id,
  const bool use_first_command, const bool use_command_state)
{
  InfrastructureCommandArray array;
  bool found_flag = false;
  if (use_first_command && !command_array.commands.empty()) {
    array.commands.push_back(command_array.commands.front());
    return array;
  }

  if (use_command_state) {
    for (const auto & command : command_array.commands) {
      if (command.state >= InfrastructureCommand::REQUESTING) {
        array.commands.push_back(command);
        found_flag = true;
      }
    }
  }

  for (const auto & command : command_array.commands) {
    if (command.id == instrument_id) {
      array.commands.push_back(command);
      found_flag = true;
    }
  }

  return found_flag ? std::optional<InfrastructureCommandArray>{array} : std::nullopt;
}

std::optional<double> calcClosestStopLineDistance(
  const PlanningFactorArray::ConstSharedPtr & planning_factors)
{
  if (!planning_factors || planning_factors->factors.empty()) {
    return std::nullopt;
  }

  const auto vtl_it = std::find_if(
    planning_factors->factors.begin(), planning_factors->factors.end(),
    [](const auto & factor) { return factor.module == "virtual_traffic_light"; });

  if (vtl_it == planning_factors->factors.end()) {
    return std::nullopt;
  }

  std::vector<double> distances;
  distances.reserve(planning_factors->factors.size() * 2);

  for (const auto & factor : planning_factors->factors) {
    if (factor.module == "virtual_traffic_light") {
      for (const auto & control_point : factor.control_points) {
        distances.push_back(std::abs(control_point.distance));
      }
    }
  }

  if (distances.empty()) {
    return std::nullopt;
  }

  const auto min_it = std::min_element(distances.begin(), distances.end());
  return *min_it;
}

}  // namespace

DummyInfrastructureNode::DummyInfrastructureNode(const rclcpp::NodeOptions & node_options)
: Node("dummy_infrastructure", node_options)
{
  using std::placeholders::_1;
  // Parameter Server
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&DummyInfrastructureNode::onSetParam, this, _1));

  // Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz");
  node_param_.use_first_command = declare_parameter<bool>("use_first_command");
  node_param_.use_command_state = declare_parameter<bool>("use_command_state");
  node_param_.instrument_id = declare_parameter<std::string>("instrument_id");
  node_param_.approval = declare_parameter<bool>("approval");
  node_param_.is_finalized = declare_parameter<bool>("is_finalized");
  node_param_.auto_approval_mode = declare_parameter<bool>("auto_approval_mode", false);
  node_param_.stop_distance_threshold = declare_parameter<double>("stop_distance_threshold", 1.0);
  node_param_.stop_velocity_threshold = declare_parameter<double>("stop_velocity_threshold", 0.1);

  // Publisher
  pub_state_array_ = create_publisher<VirtualTrafficLightStateArray>("~/output/state_array", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&DummyInfrastructureNode::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult DummyInfrastructureNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Copy to local variable
    auto p = node_param_;

    // Update params
    update_param(params, "update_rate_hz", p.update_rate_hz);
    update_param(params, "use_first_command", p.use_first_command);
    update_param(params, "use_command_state", p.use_command_state);
    update_param(params, "instrument_id", p.instrument_id);
    update_param(params, "approval", p.approval);
    update_param(params, "is_finalized", p.is_finalized);
    update_param(params, "auto_approval_mode", p.auto_approval_mode);
    update_param(params, "stop_distance_threshold", p.stop_distance_threshold);
    update_param(params, "stop_velocity_threshold", p.stop_velocity_threshold);

    // Copy back to member variable
    node_param_ = p;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = "Exception occurred: " + std::string(e.what());
  } catch (...) {
    result.successful = false;
    result.reason = "Unknown exception occurred";
  }

  return result;
}

bool DummyInfrastructureNode::isDataReady()
{
  if (!command_array_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for command_array msg...");
    return false;
  }

  if (!planning_factors_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for planning_factors msg...");
    return false;
  }

  if (!current_odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for odometry msg...");
    return false;
  }

  return true;
}

std::optional<std::string> DummyInfrastructureNode::getCurrentCommandId() const
{
  if (!command_array_ || command_array_->commands.empty()) {
    return std::nullopt;
  }
  return command_array_->commands.front().id;
}

void DummyInfrastructureNode::onTimer()
{
  command_array_ = sub_command_array_.take_data();
  planning_factors_ = sub_planning_factors_.take_data();
  current_odometry_ = sub_odometry_.take_data();

  if (!isDataReady()) {
    return;
  }
  VirtualTrafficLightStateArray state_array;
  state_array.stamp = get_clock()->now();

  const auto found_command_array = findCommand(
    *command_array_, node_param_.instrument_id, node_param_.use_first_command,
    node_param_.use_command_state);

  if (!found_command_array) {
    VirtualTrafficLightState state;
    state.stamp = get_clock()->now();
    state.id = node_param_.instrument_id;
    state.type = "dummy_infrastructure";
    state.approval = node_param_.approval;
    state.is_finalized = node_param_.is_finalized;
    state_array.states.push_back(state);
  } else {
    const bool is_stopped =
      std::abs(current_odometry_->twist.twist.linear.x) < node_param_.stop_velocity_threshold;
    const auto min_distance_opt = calcClosestStopLineDistance(planning_factors_);
    const bool is_near_stop_line =
      min_distance_opt && *min_distance_opt <= node_param_.stop_distance_threshold;

    for (const auto & command : found_command_array->commands) {
      const auto [command_approval, command_is_finalized] =
        checkApprovalCommand(command.id, is_stopped, is_near_stop_line);
      if (command_approval && command_is_finalized) {
        if (approved_command_ids_.find(command.id) == approved_command_ids_.end()) {
          approved_command_ids_.insert(command.id);
          RCLCPP_INFO(get_logger(), "Approved new command ID %s", command.id.c_str());
        }
      }
      VirtualTrafficLightState state{};
      state.stamp = get_clock()->now();
      state.id = command.id;
      state.type = command.type;
      state.approval = command_approval;
      state.is_finalized = command_is_finalized;
      state_array.states.push_back(state);
    }
  }

  pub_state_array_->publish(state_array);
}

std::pair<bool, bool> DummyInfrastructureNode::checkApprovalCommand(
  const std::string & command_id, const bool is_stopped, const bool is_near_stop_line) const
{
  if (!node_param_.auto_approval_mode) {
    return {node_param_.approval, node_param_.is_finalized};
  }

  if (approved_command_ids_.find(command_id) != approved_command_ids_.end()) {
    return {true, true};
  }

  if (is_stopped && is_near_stop_line) {
    if (approved_command_ids_.find(command_id) == approved_command_ids_.end()) {
      RCLCPP_INFO(get_logger(), "Command ID %s meets approval conditions", command_id.c_str());
    }
    return {true, true};
  }

  return {false, false};
}

}  // namespace autoware::dummy_infrastructure

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::dummy_infrastructure::DummyInfrastructureNode)
