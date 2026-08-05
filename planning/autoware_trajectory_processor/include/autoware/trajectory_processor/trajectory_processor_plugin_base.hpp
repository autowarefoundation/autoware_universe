// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_

#include "autoware/trajectory_processor/trajectory_processor_context.hpp"
#include "autoware/trajectory_processor/trajectory_processor_data.hpp"
#include "autoware/trajectory_processor/trajectory_processor_parameters.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_processor::plugin
{

using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

enum class ProcessingResult { Unchanged, Modified };

class TrajectoryProcessorPluginBase
{
public:
  TrajectoryProcessorPluginBase() = default;
  virtual ~TrajectoryProcessorPluginBase() = default;

  TrajectoryProcessorPluginBase(const TrajectoryProcessorPluginBase &) = delete;
  TrajectoryProcessorPluginBase & operator=(const TrajectoryProcessorPluginBase &) = delete;
  TrajectoryProcessorPluginBase(TrajectoryProcessorPluginBase &&) = delete;
  TrajectoryProcessorPluginBase & operator=(TrajectoryProcessorPluginBase &&) = delete;

  void initialize(
    std::string class_name, std::string instance_name, rclcpp::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    class_name_ = std::move(class_name);
    instance_name_ = std::move(instance_name);
    short_name_ = get_unqualified_name(class_name_);
    node_ptr_ = node_ptr;
    time_keeper_ = std::move(time_keeper);
    context_ = std::move(context);

    RCLCPP_DEBUG(
      node_ptr_->get_logger(), "Instantiated trajectory processor plugin '%s' as '%s'",
      class_name_.c_str(), instance_name_.c_str());
    on_initialize(params);
  }

  void initialize(
    const std::string & class_name, rclcpp::Node * node_ptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<TrajectoryProcessorContext> context, const TrajectoryProcessorParams & params)
  {
    initialize(
      class_name, class_name, node_ptr, std::move(time_keeper), std::move(context), params);
  }

  virtual ProcessingResult process(
    TrajectoryPoints & trajectory_points, TrajectoryProcessorData & data) = 0;

  virtual void update_params(const TrajectoryProcessorParams & params) = 0;

  [[nodiscard]] const std::string & get_name() const { return instance_name_; }
  [[nodiscard]] const std::string & get_class_name() const { return class_name_; }
  [[nodiscard]] const std::string & get_instance_name() const { return instance_name_; }
  [[nodiscard]] const std::string & get_short_name() const { return short_name_; }

  virtual void publish_debug_data([[maybe_unused]] const std::string & ns) const {}

  virtual void publish_planning_factor()
  {
    if (planning_factor_interface_) {
      planning_factor_interface_->publish();
    }
  }

  [[nodiscard]] std::vector<PlanningFactor> get_planning_factors() const
  {
    if (planning_factor_interface_) {
      return planning_factor_interface_->get_factors();
    }
    return {};
  }

protected:
  virtual void on_initialize(const TrajectoryProcessorParams & params) = 0;

  [[nodiscard]] rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  [[nodiscard]] rclcpp::Clock::SharedPtr get_clock() const { return node_ptr_->get_clock(); }
  [[nodiscard]] std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const
  {
    return time_keeper_;
  }

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  std::shared_ptr<TrajectoryProcessorContext> context_;
  bool enabled_{true};
  double trajectory_time_step_{0.1};

private:
  static std::string get_unqualified_name(const std::string & name)
  {
    const auto separator = name.find_last_of(':');
    return separator == std::string::npos ? name : name.substr(separator + 1);
  }

  std::string class_name_{"unnamed_plugin"};
  std::string instance_name_{"unnamed_plugin"};
  std::string short_name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_processor::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_PLUGIN_BASE_HPP_
