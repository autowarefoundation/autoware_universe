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

#ifndef ACTIVE_GENERATOR_PANEL_HPP_
#define ACTIVE_GENERATOR_PANEL_HPP_

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{

class ActiveGeneratorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ActiveGeneratorPanel(QWidget * parent = nullptr);
  ~ActiveGeneratorPanel() override;

protected:
  void onInitialize() override;

private:
  void processMessage(
    const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr msg);
  void updateLabel(const std::string & generator_name);
  void subscribe();
  void unsubscribe();

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>::SharedPtr
    subscription_;
  QLabel * generator_label_;
};

}  // namespace rviz_plugins

#endif  // ACTIVE_GENERATOR_PANEL_HPP_
