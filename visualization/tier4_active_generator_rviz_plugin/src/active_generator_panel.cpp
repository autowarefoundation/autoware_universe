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

#include "active_generator_panel.hpp"

#include <QColor>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <limits>
#include <string>

namespace
{

QString colorForGeneratorName(const std::string & name)
{
  int hash = 0;
  for (const char character : name) {
    hash = (hash * 31 + static_cast<unsigned char>(character)) % 360;
  }
  const int hue = (hash + 360) % 360;
  return QColor::fromHslF(hue / 360.0, 0.62, 0.52).name();
}

}  // namespace

namespace rviz_plugins
{

ActiveGeneratorPanel::ActiveGeneratorPanel(QWidget * parent)
: rviz_common::Panel(parent), subscription_(nullptr)
{
  auto * layout = new QVBoxLayout(this);

  generator_label_ = new QLabel(this);
  generator_label_->setAlignment(Qt::AlignCenter);
  generator_label_->setText("No data");
  generator_label_->setStyleSheet(
    "QLabel {"
    "  padding: 10px;"
    "  border-radius: 5px;"
    "  background-color: #333333;"
    "  color: white;"
    "  font-size: 14px;"
    "  font-weight: bold;"
    "}");

  layout->addWidget(generator_label_);
  setLayout(layout);
}

ActiveGeneratorPanel::~ActiveGeneratorPanel()
{
  unsubscribe();
}

void ActiveGeneratorPanel::onInitialize()
{
  subscribe();
}

void ActiveGeneratorPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void ActiveGeneratorPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void ActiveGeneratorPanel::processMessage(
  const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr msg)
{
  if (msg->scored_candidate_trajectories.empty()) {
    updateLabel("");
    return;
  }

  float best_score = -std::numeric_limits<float>::infinity();
  const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory * best = nullptr;
  for (const auto & scored : msg->scored_candidate_trajectories) {
    if (scored.score > best_score) {
      best_score = scored.score;
      best = &scored;
    }
  }

  if (best == nullptr) {
    updateLabel("");
    return;
  }

  const auto & best_uuid = best->candidate_trajectory.generator_id.uuid;
  std::string generator_name;
  for (const auto & info : msg->generator_info) {
    if (info.generator_id.uuid == best_uuid) {
      generator_name = info.generator_name.data;
      break;
    }
  }

  updateLabel(generator_name);
}

void ActiveGeneratorPanel::updateLabel(const std::string & generator_name)
{
  QString text;
  QString bg_color;

  if (generator_name.empty()) {
    text = "No data";
    bg_color = "#333333";  // Dark gray
  } else {
    text = QString::fromStdString(generator_name);
    bg_color = colorForGeneratorName(generator_name);
  }

  generator_label_->setText(text);
  generator_label_->setStyleSheet(QString(
                                    "QLabel {"
                                    "  padding: 10px;"
                                    "  border-radius: 5px;"
                                    "  background-color: %1;"
                                    "  color: white;"
                                    "  font-size: 14px;"
                                    "  font-weight: bold;"
                                    "}")
                                    .arg(bg_color));
}

void ActiveGeneratorPanel::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  try {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_ =
      node->create_subscription<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>(
        "/planning/trajectory_selector/ranker/scored_trajectories", 10,
        std::bind(&ActiveGeneratorPanel::processMessage, this, std::placeholders::_1));
  } catch (const std::exception & e) {
    generator_label_->setText("Error: Topic subscription failed");
    generator_label_->setStyleSheet(
      "QLabel {"
      "  padding: 10px;"
      "  border-radius: 5px;"
      "  background-color: #F44336;"
      "  color: white;"
      "  font-size: 14px;"
      "  font-weight: bold;"
      "}");
  }
}

void ActiveGeneratorPanel::unsubscribe()
{
  subscription_.reset();
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::ActiveGeneratorPanel, rviz_common::Panel)
