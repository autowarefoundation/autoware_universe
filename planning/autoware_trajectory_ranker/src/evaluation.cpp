// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_ranker/evaluation.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_ranker
{

void Evaluator::load_metric(
  const std::string & name, const size_t index, const double time_resolution)
{
  try {
    auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(vehicle_info_, time_resolution, node_ptr_);
    plugin->set_index(index);

    for (const auto & p : plugins_) {
      if (plugin->name() == p->name()) {
        RCLCPP_WARN_STREAM(logger_, "The plugin '" << name << "' is already loaded.");
        return;
      }
    }
    plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(logger_, "The scene plugin '" << name << "' is loaded.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "[ranker] createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "[ranker] unexpected exception for '" << name << "': " << e.what());
  }
}

void Evaluator::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<metrics::MetricInterface> plugin) { return plugin->name() == name; });

  if (it == plugins_.end()) {
    RCLCPP_WARN_STREAM(
      logger_, "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO_STREAM(logger_, "The scene plugin '" << name << "' is unloaded.");
  }
}

void Evaluator::evaluate(const std::vector<float> & max_value)
{
  for (const auto & result : results_) {
    for (const auto & plugin : plugins_) {
      if (plugin->index() < max_value.size()) {
        plugin->evaluate(result, max_value.at(plugin->index()));
      }
    }
  }
}

void Evaluator::normalize(const std::vector<std::vector<float>> & weight)
{
  if (results_.empty()) return;

  if (results_.size() < 2) {
    const auto data = results_.front();
    for (const auto & plugin : plugins_) {
      data->normalize(0.0f, data->score(plugin->index()), plugin->index());
    }
    return;
  }

  const auto range = [weight](const size_t index) {
    float min = 0.0f;
    float max = std::reduce(weight.at(index).begin(), weight.at(index).end());
    return std::make_pair(min, max);
  };

  for (const auto & plugin : plugins_) {
    const auto [min, max] = range(plugin->index());
    for (auto & data : results_) {
      data->normalize(min, max, plugin->index(), plugin->is_deviation());
    }
  }
}

void Evaluator::compress(const std::vector<std::vector<float>> & weight)
{
  std::for_each(
    results_.begin(), results_.end(), [&weight](auto & data) { data->compress(weight); });
}

void Evaluator::weighting(const std::vector<float> & weight)
{
  std::for_each(
    results_.begin(), results_.end(), [&weight](auto & data) { data->weighting(weight); });

  std::sort(results_.begin(), results_.end(), [](const auto & a, const auto & b) {
    return a->total() > b->total();
  });
}

auto Evaluator::get(const std::string & tag) const -> std::shared_ptr<DataInterface>
{
  const auto itr = std::find_if(
    results_.begin(), results_.end(), [&tag](const auto & data) { return data->tag() == tag; });

  return itr != results_.end() ? *itr : nullptr;
}

void Evaluator::add(const std::shared_ptr<CoreData> & core_data)
{
  const auto ptr = std::make_shared<DataInterface>(core_data, plugins_.size());
  results_.push_back(ptr);
}

void Evaluator::setup(const std::shared_ptr<TrajectoryPoints> & previous_points)
{
  for (auto & result : results_) {
    result->setup(previous_points);
  }
}

std::shared_ptr<DataInterface> Evaluator::best(
  const std::shared_ptr<EvaluatorParameters> & parameters, const std::string & exclude)
{
  evaluate(parameters->metrics_max_value);

  compress(parameters->time_decay_weight);

  normalize(parameters->time_decay_weight);

  weighting(parameters->score_weight);

  return best(exclude);
}

std::shared_ptr<DataInterface> Evaluator::best(const std::string & exclude) const
{
  if (results_.empty()) return nullptr;

  const auto itr = std::find_if(results_.begin(), results_.end(), [&exclude](const auto & result) {
    return result->tag() != exclude && result->feasible();
  });
  if (results_.end() == itr) return nullptr;

  return *itr;
}
}  // namespace autoware::trajectory_ranker
